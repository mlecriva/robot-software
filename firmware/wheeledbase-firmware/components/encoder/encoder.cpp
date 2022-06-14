/**
 * @file encoder.cpp
 * @author Mathis Lécrivain
 * @brief Encoder object
 * @date 2022-06-07 (Creation)
 *
 * @copyright (c) 2022
 *
 */

#include "encoder.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"
#include "sys/lock.h"

/****************************************************************************************************
 *                                         Private defines
 ***************************************************************************************************/
#define ENCODER_PCNT_LOW_LIMIT (-2500)
#define ENCODER_PCNT_HIGH_LIMIT (2500)

#define NVS_PARTITION_NAME "storage"

/****************************************************************************************************
 *                                       Private structures
 ***************************************************************************************************/
typedef struct {
    float wheel_radius;
    int64_t counts_per_rev;
} encoder_nvs_config_t;

/****************************************************************************************************
 *                                        Private variables
 ***************************************************************************************************/
/* A lock to avoid PCNT isr service being installed twice in multiple threads. */
static _lock_t isr_service_install_lock;
#define LOCK_ACQUIRE() _lock_acquire(&isr_service_install_lock)
#define LOCK_RELEASE() _lock_release(&isr_service_install_lock)

static bool _is_pcnt_isr_service_installed = false;

/****************************************************************************************************
 *                                     Private function definition
 ***************************************************************************************************/
/**
 * @brief PCNT overflow handler
 * 
 * @param arg 
 */
static void pcnt_overflow_handler(void* arg)
{
    if (arg != NULL) {
        Encoder* instance = (Encoder*)arg;
        uint32_t status = 0;
        pcnt_get_event_status(instance->get_pcnt_unit(), &status);

        if (status & PCNT_EVT_H_LIM) {
            instance->overflow_counter_add(ENCODER_PCNT_HIGH_LIMIT);
        } else if (status & PCNT_EVT_L_LIM) {
            instance->overflow_counter_add(ENCODER_PCNT_LOW_LIMIT);
        }
    }
}

/****************************************************************************************************
 *                                     Class function definition
 ***************************************************************************************************/
void Encoder::attach_counter(pcnt_unit_t unit, gpio_num_t phase_a_gpio, gpio_num_t phase_b_gpio)
{
    pcnt_config_t dev_config;

    /* Save PCNT unit */
    _unit = unit;

    /* Global PCNT config */
    dev_config.unit = _unit;
    dev_config.lctrl_mode = PCNT_MODE_REVERSE;
    dev_config.hctrl_mode = PCNT_MODE_KEEP;
    dev_config.counter_l_lim = ENCODER_PCNT_LOW_LIMIT;
    dev_config.counter_h_lim = ENCODER_PCNT_HIGH_LIMIT;

    /* Channel 0 PCNT config */
    dev_config.pulse_gpio_num = phase_a_gpio;
    dev_config.ctrl_gpio_num = phase_b_gpio;
    dev_config.channel = PCNT_CHANNEL_0;
    dev_config.pos_mode = PCNT_COUNT_DEC;
    dev_config.neg_mode = PCNT_COUNT_INC;

    if (pcnt_unit_config(&dev_config) != ESP_OK) {
        ESP_LOGE(_name, "Failed to configure PCNT channel 0!");
    }

    /* Channel 1 PCNT config */
    dev_config.pulse_gpio_num = phase_b_gpio;
    dev_config.ctrl_gpio_num = phase_a_gpio;
    dev_config.channel = PCNT_CHANNEL_1;
    dev_config.pos_mode = PCNT_COUNT_INC;
    dev_config.neg_mode = PCNT_COUNT_DEC;

    if (pcnt_unit_config(&dev_config) != ESP_OK) {
        ESP_LOGE(_name, "Failed to configure PCNT channel 1!");
    }

    /* PCNT pause and reset value */
    if (pcnt_counter_pause(_unit) != ESP_OK) {
        ESP_LOGE(_name, "Failed to pause counter!");
    }
    if (pcnt_counter_clear(_unit) != ESP_OK) {
        ESP_LOGE(_name, "Failed to clear counter!");
    }

    /* register interrupt handler in a thread-safe way */
    LOCK_ACQUIRE();
    if (!_is_pcnt_isr_service_installed) {
        pcnt_isr_service_install(0);
        /* make sure pcnt isr service won't be installed more than one time */
        _is_pcnt_isr_service_installed = true;
    }
    LOCK_RELEASE();

    if (pcnt_isr_handler_add(_unit, pcnt_overflow_handler, this) != ESP_OK) {
        ESP_LOGE(_name, "Failed to add overflow handler!");
    }

    if (pcnt_event_enable(_unit, PCNT_EVT_H_LIM) != ESP_OK) {
        ESP_LOGE(_name, "Failed to enable high limit event");
    }
    if (pcnt_event_enable(_unit, PCNT_EVT_L_LIM) != ESP_OK) {
        ESP_LOGE(_name, "Failed to enable low limit event!");
    }

    if (pcnt_counter_resume(_unit)) {
        ESP_LOGE(_name, "Failed to resume counter!");
    }

    ESP_LOGI(_name, "Successfully initialized!");
}

int64_t Encoder::get_counter(void)
{
    int16_t count = 0;

    pcnt_get_counter_value(_unit, &count);

    _current_counter = count + _overflow_counter;

    return _current_counter;
}

void Encoder::set_counts_per_revolution(int64_t value)
{
    _counts_per_rev = value;
}

void Encoder::set_wheel_radius(float value)
{
    _wheel_radius = value;
}

void Encoder::reset(void)
{
    if (pcnt_counter_clear(_unit) != ESP_OK) {
        ESP_LOGE(_name, "Failed to clear counter!");
    }
    _start_counter = 0;
    _overflow_counter = 0;
}

float Encoder::get_traveled_distance(void)
{
    return (float)(get_counter() - _start_counter) / _counts_per_rev * (2.0 * M_PI * _wheel_radius);
}

float Encoder::restart(void)
{
    float distance = get_traveled_distance();
    _start_counter = _current_counter;
    return distance;
}

void Encoder::load_config(void)
{
    std::shared_ptr<nvs::NVSHandle> handle;
    esp_err_t result;
    encoder_nvs_config_t config;

    /* Handle will automatically close when going out of scope or when it's reset. */
    handle = nvs::open_nvs_handle(NVS_PARTITION_NAME, NVS_READWRITE, &result);

    if (result != ESP_OK) {
        ESP_LOGE(_name, "Error (%s) opening NVS handle!", esp_err_to_name(result));
    } else {
        ESP_LOGI(_name, "Reading configuration from NVS ... ");

        result = handle->get_blob(_name, (void*)&config, sizeof(config));
        switch (result) {
        case ESP_OK:
            _wheel_radius = config.wheel_radius;
            _counts_per_rev = config.counts_per_rev;

            ESP_LOGI(_name, "Loaded configuration :"
                            "\n\t -> wheel_radius : %f"
                            "\n\t -> counts_per_rev : %lld",
                _wheel_radius, _counts_per_rev);

            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(_name, "Configuration is not initialized yet!");
            break;
        default:
            ESP_LOGE(_name, "Error (%s) reading!", esp_err_to_name(result));
        }
    }
}

void Encoder::save_config(void) const
{
    std::shared_ptr<nvs::NVSHandle> handle;
    esp_err_t result;

    encoder_nvs_config_t config = {
        .wheel_radius = _wheel_radius,
        .counts_per_rev = _counts_per_rev,
    };

    /* Handle will automatically close when going out of scope or when it's reset. */
    handle = nvs::open_nvs_handle(NVS_PARTITION_NAME, NVS_READWRITE, &result);

    if (result != ESP_OK) {
        ESP_LOGE(_name, "Error (%s) opening NVS handle!", esp_err_to_name(result));
    } else {
        ESP_LOGI(_name, "Updating configuration in NVS ... ");
        result = handle->set_blob(_name, (const void*)&config, sizeof(config));
        if (result != ESP_OK) {
            ESP_LOGE(_name, "Failed!");
        }

        ESP_LOGI(_name, "Committing updates in NVS ... ");
        result = handle->commit();
        if (result != ESP_OK) {
            ESP_LOGE(_name, "Failed!");
        } else {
            ESP_LOGI(_name, "Done!");
        }
    }
}
