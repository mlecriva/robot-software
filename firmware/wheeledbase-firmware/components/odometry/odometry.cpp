/**
 * @file odometry.cpp
 * @author Mathis Lécrivain
 * @brief Odometry calculation object
 * @date 2022-06-14 (Creation)
 * 
 * @copyright (c) 2022
 * 
 */
#include "odometry.h"

#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"
#include "soc/soc.h"

/****************************************************************************************************
 *                                         Private defines
 ***************************************************************************************************/
#define NVS_PARTITION_NAME "storage"

#define ODOMETRY_TASK_STACK_SIZE (4096)
#define ODOMETRY_TASK_PRIORITY (1)

/****************************************************************************************************
 *                                       Private structures
 ***************************************************************************************************/
typedef struct {
    float axle_track;
    float slippage;
    uint32_t timestep;
} odometry_nvs_config_t;

/****************************************************************************************************
 *                                        Private variables
 ***************************************************************************************************/

/****************************************************************************************************
 *                                     Private function definition
 ***************************************************************************************************/
/**
 * @brief Odometry process task wrapper
 * 
 * @param pvParameters 
 */
static void process_task_wrapper(void* pvParameters)
{
    if (pvParameters != NULL) {
        static_cast<Odometry*>(pvParameters)->task();
    }
}

/****************************************************************************************************
 *                                     Class function definition
 ***************************************************************************************************/
void Odometry::start(void)
{
    /* Create internal task */
    if (xTaskCreatePinnedToCore(process_task_wrapper,
            "odometry_task",
            ODOMETRY_TASK_STACK_SIZE,
            this,
            ODOMETRY_TASK_PRIORITY,
            &_task_handle,
            APP_CPU_NUM)
        != pdPASS) {
        ESP_LOGE(_name, "Fail to create internal task!");
    }
}

void Odometry::task(void)
{
    TickType_t last_time;

    for (;;) {
        const float dL = _left_encoder->restart();
        const float dR = _right_encoder->restart();

        const float delta_lin_pos = (dL + dR) / 2;
        const float delta_orth_lin_pos = fabs(delta_lin_pos) * _slippage;
        const float delta_ang_pos = (dR - dL) / _axle_track;

        const float avg_theta = _pos.theta + delta_ang_pos / 2;
        _pos.x += delta_lin_pos * cos(avg_theta) - delta_orth_lin_pos * sin(avg_theta);
        _pos.y += delta_lin_pos * sin(avg_theta) + delta_orth_lin_pos * cos(avg_theta);
        _pos.theta += delta_ang_pos;

        /* Compute timestep value in second in order to get velocities in mm/s or rad/s */
        const float timestep_sec = ((float)_timestep) / 1000;

        _lin_velocity = delta_lin_pos / timestep_sec;
        _ang_velocity = delta_ang_pos / timestep_sec;

        ESP_LOGD(_name, "x : %f, y: %f, theta : %f", _pos.x, _pos.y, _pos.theta);

        vTaskDelayUntil(&last_time, pdMS_TO_TICKS(_timestep));
    }
}

void Odometry::load_config(void)
{
    std::shared_ptr<nvs::NVSHandle> handle;
    esp_err_t result;
    odometry_nvs_config_t config;

    /* Handle will automatically close when going out of scope or when it's reset. */
    handle = nvs::open_nvs_handle(NVS_PARTITION_NAME, NVS_READWRITE, &result);

    if (result != ESP_OK) {
        ESP_LOGE(_name, "Error (%s) opening NVS handle!", esp_err_to_name(result));
    } else {
        ESP_LOGI(_name, "Reading configuration from NVS ... ");

        result = handle->get_blob(_name, (void*)&config, sizeof(config));
        switch (result) {
        case ESP_OK:
            _axle_track = config.axle_track;
            _slippage = config.slippage;
            _timestep = config.timestep;

            ESP_LOGI(_name, "Loaded configuration :"
                            "\n\t -> axle_track : %f"
                            "\n\t -> slippage : %f"
                            "\n\t -> timestep : %d",
                _axle_track, _slippage, _timestep);

            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(_name, "Configuration is not initialized yet!");
            break;
        default:
            ESP_LOGE(_name, "Error (%s) reading!", esp_err_to_name(result));
        }
    }
}

void Odometry::save_config(void) const
{
    std::shared_ptr<nvs::NVSHandle> handle;
    esp_err_t result;

    odometry_nvs_config_t config = {
        .axle_track = _axle_track,
        .slippage = _slippage,
        .timestep = _timestep,
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
