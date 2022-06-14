/**
 * @file wheeledbase-firmware.cpp
 * @author Mathis Lécrivain
 * @brief 
 * @date 2022-06-07 (Creation)
 * 
 * @copyright (c) 2022
 * 
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include <stdio.h>

/****************************************************************************************************
 *                                         Private defines
 ***************************************************************************************************/

/****************************************************************************************************
 *                                          Private variable
 ***************************************************************************************************/
const static char* TAG = "main";

/****************************************************************************************************
 *                                          Private struct
 ***************************************************************************************************/

/****************************************************************************************************
 *                                          Private tab
 ***************************************************************************************************/

/****************************************************************************************************
 *                                     Private function prototypes
 ***************************************************************************************************/
static void main_init(void);
static void main_print_system_info(void);

/****************************************************************************************************
 *                                            Extern C
 ***************************************************************************************************/
extern "C" {
void app_main(void);
}

/****************************************************************************************************
 *                                     Public function definition
 ***************************************************************************************************/
/**
 * @brief Application main
 * 
 */
void app_main(void)
{
    main_print_system_info();

    main_init();

    ESP_LOGI(TAG, "Wheeledbase is successfully initialized!");

    while (1) {
    {
        vTaskDelay(portMAX_DELAY);
    }

}

/****************************************************************************************************
 *                                     Private function definition
 ***************************************************************************************************/
/**
 * @brief Initialize system controllers
 *
 */
static void main_init(void)
{
}

/**
 * @brief Display system informations
 *
 */
static void main_print_system_info(void)
{
    esp_chip_info_t chip_info;

    /* Print chip information */
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG,
        "This is %s chip with %d CPU core(s), WiFi%s%s, ",
        CONFIG_IDF_TARGET,
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "Silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG,
        "%dMB %s flash\n",
        spi_flash_get_chip_size() / (1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
}
