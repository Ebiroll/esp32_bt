/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_bt.h"
#include "hal/uhci_periph.h"
#include "driver/uart.h"
#include "esp_private/periph_ctrl.h" // for enabling UHCI module, remove it after UHCI driver is released
#include "esp_log.h"
#include "esp32_btdm.h"
#include "esp_bt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
extern int g_bt_plf_log_level;

static const char *tag = "CONTROLLER_UART_HCI";

static void basetimecnt_monitor_task(void *pvParameters)
{
    btdm_dev_t *btdm = (btdm_dev_t *)BTDM_PTR;
    uint32_t prev_cnt = 0;
    uint32_t curr_cnt = 0;
    
    ESP_LOGI(tag, "Starting BB_BASETIMECNT monitor task");
    
    // Read initial value
    prev_cnt = btdm->BLEBASETIMECNT.BASETIMECNT;
    
    while (1) {
        // Sleep for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Read current value
        curr_cnt = btdm->BLEBASETIMECNT.BASETIMECNT;
        
        // Calculate the difference (handle 27-bit wrap-around)
        uint32_t diff;
        if (curr_cnt >= prev_cnt) {
            diff = curr_cnt - prev_cnt;
        } else {
            // Counter wrapped around (27-bit counter max = 0x7FFFFFF)
            diff = (0x7FFFFFF - prev_cnt) + curr_cnt + 1;
        }
        
        ESP_LOGI(tag, "BB_BASETIMECNT: prev=0x%07"PRIx32" curr=0x%07"PRIx32" diff=%"PRIu32" (rate: %"PRIu32" Hz)",
                 prev_cnt, curr_cnt, diff, diff);
        
        prev_cnt = curr_cnt;
    }
}


void print_all_btdm_registers(void)
{
    btdm_dev_t *btdm = (btdm_dev_t *)BTDM_PTR;

    printf("BTDM Registers:\n");
    printf("BTCNTL: 0x%08"PRIx32"\n", btdm->BTCNTL);
    printf("BTVERSION: 0x%08"PRIx32"\n", btdm->BTVERSION.val);
    printf("BTINTCNTL: 0x%08"PRIx32"\n", btdm->BTINTCNTL);
    printf("BTINTSTAT: 0x%08"PRIx32"\n", btdm->BTINTSTAT);
    printf("BTINTRAWSTAT: 0x%08"PRIx32"\n", btdm->BTINTRAWSTAT);
    printf("BTINTACK: 0x%08"PRIx32"\n", btdm->BTINTACK);

    printf("BLECNTL: 0x%08"PRIx32"\n", btdm->BLECNTL.val);
    printf("BLEVERSION: 0x%08"PRIx32"\n", btdm->BLEVERSION.val);
    printf("BLECONF: 0x%08"PRIx32"\n", btdm->BLECONF);
    printf("BLEINTCNTL: 0x%08"PRIx32"\n", btdm->BLEINTCNTL.val);
    printf("BLEINTSTAT: 0x%08"PRIx32"\n", btdm->BLEINTSTAT);
    printf("BLEINTRAWSTAT: 0x%08"PRIx32"\n", btdm->BLEINTRAWSTAT);
    printf("BLEINTACK: 0x%08"PRIx32"\n", btdm->BLEINTACK);
    printf("BLEBASETIMECNT: 0x%08"PRIx32"\n", btdm->BLEBASETIMECNT.val);
    printf("BLEFINETIMECNT: 0x%08"PRIx32"\n", btdm->BLEFINETIMECNT.val);
    printf("BLEBDADDRL: 0x%08"PRIx32"\n", btdm->BLEBDADDRL);
    printf("BLEBDADDRU: 0x%08"PRIx32"\n", btdm->BLEBDADDRU.val);
    printf("BLECURRENTRXDESCPTR: 0x%08"PRIx32"\n", btdm->BLECURRENTRXDESCPTR.val);
    printf("BLEDIAGCNTL: 0x%08"PRIx32"\n", btdm->BLEDIAGCNTL);
    printf("BLEDIAGSTAT: 0x%08"PRIx32"\n", btdm->BLEDIAGSTAT);
    printf("BLEERRORTYPESTAT: 0x%08"PRIx32"\n", btdm->BLEERRORTYPESTAT);
    printf("BLERADIOCNTL0: 0x%08"PRIx32"\n", btdm->BLERADIOCNTL0);
    printf("BLERADIOCNTL1: 0x%08"PRIx32"\n", btdm->BLERADIOCNTL1);
    printf("BLERADIOPWRUPDN: 0x%08"PRIx32"\n", btdm->BLERADIOPWRUPDN);
    printf("BLEADVCHMAP: 0x%08"PRIx32"\n", btdm->BLEADVCHMAP.val);
    printf("BLEADVTIM: 0x%08"PRIx32"\n", btdm->BLEADVTIM);
    printf("BLEWLPUBADDRPTR: 0x%08"PRIx32"\n", btdm->BLEWLPUBADDRPTR);
    printf("BLEWLPRIVADDRPTR: 0x%08"PRIx32"\n", btdm->BLEWLPRIVADDRPTR);
    printf("BLEWLNBDEV: 0x%08"PRIx32"\n", btdm->BLEWLNBDEV);
    printf("BLEAESCNTL: 0x%08"PRIx32"\n", btdm->BLEAESCNTL.val);
    printf("BLEAESKEY[0]: 0x%08"PRIx32"\n", btdm->BLEAESKEY[0]);
    printf("BLEAESKEY[1]: 0x%08"PRIx32"\n", btdm->BLEAESKEY[1]);
    printf("BLEAESKEY[2]: 0x%08"PRIx32"\n", btdm->BLEAESKEY[2]);
    printf("BLEAESKEY[3]: 0x%08"PRIx32"\n", btdm->BLEAESKEY[3]);
    printf("BLEAESPTR: 0x%08"PRIx32"\n", btdm->BLEAESPTR);
    printf("BLERFTESTCNTL: 0x%08"PRIx32"\n", btdm->BLERFTESTCNTL.val);
    printf("BLERFTESTTXSTAT: 0x%08"PRIx32"\n", btdm->BLERFTESTTXSTAT);
    printf("BLERFTESTRXSTAT: 0x%08"PRIx32"\n", btdm->BLERFTESTRXSTAT);
    printf("BLETIMGENCNTL: 0x%08"PRIx32"\n", btdm->BLETIMGENCNTL.val);
    printf("BLECOEXIFCNTL0: 0x%08"PRIx32"\n", btdm->BLECOEXIFCNTL0);
    printf("BLERALPTR: 0x%08"PRIx32"\n", btdm->BLERALPTR);
    printf("BLERALNBDEV: 0x%08"PRIx32"\n", btdm->BLERALNBDEV);
}



static void uart_gpio_reset(void)
{
#if CONFIG_BTDM_CTRL_HCI_UART_NO == 1
    periph_module_enable(PERIPH_UART1_MODULE);
#elif CONFIG_BTDM_CTRL_HCI_UART_NO == 2
    periph_module_enable(PERIPH_UART2_MODULE);
#endif
    periph_module_enable(PERIPH_UHCI0_MODULE);

#ifdef CONFIG_BTDM_CTRL_HCI_UART_NO
    ESP_LOGI(tag, "HCI UART%d Pin select: TX 5, RX 18, CTS 23, RTS 19 Baudrate:%d", CONFIG_BTDM_CTRL_HCI_UART_NO, CONFIG_BTDM_CTRL_HCI_UART_BAUDRATE);

    uart_set_pin(CONFIG_BTDM_CTRL_HCI_UART_NO, 5, 18, 19, 23);
#endif
}

void app_main(void)
{
    esp_err_t ret;

    /* Initialize NVS — it is used to store PHY calibration data */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );


    /* As the UART1/2 pin conflict with flash pin, so do matrix of the signal and pin */
    uart_gpio_reset();

    // Set the logging level to debug
    g_bt_plf_log_level = ESP_LOG_DEBUG;


    print_all_btdm_registers();
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Bluetooth Controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    print_all_btdm_registers();

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Bluetooth Controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Create task to monitor BB_BASETIMECNT register rate
    xTaskCreate(basetimecnt_monitor_task, "basetimecnt_mon", 2048, NULL, 5, NULL);
}
