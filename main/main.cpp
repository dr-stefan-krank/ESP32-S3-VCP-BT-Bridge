#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"
#include "cmd.h"

using namespace esp_usb;

#include "driver/gpio.h"
#include "led_strip.h"

// GPIO assignment
#define LED_STRIP_GPIO_PIN 48
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 1
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)
#define LED_BLINK_TIME 60

// Change these values to match your needs
#define EXAMPLE_BAUDRATE (115200)
#define EXAMPLE_STOP_BITS (0)  // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY (0)     // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS (8)

QueueHandle_t xQueueSpp;
QueueHandle_t xQueueUartTX;


extern "C" void spp_task(void *arg);

static const char *TAG = "VCP example";
static SemaphoreHandle_t device_disconnected_sem;
static SemaphoreHandle_t led_sync;

int led_rx = 0;
int led_tx = 0;
int led_vcp = 0;

static void bt_log(const char *data);
std::unique_ptr<CdcAcmDevice> vcp = nullptr;

/**
 * @brief Device event callback
 *
 * Apart from handling device disconnection it doesn't do anything useful
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument we passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event,
                         void *user_ctx) {
    char *err_text = new char[128];
    switch (event->type) {
        case CDC_ACM_HOST_ERROR:
            snprintf(err_text, 128, ">>CDC-ACM error has occurred, err_no = %d",
                     event->data.error);
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_vcp = 0;
            xSemaphoreGive(led_sync);

            ESP_LOGE(TAG, "%s", err_text);
            bt_log(err_text);
            break;
        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            snprintf(err_text, 128, ">>Device suddenly disconnected");
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_vcp = 0;
            xSemaphoreGive(led_sync);

            //ESP_LOGI(TAG, "%s", err_text);
            bt_log(err_text);
            xSemaphoreGive(device_disconnected_sem);
            break;
        case CDC_ACM_HOST_SERIAL_STATE:
            snprintf(err_text, 128, "Serial state notif 0x%04X",
                     event->data.serial_state.val);
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_vcp = 0;
            xSemaphoreGive(led_sync);

            //ESP_LOGI(TAG, "%s", err_text);
            bt_log(err_text);
            break;
        case CDC_ACM_HOST_NETWORK_CONNECTION:
        default:
            break;
    }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg Unused
 */
static void usb_lib_task(void *arg) {
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}

static void uart_tx_task(void *pvParameters) {
    CMD_t cmdBuf;

    while (1) {
        if (vcp == nullptr) {
            //ESP_LOGI(TAG, "VCP device not open.");
            vTaskDelay(100);
            continue;
        }

        xQueueReceive(xQueueUartTX, &cmdBuf, portMAX_DELAY);
        ////ESP_LOGI(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
        // ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload,
        //                       cmdBuf.length, ESP_LOG_INFO);
        BaseType_t err = vcp->tx_blocking(cmdBuf.payload, cmdBuf.length, 1000);
        while (err == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "uart_tx_task Timeout");
            vTaskDelay(15/portTICK_PERIOD_MS);
            err = vcp->tx_blocking(cmdBuf.payload, cmdBuf.length, 1000);
        }

        BaseType_t sem = xSemaphoreTake(led_sync, 1);
        if (sem == pdTRUE) {
        led_tx = 1;
        xSemaphoreGive(led_sync);
        }
        //vTaskDelay(20/portTICK_PERIOD_MS);
    }  // end while
    // Never reach here
    vTaskDelete(NULL);
}

/**
 * @brief Data received callback
 *
 * Just pass received data to stdout
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg) {
    CMD_t cmdBuf;
    cmdBuf.spp_event_id = BLE_UART_EVT;
    cmdBuf.length = data_len;
    memcpy(cmdBuf.payload, data, data_len);

    BaseType_t err = xQueueSend(xQueueSpp, &cmdBuf, portMAX_DELAY);
    if (err != pdTRUE) {
        ESP_LOGE(pcTaskGetName(NULL), "xQueueSend Fail");
    }

    BaseType_t sem = xSemaphoreTake(led_sync, 1);
    if (sem == pdTRUE) {
      led_rx = 1;
      xSemaphoreGive(led_sync);
    }

    return true;
}

static void vcp_open_task(void *arg) {
    // Do everything else in a loop, so we can demonstrate USB device
    // reconnections
    while (1) {
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms =
                5000,  // 5 seconds, enough time to plug the device in or
                       // experiment with timeout
            .out_buffer_size = 5120,
            .in_buffer_size = 5120,
            .event_cb = handle_event,
            .data_cb = handle_rx,
            .user_arg = NULL,
        };

        // You don't need to know the device's VID and PID. Just plug in any
        // device and the VCP service will load correct (already registered)
        // driver for the device
        //ESP_LOGI(TAG, "Opening any VCP device...");

        vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));

        if (vcp == nullptr) {
            //ESP_LOGI(TAG, "Failed to open VCP device");
            bt_log(">>Failed to open VCP device");
            continue;
        }
        vTaskDelay(10);

        //ESP_LOGI(TAG, "Setting up line coding");
        cdc_acm_line_coding_t line_coding = {
            .dwDTERate = EXAMPLE_BAUDRATE,
            .bCharFormat = EXAMPLE_STOP_BITS,
            .bParityType = EXAMPLE_PARITY,
            .bDataBits = EXAMPLE_DATA_BITS,
        };
        ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));

        /*
        Now the USB-to-UART converter is configured and receiving data.
        You can use standard CDC-ACM API to interact with it. E.g.

        ESP_ERROR_CHECK(vcp->set_control_line_state(false, true));
        ESP_ERROR_CHECK(vcp->tx_blocking((uint8_t *)"Test string", 12));
        */

        bt_log(">> Serial Device connected\n");

        xSemaphoreTake(led_sync, portMAX_DELAY);
        led_vcp = 1;
        xSemaphoreGive(led_sync);

        // We are done. Wait for device disconnection and start over
        //ESP_LOGI(TAG, "Done. You can reconnect the VCP device to run again.");
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
        vTaskDelay(10);
    }
}

static void bt_log(const char *data) {
    char *log_text = new char[128];
    snprintf(log_text, 128, "%s\r\n", data);
    uint8_t data_len = strlen(log_text);
    CMD_t cmdBuf;
    cmdBuf.spp_event_id = BLE_UART_EVT;
    cmdBuf.length = data_len;
    memcpy(cmdBuf.payload, log_text, 128);

    BaseType_t err = xQueueSend(xQueueSpp, &cmdBuf, portMAX_DELAY);
    if (err != pdTRUE) {
        ESP_LOGE(pcTaskGetName(NULL), "xQueueSend Fail");
    }
}

led_strip_handle_t configure_led(void) {
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,  // The GPIO that connected to the
                                               // LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,  // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,    // LED strip model
        .color_component_format =
            LED_STRIP_COLOR_COMPONENT_FMT_GRB,  // The color order of the strip:
                                                // GRB
        .flags = {
            .invert_out = false,  // don't invert the output signal
        }};

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,  // different clock source can lead to
                                         // different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ,  // RMT counter clock frequency
        .mem_block_symbols =
            64,  // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma =
                true,  // DMA feature is available on chips like ESP32-S3/P4
        }};

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    //ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

static void ledTask(void *arg) {
    int l_rx, l_tx, l_vcp;
    led_strip_handle_t led_strip = configure_led();
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, 0));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    while (1) {
        xSemaphoreTake(led_sync, portMAX_DELAY);
        l_rx = led_rx;
        l_tx = led_tx;
        l_vcp = led_vcp;
        xSemaphoreGive(led_sync);

        if (l_rx and l_tx and l_vcp) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 100, 100, 0));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, 20));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        } else {
            if (l_rx and !(l_tx) and l_vcp) {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 100, 0, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, 20));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            } else {
                if (!(l_rx) and l_tx and l_vcp) {
                    ESP_ERROR_CHECK(
                        led_strip_set_pixel(led_strip, 0, 0, 100, 0));
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
                    ESP_ERROR_CHECK(
                        led_strip_set_pixel(led_strip, 0, 0, 0, 20));
                    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                } else {
                    if (!(l_rx) and !(l_tx) and l_vcp) {
                        ESP_ERROR_CHECK(
                            led_strip_set_pixel(led_strip, 0, 0, 0, 20));
                        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                        vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
                        ESP_ERROR_CHECK(
                            led_strip_set_pixel(led_strip, 0, 0, 0, 20));
                        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                    } else {
                        if (l_rx and l_tx and !(l_vcp)) {
                            ESP_ERROR_CHECK(
                                led_strip_set_pixel(led_strip, 0, 100, 100, 0));
                            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                            vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
                            ESP_ERROR_CHECK(
                                led_strip_set_pixel(led_strip, 0, 0, 0, 0));
                            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                        } else {
                            if (l_rx and !(l_tx) and !(l_vcp)) {
                                ESP_ERROR_CHECK(led_strip_set_pixel(
                                    led_strip, 0, 100, 0, 0));
                                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                                vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
                                ESP_ERROR_CHECK(
                                    led_strip_set_pixel(led_strip, 0, 0, 0, 0));
                                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                            } else {
                                if (!(l_rx) and l_tx and !(l_vcp)) {
                                    ESP_ERROR_CHECK(led_strip_set_pixel(
                                        led_strip, 0, 0, 100, 0));
                                    ESP_ERROR_CHECK(
                                        led_strip_refresh(led_strip));
                                    vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
                                    ESP_ERROR_CHECK(led_strip_set_pixel(
                                        led_strip, 0, 0, 0, 0));
                                    ESP_ERROR_CHECK(
                                        led_strip_refresh(led_strip));
                                } else {
                                    if (!(l_rx) and !(l_tx) and !(l_vcp)) {
                                        ESP_ERROR_CHECK(led_strip_set_pixel(
                                            led_strip, 0, 0, 0, 0));
                                        ESP_ERROR_CHECK(
                                            led_strip_refresh(led_strip));
                                        vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
                                        ESP_ERROR_CHECK(led_strip_set_pixel(
                                            led_strip, 0, 0, 0, 0));
                                        ESP_ERROR_CHECK(
                                            led_strip_refresh(led_strip));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        xSemaphoreTake(led_sync, portMAX_DELAY);
        if(l_rx) {
            led_rx = 0;
        }
        if(l_tx) {
            led_tx = 0;
        }
        xSemaphoreGive(led_sync);
        vTaskDelay(LED_BLINK_TIME/portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void) {
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    led_sync = xSemaphoreCreateMutex();

    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Install USB Host driver. Should only be called once in entire application
    //ESP_LOGI(TAG, "Installing USB Host");
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    BaseType_t task_created =
        xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    assert(task_created == pdTRUE);

    //ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    // Register VCP drivers to VCP service
    VCP::register_driver<FT23x>();
    VCP::register_driver<CP210x>();
    VCP::register_driver<CH34x>();

    // Create Queue
    xQueueSpp = xQueueCreate(10, sizeof(CMD_t));
    configASSERT(xQueueSpp);
    xQueueUartTX = xQueueCreate(10, sizeof(CMD_t));
    configASSERT(xQueueUartTX);

    // Start tasks
    xTaskCreate(uart_tx_task, "UART-TX", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(spp_task, "SPP", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(vcp_open_task, "VCP Open", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(ledTask, "LED Task", 1024 * 4, NULL, 2, NULL);
}