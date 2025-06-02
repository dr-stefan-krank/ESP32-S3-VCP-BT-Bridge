#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "cmd.h"
#include "driver/uart.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
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

using namespace esp_usb;

#include "driver/gpio.h"
#include "led_strip.h"

// GPIO assignment for LED strip
#define LED_STRIP_GPIO_PIN 48  // Dev Board GPIO for LED strip
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 1
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)
#define LED_BLINK_TIME 60

// These values should be the most common for USB-Serial devices
#define BAUDRATE (115200)
#define STOP_BITS (0)  // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define PARITY (0)     // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define DATA_BITS (8)

#define SPP_QUEUE_LEN 256
#define UART_QUEUE_LEN 256

QueueHandle_t xQueueSpp;
QueueHandle_t xQueueUartTX;

extern "C" void spp_task(void *arg);

static const char *TAG = "VCP example";
static SemaphoreHandle_t device_disconnected_sem;
SemaphoreHandle_t led_sync;  // LED synchronization semaphore

int led_rx = 0;   // USB-Serial RX activity
int led_tx = 0;   // USB-Serial TX activity
int led_vcp = 0;  // USB-Serial connection status
int led_ble = 0;  // BLE connection status

TickType_t last_ble_activity = 0;
#define BLE_TIMEOUT_MS \
    5000  // 5 seconds of inactivity before we consider BLE disconnected

static void bt_log(const char *data);
std::unique_ptr<CdcAcmDevice> vcp = nullptr;

/**
 * @brief Device event callback
 *
 * Handling device disconnection events
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event,
                         void *user_ctx) {
    char err_text[128];
    switch (event->type) {
        case CDC_ACM_HOST_ERROR:
            snprintf(err_text, sizeof(err_text),
                     ">>CDC-ACM error has occurred, err_no = %d",
                     event->data.error);
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_vcp = 0;
            xSemaphoreGive(led_sync);

            ESP_LOGE(TAG, "%s", err_text);
            bt_log(err_text);
            break;
        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            snprintf(err_text, sizeof(err_text),
                     ">>Device suddenly disconnected");
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_vcp = 0;
            xSemaphoreGive(led_sync);

            ESP_LOGI(TAG, "%s", err_text);
            bt_log(err_text);
            xSemaphoreGive(device_disconnected_sem);
            break;
        case CDC_ACM_HOST_SERIAL_STATE:
            snprintf(err_text, sizeof(err_text), "Serial state notif 0x%04X",
                     event->data.serial_state.val);
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_vcp = 0;
            xSemaphoreGive(led_sync);

            ESP_LOGI(TAG, "%s", err_text);
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
            // ESP_LOGI(TAG, "VCP device not open.");
            vTaskDelay(100);
            continue;
        }

        xQueueReceive(xQueueUartTX, &cmdBuf, portMAX_DELAY);
        BaseType_t err = vcp->tx_blocking(cmdBuf.payload, cmdBuf.length, 1000);
        while (err == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "uart_tx_task Timeout");
            vTaskDelay(15 / portTICK_PERIOD_MS);
            err = vcp->tx_blocking(cmdBuf.payload, cmdBuf.length, 1000);
        }

        BaseType_t sem = xSemaphoreTake(led_sync, 1);
        if (sem == pdTRUE) {
            led_tx = 1;
            xSemaphoreGive(led_sync);
        }
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
    size_t offset = 0;
    while (data_len > 0) {
        CMD_t cmdBuf;
        size_t chunk =
            data_len > SPP_DATA_MAX_LEN ? SPP_DATA_MAX_LEN : data_len;
        cmdBuf.spp_event_id = BLE_UART_EVT;
        cmdBuf.length = chunk;
        memcpy(cmdBuf.payload, data + offset, chunk);
        if (uxQueueSpacesAvailable(xQueueSpp) > 0) {
            BaseType_t err = xQueueSend(xQueueSpp, &cmdBuf, 0);
            if (err != pdTRUE) {
                ESP_LOGE(pcTaskGetName(NULL), "xQueueSend Fail");
            }
        } else {
            ESP_LOGW(pcTaskGetName(NULL),
                     "BLE TX queue full, dropping serial data");
            return false;
        }
        offset += chunk;
        data_len -= chunk;
    }
    last_ble_activity = xTaskGetTickCount();
    BaseType_t sem = xSemaphoreTake(led_sync, portMAX_DELAY);
    if (sem == pdTRUE) {
        led_rx = 1;
        xSemaphoreGive(led_sync);
    }
    return true;
}

static void vcp_open_task(void *arg) {
    // Do everything else in a loop, to handle USB device reconnections
    while (1) {
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms = BLE_TIMEOUT_MS,
            .out_buffer_size = 5120,
            .in_buffer_size = 5120,
            .event_cb = handle_event,
            .data_cb = handle_rx,
            .user_arg = NULL,
        };

        vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));

        if (vcp == nullptr) {
            // ESP_LOGI(TAG, "Failed to open VCP device");
            bt_log(">>Failed to open VCP device");
            continue;
        }
        vTaskDelay(10);

        cdc_acm_line_coding_t line_coding = {
            .dwDTERate = BAUDRATE,
            .bCharFormat = STOP_BITS,
            .bParityType = PARITY,
            .bDataBits = DATA_BITS,
        };
        ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));

        bt_log(">> Serial Device connected\n");

        xSemaphoreTake(led_sync, portMAX_DELAY);
        led_vcp = 1;
        xSemaphoreGive(led_sync);
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
        vTaskDelay(10);
    }
}

static void bt_log(const char *data) {
    size_t offset = 0;
    size_t data_len = strlen(data);
    while (data_len > 0) {
        CMD_t cmdBuf;
        size_t chunk =
            data_len > SPP_DATA_MAX_LEN ? SPP_DATA_MAX_LEN : data_len;
        cmdBuf.spp_event_id = BLE_UART_EVT;
        cmdBuf.length = chunk;
        memcpy(cmdBuf.payload, data + offset, chunk);
        BaseType_t err = xQueueSend(xQueueSpp, &cmdBuf, portMAX_DELAY);
        if (err != pdTRUE) {
            ESP_LOGE(pcTaskGetName(NULL), "xQueueSend Fail");
        }
        offset += chunk;
        data_len -= chunk;
    }
}

/**
 * @brief Configure the LED strip
 *
 * Initializes the LED strip with the specified GPIO pin, LED count, and RMT
 * configuration.
 *
 * @return led_strip_handle_t Handle to the initialized LED strip
 */
led_strip_handle_t configure_led(void) {
    // LED strip general initialization. It is only one on the dev board
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
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
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
    return led_strip;
}

/**
 * @brief LED task to control the LED strip based on USB-Serial and BLE activity
 *
 * This task updates the LED color based on the current activity state.
 * It uses a semaphore to synchronize access to the LED state variables.
 */
static void ledTask(void *arg) {
    int l_rx, l_tx, l_vcp, l_ble;
    led_strip_handle_t led_strip = configure_led();
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, 0));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    while (1) {
        xSemaphoreTake(led_sync, portMAX_DELAY);
        l_rx = led_rx;
        l_tx = led_tx;
        l_vcp = led_vcp;
        l_ble = led_ble;
        xSemaphoreGive(led_sync);

        uint8_t r = 0, g = 0, b = 0;
        if (l_ble) {             // BLE connected
            if (l_rx && l_tx) {  // Both RX and TX - yellow
                r = 100;
                g = 100;
                b = 0;
            } else if (l_rx) {  // RX only - red
                r = 100;
                g = 0;
                b = 0;
            } else if (l_tx) {  // TX only - green
                r = 0;
                g = 100;
                b = 0;
            } else {  // No traffic - purple
                r = 80;
                g = 0;
                b = 80;
            }
        } else if (l_vcp) {  // USB-Serial connected
            if (l_rx && l_tx) {
                r = 100;
                g = 100;
                b = 0;
            } else if (l_rx) {
                r = 100;
                g = 0;
                b = 0;
            } else if (l_tx) {
                r = 0;
                g = 100;
                b = 0;
            } else {
                r = 0;
                g = 0;
                b = 20;
            }
        } else {  // Nothing connected
            if (l_rx && l_tx) {
                r = 100;
                g = 100;
                b = 0;
            } else if (l_rx) {
                r = 100;
                g = 0;
                b = 0;
            } else if (l_tx) {
                r = 0;
                g = 100;
                b = 0;
            } else {
                r = 0;
                g = 0;
                b = 0;
            }
        }
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);

        xSemaphoreTake(led_sync, portMAX_DELAY);
        if (l_rx) led_rx = 0;
        if (l_tx) led_tx = 0;
        xSemaphoreGive(led_sync);
        vTaskDelay(LED_BLINK_TIME / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Initialize queues for SPP and UART communication
 *
 * This function allocates memory for the queues in PSRAM if available, or falls
 * back to internal RAM if PSRAM allocation fails.
 * Queues in PSRAM can be much bigger than in internal RAM, which is useful for
 * handling larger data transfers without blocking when BLE connection is slow.
 */
static void init_queues() {
    size_t cmd_size = sizeof(CMD_t);
    void *spp_queue_storage = nullptr;
    void *uart_queue_storage = nullptr;
    StaticQueue_t *spp_queue_struct = nullptr;
    StaticQueue_t *uart_queue_struct = nullptr;

    if (esp_psram_is_initialized()) {
        // Allocate queue storage buffers
        spp_queue_storage = heap_caps_malloc(
            SPP_QUEUE_LEN * cmd_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        uart_queue_storage = heap_caps_malloc(
            UART_QUEUE_LEN * cmd_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        // Allocate queue structures
        spp_queue_struct = (StaticQueue_t *)heap_caps_malloc(
            sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        uart_queue_struct = (StaticQueue_t *)heap_caps_malloc(
            sizeof(StaticQueue_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

        if (spp_queue_storage && uart_queue_storage && spp_queue_struct &&
            uart_queue_struct) {
            xQueueSpp = xQueueCreateStatic(SPP_QUEUE_LEN, cmd_size,
                                           (uint8_t *)spp_queue_storage,
                                           spp_queue_struct);
            xQueueUartTX = xQueueCreateStatic(UART_QUEUE_LEN, cmd_size,
                                              (uint8_t *)uart_queue_storage,
                                              uart_queue_struct);
            ESP_LOGI(TAG, "Queues allocated in PSRAM");
        } else {
            // Free any allocated memory before falling back
            if (spp_queue_storage) heap_caps_free(spp_queue_storage);
            if (uart_queue_storage) heap_caps_free(uart_queue_storage);
            if (spp_queue_struct) heap_caps_free(spp_queue_struct);
            if (uart_queue_struct) heap_caps_free(uart_queue_struct);

            ESP_LOGW(TAG,
                     "PSRAM allocation failed, falling back to internal RAM");
            xQueueSpp = xQueueCreate(32, cmd_size);
            xQueueUartTX = xQueueCreate(32, cmd_size);
        }
    } else {
        xQueueSpp = xQueueCreate(32, cmd_size);
        xQueueUartTX = xQueueCreate(32, cmd_size);
        ESP_LOGI(TAG, "Queues allocated in internal RAM");
    }

    configASSERT(xQueueSpp);
    configASSERT(xQueueUartTX);
}

/**
 * @brief Main application entry point
 *
 * Initializes NVS, installs USB Host driver, creates tasks, and starts the
 * application.
 */
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
    // ESP_LOGI(TAG, "Installing USB Host");
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    BaseType_t task_created =
        xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    assert(task_created == pdTRUE);

    // ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    // Register VCP drivers to VCP service
    VCP::register_driver<FT23x>();
    VCP::register_driver<CP210x>();
    VCP::register_driver<CH34x>();

    // Neue Queue-Initialisierung
    init_queues();

    // Start tasks
    xTaskCreate(uart_tx_task, "UART-TX", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(spp_task, "SPP", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(vcp_open_task, "VCP Open", 1024 * 4, NULL, 2, NULL);
    xTaskCreate(ledTask, "LED Task", 1024 * 4, NULL, 2, NULL);
}