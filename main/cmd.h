#ifndef CMD_H
#define CMD_H

#include "esp_gatts_api.h"
#include "freertos/FreeRTOS.h"  // Hinzufügen für FreeRTOS Basistypen
#include "freertos/task.h"      // Hinzufügen für TaskHandle_t

typedef enum {
    BLE_CONNECT_EVT,
    BLE_AUTH_EVT,
    BLE_WRITE_EVT,
    BLE_DISCONNECT_EVT,
    BLE_UART_EVT
} COMMAND;

#define SPP_DATA_MAX_LEN 512
#define PAYLOAD_SIZE (SPP_DATA_MAX_LEN - 20)  // Leave headroom for BLE overhead

typedef struct {
    uint8_t spp_event_id;
    uint16_t spp_conn_id;
    uint8_t spp_gatts_if;
    uint16_t length;
    uint8_t payload[SPP_DATA_MAX_LEN];
    // Add padding to ensure proper alignment
    uint8_t padding[2];
} __attribute__((packed, aligned(4))) CMD_t;

// Static assertions to validate buffer size assumptions at compile time
_Static_assert(SPP_DATA_MAX_LEN >= 20,
               "SPP_DATA_MAX_LEN must be at least 20 bytes");
_Static_assert(PAYLOAD_SIZE == (SPP_DATA_MAX_LEN - 20),
               "PAYLOAD_SIZE must be SPP_DATA_MAX_LEN - 20");
_Static_assert(sizeof(CMD_t) % 4 == 0, "CMD_t size must be 32-bit aligned");

#endif  // CMD_H