/*
    BLE SPP Server Example

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
*/
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "cmd.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "SPP";

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56

#define DEVICE_NAME "ESP_SPP_SERVER"  // The Device Name Characteristics in GAP

#define SPP_SVC_INST_ID 0

#define FLOW_CONTROL_OFF 0x00
#define FLOW_CONTROL_ON 0x01

#define SPP_QUEUE_LEN 256
#define UART_QUEUE_LEN 256
#define FLOW_CHECK_INTERVAL_MS 50
#define QUEUE_SEND_TIMEOUT_MS 10

static bool ble_flow_paused = false;

/// Attributes State Machine
enum {
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NOTIFY_VAL,
    SPP_IDX_SPP_DATA_NOTIFY_CFG,

    SPP_IDX_NB,
};

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE 0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY 0xABF2

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02, 0x01, 0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03, 0x03, 0xF0, 0xAB,
    /* Complete Local Name in advertising */
    0x0F, 0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R', 'V', 'E',
    'R'};

extern QueueHandle_t xQueueSpp;
extern QueueHandle_t xQueueUartTX;

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;

static uint8_t test_manufacturer[3] = {'E', 'S', 'P'};

static uint8_t sec_service_uuid[16] = {
    /* LSB
       <-------------------------------------------------------------------------------->
       MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
};

// config adv data
static esp_ble_adv_data_t spp_adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0006,  // slave connection min interval, Time =
                             // min_interval * 1.25 msec
    .max_interval = 0x0006,  // slave connection max interval, Time =
                             // max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,        // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,  //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// config scan response data
static esp_ble_adv_data_t spp_scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(test_manufacturer),
    .p_manufacturer_data = test_manufacturer,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the
 * gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] =
        {
            .gatts_cb = gatts_profile_event_handler,
            .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is
                                             ESP_GATT_IF_NONE */
        },
};

/*
 *	SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid =
    ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify =
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t char_prop_read_write =
    ESP_GATT_CHAR_PROP_BIT_WRITE |  // Write with response
    ESP_GATT_CHAR_PROP_BIT_READ;    // Remove write without response for now

/// SPP Service - data receive characteristic, read & write with/without
/// response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t spp_data_receive_val[SPP_DATA_MAX_LEN] = {0x00};

/// SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t spp_data_notify_val[SPP_DATA_MAX_LEN] = {0x00};
static const uint8_t spp_data_notify_ccc[2] = {0x00, 0x00};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] = {
    // Service Declaration
    [SPP_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                     {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                      ESP_GATT_PERM_READ, sizeof(spp_service_uuid),
                      sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    // Data Receive Characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR] = {{ESP_GATT_AUTO_RSP},
                                    {ESP_UUID_LEN_16,
                                     (uint8_t *)&character_declaration_uuid,
                                     ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                                     CHAR_DECLARATION_SIZE,
                                     (uint8_t *)&char_prop_read_write}},

    // Data Receive Characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL] =
        {{ESP_GATT_RSP_BY_APP},  // Change from AUTO_RSP to RSP_BY_APP
         {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_DATA_MAX_LEN,
          sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    // Data Notify Characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
          ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
          (uint8_t *)&char_prop_read_notify}},

    // Data Notify Characteristic Value
    [SPP_IDX_SPP_DATA_NOTIFY_VAL] = {{ESP_GATT_AUTO_RSP},
                                     {ESP_UUID_LEN_16,
                                      (uint8_t *)&spp_data_notify_uuid,
                                      ESP_GATT_PERM_READ, SPP_DATA_MAX_LEN,
                                      sizeof(spp_data_notify_val),
                                      (uint8_t *)spp_data_notify_val}},

    // Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NOTIFY_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
         sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}}};

static char *esp_key_type_to_str(esp_ble_key_type_t key_type) {
    char *key_str = NULL;
    switch (key_type) {
        case ESP_LE_KEY_NONE:
            key_str = "ESP_LE_KEY_NONE";
            break;
        case ESP_LE_KEY_PENC:
            key_str = "ESP_LE_KEY_PENC";
            break;
        case ESP_LE_KEY_PID:
            key_str = "ESP_LE_KEY_PID";
            break;
        case ESP_LE_KEY_PCSRK:
            key_str = "ESP_LE_KEY_PCSRK";
            break;
        case ESP_LE_KEY_PLK:
            key_str = "ESP_LE_KEY_PLK";
            break;
        case ESP_LE_KEY_LLK:
            key_str = "ESP_LE_KEY_LLK";
            break;
        case ESP_LE_KEY_LENC:
            key_str = "ESP_LE_KEY_LENC";
            break;
        case ESP_LE_KEY_LID:
            key_str = "ESP_LE_KEY_LID";
            break;
        case ESP_LE_KEY_LCSRK:
            key_str = "ESP_LE_KEY_LCSRK";
            break;
        default:
            key_str = "INVALID BLE KEY TYPE";
            break;
    }

    return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req) {
    char *auth_str = NULL;
    switch (auth_req) {
        case ESP_LE_AUTH_NO_BOND:
            auth_str = "ESP_LE_AUTH_NO_BOND";
            break;
        case ESP_LE_AUTH_BOND:
            auth_str = "ESP_LE_AUTH_BOND";
            break;
        case ESP_LE_AUTH_REQ_MITM:
            auth_str = "ESP_LE_AUTH_REQ_MITM";
            break;
        case ESP_LE_AUTH_REQ_BOND_MITM:
            auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
            break;
        case ESP_LE_AUTH_REQ_SC_ONLY:
            auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
            break;
        case ESP_LE_AUTH_REQ_SC_BOND:
            auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
            break;
        case ESP_LE_AUTH_REQ_SC_MITM:
            auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
            break;
        case ESP_LE_AUTH_REQ_SC_MITM_BOND:
            auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
            break;
        default:
            auth_str = "INVALID BLE AUTH REQ";
            break;
    }

    return auth_str;
}

static void show_bonded_devices(void) {
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list =
        (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(__FUNCTION__, "Bonded devices number : %d", dev_num);
    ESP_LOGI(__FUNCTION__, "Bonded devices list : %d", dev_num);
    for (int i = 0; i < dev_num; i++) {
        esp_log_buffer_hex(__FUNCTION__, (void *)dev_list[i].bd_addr,
                           sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(__FUNCTION__, "GAP_EVT, event %d", event);
    CMD_t cmdBuf;
    BaseType_t err;

    switch (event) {
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&spp_adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&spp_adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            // advertising start complete event to indicate advertising start
            // successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(__FUNCTION__,
                         "advertising start failed, error status = %x",
                         param->adv_start_cmpl.status);
                break;
            }
            ESP_LOGI(__FUNCTION__, "advertising start success");
            break;
        case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
            /* Call the following function to input the passkey which is
             * displayed on the remote device */
            // esp_ble_passkey_reply(spp_profile_tab[SPP_PROFILE_APP_IDX].remote_bda,
            // true, 0x00);
            break;
        case ESP_GAP_BLE_OOB_REQ_EVT: {
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_OOB_REQ_EVT");
            uint8_t tk[16] = {1};  // If you paired with OOB, both devices need
                                   // to use the same tk
            esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk,
                                  sizeof(tk));
            break;
        }
        case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_LOCAL_IR_EVT");
            break;
        case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_LOCAL_ER_EVT");
            break;
        case ESP_GAP_BLE_NC_REQ_EVT:
            /* The app will receive this evt when the IO has DisplayYesNO
            capability and the peer device IO also has DisplayYesNo capability.
            show the passkey number to the user to confirm it with the number
            displayed by peer device. */
            esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
            ESP_LOGI(
                __FUNCTION__,
                "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32,
                param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            /* send the positive(true) security response to the peer device to
            accept the security request. If not accept the security request,
            should send the security response with negative(false) accept
            value*/
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  /// the app will receive this evt
                                             /// when the IO  has Output
                                             /// capability and the peer device
                                             /// IO has Input capability.
            /// show the passkey number to the user to input it in the peer
            /// device.
            ESP_LOGI(__FUNCTION__, "The passkey Notify number:%06" PRIu32,
                     param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_KEY_EVT:
            // shows the ble key info share with peer device to the user.
            ESP_LOGI(__FUNCTION__, "key type = %s",
                     esp_key_type_to_str(param->ble_security.ble_key.key_type));
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT: {
            esp_bd_addr_t bd_addr;
            memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr,
                   sizeof(esp_bd_addr_t));
            ESP_LOGI(__FUNCTION__, "remote BD_ADDR: %08x%04x",
                     (bd_addr[0] << 24) + (bd_addr[1] << 16) +
                         (bd_addr[2] << 8) + bd_addr[3],
                     (bd_addr[4] << 8) + bd_addr[5]);
            ESP_LOGI(__FUNCTION__, "address type = %d",
                     param->ble_security.auth_cmpl.addr_type);
            ESP_LOGI(
                __FUNCTION__, "pair status = %s",
                param->ble_security.auth_cmpl.success ? "success" : "fail");
            if (!param->ble_security.auth_cmpl.success) {
                ESP_LOGI(__FUNCTION__, "fail reason = 0x%x",
                         param->ble_security.auth_cmpl.fail_reason);
            } else {
                ESP_LOGI(__FUNCTION__, "auth mode = %s",
                         esp_auth_req_to_str(
                             param->ble_security.auth_cmpl.auth_mode));
            }
            show_bonded_devices();

            cmdBuf.spp_event_id = BLE_AUTH_EVT;
            err = xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
            if (err != pdTRUE) {
                ESP_LOGE(__FUNCTION__, "Failed to send to queue from ISR");
            }
            break;
        }
        case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
            ESP_LOGD(__FUNCTION__,
                     "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d",
                     param->remove_bond_dev_cmpl.status);
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_REMOVE_BOND_DEV");
            ESP_LOGI(__FUNCTION__, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
            esp_log_buffer_hex(__FUNCTION__,
                               (void *)param->remove_bond_dev_cmpl.bd_addr,
                               sizeof(esp_bd_addr_t));
            ESP_LOGI(__FUNCTION__, "------------------------------------");
            break;
        }
        case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
            if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(__FUNCTION__,
                         "config local privacy failed, error status = %x",
                         param->local_privacy_cmpl.status);
                break;
            }

            esp_err_t ret = esp_ble_gap_config_adv_data(&spp_adv_config);
            if (ret) {
                ESP_LOGE(__FUNCTION__,
                         "config adv data failed, error code = %x", ret);
            } else {
                adv_config_done |= ADV_CONFIG_FLAG;
            }

            ret = esp_ble_gap_config_adv_data(&spp_scan_rsp_config);
            if (ret) {
                ESP_LOGE(__FUNCTION__,
                         "config adv data failed, error code = %x", ret);
            } else {
                adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            }

            break;
        default:
            break;
    }
}

static uint8_t find_char_and_desr_index(uint16_t handle) {
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB; i++) {
        if (handle == spp_handle_table[i]) {
            return i;
        }
    }

    return error;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    CMD_t cmdBuf;
    BaseType_t err;

    ESP_LOGI(__FUNCTION__, "event = %d", event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(DEVICE_NAME);
            // generate a resolvable random address
            esp_ble_gap_config_local_privacy(true);
            esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data,
                                            sizeof(spp_adv_data));
            esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB,
                                          SPP_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            if (find_char_and_desr_index(param->read.handle) ==
                SPP_IDX_SPP_DATA_RECV_VAL) {
                ESP_LOGI(TAG, "ESP_GATTS_READ_EVT, handle = %d",
                         param->read.handle);
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->read.handle;
                rsp.attr_value.len = SPP_DATA_MAX_LEN;
                if (rsp.attr_value.len > ESP_GATT_MAX_ATTR_LEN) {
                    rsp.attr_value.len = ESP_GATT_MAX_ATTR_LEN;
                }
                memcpy(rsp.attr_value.value, spp_data_receive_val,
                       rsp.attr_value.len);
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                            param->read.trans_id, ESP_GATT_OK,
                                            &rsp);
            }
            break;
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(__FUNCTION__, "ESP_GATTS_WRITE_EVT");
            if (find_char_and_desr_index(param->write.handle) ==
                SPP_IDX_SPP_DATA_RECV_VAL) {
                if (param->write.len > PAYLOAD_SIZE) {
                    ESP_LOGE(__FUNCTION__, "Write data too long: %d > %d",
                             param->write.len, PAYLOAD_SIZE);
                    // Send error response
                    esp_gatt_rsp_t rsp;
                    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                    rsp.attr_value.handle = param->write.handle;
                    esp_ble_gatts_send_response(
                        gatts_if, param->write.conn_id, param->write.trans_id,
                        ESP_GATT_INVALID_ATTR_LEN, &rsp);
                    break;
                }

                // Process the write data with safe copy
                cmdBuf.spp_event_id = BLE_WRITE_EVT;
                cmdBuf.length = param->write.len;
                memcpy(cmdBuf.payload, param->write.value, cmdBuf.length);

                // Send to queue with timeout
                BaseType_t err =
                    xQueueSend(xQueueSpp, &cmdBuf, pdMS_TO_TICKS(100));

                // Always send response for write operations
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->write.handle;
                rsp.attr_value.len = 0;

                if (err == pdTRUE) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                                param->write.trans_id,
                                                ESP_GATT_OK, &rsp);
                } else {
                    // Queue full - send error response
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                                param->write.trans_id,
                                                ESP_GATT_CONGESTED, &rsp);
                }
            }
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT:
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(__FUNCTION__, "ESP_GATTS_MTU_EVT, MTU%d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(__FUNCTION__, "ESP_GATTS_CONNECT_EVT");
            /* start security connect with peer device when receive the connect
             * event sent by the master */
            esp_ble_set_encryption(param->connect.remote_bda,
                                   ESP_BLE_SEC_ENCRYPT_MITM);
            cmdBuf.spp_event_id = BLE_CONNECT_EVT;
            cmdBuf.spp_conn_id = p_data->connect.conn_id;
            cmdBuf.spp_gatts_if = gatts_if;
            err = xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
            if (err != pdTRUE) {
                ESP_LOGE(TAG, "xQueueSendFromISR Fail");
            }

            ESP_LOGI(__FUNCTION__, "ESP_GATTS_CONNECT_EVT, conn_id = %d",
                     param->connect.conn_id);
            esp_log_buffer_hex(__FUNCTION__, param->connect.remote_bda, 6);

            // --- BLE MTU and connection interval tuning ---
            // Set maximum possible MTU (up to 517 for BLE)
            esp_err_t mtu_ret = esp_ble_gatt_set_local_mtu(517);
            if (mtu_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set local MTU: %s",
                         esp_err_to_name(mtu_ret));
            } else {
                ESP_LOGI(TAG, "Requested BLE MTU 517");
            }

            // Tune connection interval for higher throughput
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda,
                   sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x10;    // 20ms (0x10 * 1.25ms)
            conn_params.min_int = 0x10;    // 20ms
            conn_params.timeout = 0x0C80;  // 4s
            esp_ble_gap_update_conn_params(&conn_params);

            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(__FUNCTION__,
                     "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x",
                     param->disconnect.reason);
            cmdBuf.spp_event_id = BLE_DISCONNECT_EVT;
            err = xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
            if (err != pdTRUE) {
                ESP_LOGE(TAG, "xQueueSendFromISR Fail");
            }
            /* start advertising again when missing the connect */
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGI(__FUNCTION__, "The number handle =%x",
                     param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(__FUNCTION__,
                         "Create attribute table failed, error code=0x%x",
                         param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != SPP_IDX_NB) {
                ESP_LOGE(__FUNCTION__,
                         "Create attribute table abnormally, num_handle (%d) "
                         "doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, SPP_IDX_NB);
            } else {
                memcpy(spp_handle_table, param->add_attr_tab.handles,
                       sizeof(spp_handle_table));
                esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
            }
            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(__FUNCTION__, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if ==
                    ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a
                                           certain gatt_if, need to call every
                                           profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void spp_init_queues(void) {
    size_t queue_len = 256;  // Large queue for bursty serial data
    size_t cmd_size = sizeof(CMD_t);

    // Check if PSRAM is available
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM detected. Allocating queues in PSRAM.");
        // Allocate queue storage in PSRAM
        void *spp_queue_storage = heap_caps_malloc(
            queue_len * cmd_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        void *uart_queue_storage = heap_caps_malloc(
            queue_len * cmd_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        if (spp_queue_storage && uart_queue_storage) {
            xQueueSpp = xQueueCreateStatic(SPP_QUEUE_LEN, sizeof(CMD_t),
                                           spp_queue_storage, NULL);
            if (xQueueSpp == NULL) {
                ESP_LOGE(TAG, "xQueueSpp creation failed!");
                abort();
            }
            xQueueUartTX = xQueueCreateStatic(queue_len, cmd_size,
                                              uart_queue_storage, NULL);
            ESP_LOGI(TAG, "Queues allocated in PSRAM, length=%d", queue_len);
        } else {
            ESP_LOGE(TAG,
                     "Failed to allocate queues in PSRAM, falling back to "
                     "normal RAM.");
            xQueueSpp = xQueueCreate(queue_len / 2, cmd_size);
            xQueueUartTX = xQueueCreate(queue_len / 2, cmd_size);
        }
    } else {
        ESP_LOGW(TAG, "No PSRAM detected. Using internal RAM for queues.");
        xQueueSpp = xQueueCreate(32, cmd_size);
        xQueueUartTX = xQueueCreate(32, cmd_size);
    }
}

void spp_task(void *arg) {
    ESP_LOGI(pcTaskGetName(0), "Start");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize BLE controller
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    CMD_t cmdBuf;
    uint16_t spp_conn_id = 0xffff;
    esp_gatt_if_t spp_gatts_if = 0xff;
    bool connected = false;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    const TickType_t flow_check_interval =
        FLOW_CHECK_INTERVAL_MS / portTICK_PERIOD_MS;  // 50ms
    TickType_t last_flow_check = xTaskGetTickCount();

    while (1) {
        // Critical section for flow control
        portENTER_CRITICAL(&mux);
        if (ble_flow_paused && connected) {
            if ((xTaskGetTickCount() - last_flow_check) >=
                flow_check_interval) {
                last_flow_check = xTaskGetTickCount();
                portEXIT_CRITICAL(&mux);

                if (uxQueueSpacesAvailable(xQueueUartTX) > 0) {
                    uint8_t flow_on = FLOW_CONTROL_ON;
                    esp_ble_gatts_send_indicate(
                        spp_gatts_if, spp_conn_id,
                        spp_handle_table[SPP_IDX_SPP_DATA_NOTIFY_VAL], 1,
                        &flow_on, false);

                    portENTER_CRITICAL(&mux);
                    ble_flow_paused = false;
                    portEXIT_CRITICAL(&mux);

                    ESP_LOGI(pcTaskGetName(NULL), "BLE flow control: ON");
                }
            } else {
                portEXIT_CRITICAL(&mux);
            }
        } else {
            portEXIT_CRITICAL(&mux);
        }

        // Handle queue messages with timeout
        if (xQueueReceive(xQueueSpp, &cmdBuf,
                          pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS)) == pdTRUE) {
            ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.spp_event_id=%d connected=%d",
                     cmdBuf.spp_event_id, connected);

            // Critical section for connection state changes
            portENTER_CRITICAL(&mux);
            if (cmdBuf.spp_event_id == BLE_CONNECT_EVT) {
                ESP_LOGI(pcTaskGetName(NULL), "BLE_CONNECT_EVT");
                spp_conn_id = cmdBuf.spp_conn_id;
                spp_gatts_if = cmdBuf.spp_gatts_if;
            } else if (cmdBuf.spp_event_id == BLE_AUTH_EVT) {
                ESP_LOGI(pcTaskGetName(NULL), "BLE_AUTH_EVT");
                connected = true;
            } else if (cmdBuf.spp_event_id == BLE_DISCONNECT_EVT) {
                ESP_LOGI(pcTaskGetName(NULL), "BLE_DISCONNECT_EVT");
                connected = false;
                ble_flow_paused = false;
            }
            portEXIT_CRITICAL(&mux);

            if (cmdBuf.spp_event_id == BLE_UART_EVT) {
                if (connected) {
                    ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload,
                                           cmdBuf.length, ESP_LOG_DEBUG);
                    // Check payload size before sending
                    if (cmdBuf.length > PAYLOAD_SIZE) {
                        ESP_LOGW(pcTaskGetName(NULL),
                                 "Payload too large (%d > %d), truncating",
                                 cmdBuf.length, PAYLOAD_SIZE);
                        cmdBuf.length = PAYLOAD_SIZE;
                    }
                    esp_ble_gatts_send_indicate(
                        spp_gatts_if, spp_conn_id,
                        spp_handle_table[SPP_IDX_SPP_DATA_NOTIFY_VAL],
                        cmdBuf.length, cmdBuf.payload, false);
                }
            } else if (cmdBuf.spp_event_id == BLE_WRITE_EVT) {
                ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload,
                                       cmdBuf.length, ESP_LOG_INFO);

                BaseType_t send_err =
                    xQueueSend(xQueueUartTX, &cmdBuf,
                               pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));
                if (send_err != pdTRUE) {
                    ESP_LOGW(TAG, "Queue full - implementing flow control");
                    // FLOW CONTROL: Notify client to pause sending
                    portENTER_CRITICAL(&mux);
                    if (!ble_flow_paused && connected) {
                        uint8_t flow_off = FLOW_CONTROL_OFF;
                        esp_ble_gatts_send_indicate(
                            spp_gatts_if, spp_conn_id,
                            spp_handle_table[SPP_IDX_SPP_DATA_NOTIFY_VAL], 1,
                            &flow_off, false);
                        ble_flow_paused = true;
                        ESP_LOGI(pcTaskGetName(NULL), "BLE flow control: OFF");
                    }
                    portEXIT_CRITICAL(&mux);
                }
            }
        }
    }  // end while

    // never reach here
    vTaskDelete(NULL);
}
