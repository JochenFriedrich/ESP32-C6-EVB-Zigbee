// (c) 2024 Jochen Friedrich

#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_err.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#define GPIO_WATER_SENSOR GPIO_NUM_1
#define GPIO_INPUT_SENSOR GPIO_NUM_2
#define GPIO_TAMPER_SENSOR GPIO_NUM_3
#define GPIO_BATTERY_SENSOR GPIO_NUM_15

#define GPIO_RELAY1 GPIO_NUM_10
#define GPIO_RELAY2 GPIO_NUM_11
#define GPIO_RELAY3 GPIO_NUM_22
#define GPIO_RELAY4 GPIO_NUM_23

#define IAS_ENDPOINT 1
#define RELAY1_ENDPOINT 11
#define RELAY2_ENDPOINT 12
#define RELAY3_ENDPOINT 13
#define RELAY4_ENDPOINT 14

#define LED_PIN GPIO_NUM_8

#define MAX_CHILDREN 10

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x14""Custom devices (DiY)"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x0f""ESP32-C6-EVB-ZW" /* Customized model identifier */
#define OTA_UPGRADE_QUERY_INTERVAL (1 * 60) // 1 minutes
#define OTA_UPGRADE_HW_VERSION              0x1                                     /* The parameter indicates the version of hardware */
#define OTA_UPGRADE_IMAGE_TYPE              0x0102
#define OTA_UPGRADE_MAX_DATA_SIZE           223                                     /* The recommended OTA image block size */
#define OTA_VERSION 0x00000009
#define FIRMWARE_VERSION "\x07""0.9-DEV"
#define FIRMWARE_DATE "\x08""20241101"


#define INSTALLCODE_POLICY_ENABLE   false /* enable the install code policy for security */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
  { .radio_mode = ZB_RADIO_MODE_NATIVE, }

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, }

static const char *TAG = "ZCL_UTILITY";

static const esp_partition_t *s_ota_partition = NULL;
static esp_ota_handle_t s_ota_handle = 0;
size_t ota_data_len_;
size_t ota_header_len_;
bool ota_upgrade_subelement_;
uint8_t ota_header_[6];
uint8_t blink;

// Zigbee Functions

static void zb_zdo_match_desc_handler(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        esp_zb_ota_upgrade_client_query_interval_set(IAS_ENDPOINT, OTA_UPGRADE_QUERY_INTERVAL);
        esp_zb_ota_upgrade_client_query_image_req(addr, endpoint);
        ESP_LOGI(TAG, "Query OTA upgrade from server endpoint: %d after %d seconds", endpoint, OTA_UPGRADE_QUERY_INTERVAL);
    } else {
        ESP_LOGW(TAG, "No OTA Server found");
    }
}

static void match_ota(void)
{
    esp_zb_zdo_match_desc_req_param_t req;
    uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};

    /* Match the OTA server of coordinator */
    req.addr_of_interest = 0x0000;
    req.dst_nwk_addr = 0x0000;
    req.num_in_clusters = 1;
    req.num_out_clusters = 0;
    req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    req.cluster_list = cluster_list;
    esp_zb_lock_acquire(portMAX_DELAY);
    if (esp_zb_bdb_dev_joined()) {
        esp_zb_zdo_match_cluster(&req, zb_zdo_match_desc_handler, NULL);
    }
    esp_zb_lock_release();
}
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
  esp_zb_zdo_signal_leave_params_t *leave_params = NULL;

  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      log_i("Zigbee stack initialized");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        log_i("Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
        if (esp_zb_bdb_is_factory_new()) {
          log_i("Start network formation");
          esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
          log_i("Device rebooted");
        }
      } else {
        /* commissioning failed */
        log_w("Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        log_i(
          "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
          extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1],
          extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address()
        );
        digitalWrite(LED_PIN,1);
        match_ota();
      } else {
        log_i("Network steering was not successful (status: %s)", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break; 
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
      leave_params = (esp_zb_zdo_signal_leave_params_t *) esp_zb_app_signal_get_params(p_sg_p);
      if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
        ESP_LOGI(TAG, "Reset device");
        esp_zb_factory_reset();
      } else {
        ESP_LOGI(TAG, "Leave_type: %u", leave_params->leave_type);
      }
      break;
    default: log_i("ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status)); break;
  }
}

static esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message)
{
    esp_err_t ret = ESP_OK;
    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Queried OTA image from address: 0x%04hx, endpoint: %d", message.server_addr.u.short_addr, message.server_endpoint);
        ESP_LOGI(TAG, "Image version: 0x%lx, manufacturer code: 0x%x, image size: %ld", message.file_version, message.manufacturer_code,
                 message.image_size);
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Approving OTA image upgrade");
    } else {
        ESP_LOGI(TAG, "Rejecting OTA image upgrade, status: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    static bool relay1_state = 0;
    static bool relay2_state = 0;
    static bool relay3_state = 0;
    static bool relay4_state = 0;

    
    if (!message) {
      log_e("Empty message");
      return ESP_FAIL;
    }
    if(!(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS)) {
      log_e("Received message: error status(%d)", message->info.status);
      return ESP_ERR_INVALID_ARG;
    }

    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY ) {
      switch (message->info.dst_endpoint) {
        case IAS_ENDPOINT:
          blink = 10;
          break;
        case RELAY1_ENDPOINT:
          blink = 2;
          break;
        case RELAY2_ENDPOINT:
          blink = 4;
          break;
        case RELAY3_ENDPOINT:
          blink = 6;
          break;
        case RELAY4_ENDPOINT:
          blink = 8;
          break;
        default:
          return ESP_FAIL;
      }
      return ESP_OK;
    }
    
    if (message->info.dst_endpoint == RELAY1_ENDPOINT) {
      if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
          relay1_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : relay1_state;
          digitalWrite(GPIO_RELAY1, relay1_state);
          return ESP_OK;
        }
      }
    }

    if (message->info.dst_endpoint == RELAY2_ENDPOINT) {
      if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
          relay2_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : relay2_state;
          digitalWrite(GPIO_RELAY2, relay2_state);
          return ESP_OK;
        }
      }
    }
    if (message->info.dst_endpoint == RELAY3_ENDPOINT) {
      if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
          relay3_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : relay3_state;
          digitalWrite(GPIO_RELAY3, relay3_state);
          return ESP_OK;
        }
      }
    }
    if (message->info.dst_endpoint == RELAY4_ENDPOINT) {
      if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
          relay4_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : relay4_state;
          digitalWrite(GPIO_RELAY4, relay4_state);
          return ESP_OK;
        }
      }
    }
 
    log_i("Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
      message->attribute.id, message->attribute.data.size);
   
    return ret;
}

static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message)
{
    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;
    esp_err_t ret = ESP_OK;
    size_t payload_size = 0;
    const uint8_t *payload = NULL;
    
    ESP_LOGI(TAG, "-- OTA upgrade called");
    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
      switch (message.upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(TAG, "-- OTA upgrade start");
       			ota_upgrade_subelement_ = false;
       			ota_data_len_ = 0;
            ota_header_len_ = 0;
            start_time = esp_timer_get_time();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            assert(s_ota_partition);
            ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
        	  payload_size = message.payload_size;
            payload = message.payload;

            total_size = message.ota_header.image_size;
            offset += payload_size;

            ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);

            /* Read and process the first sub-element, ignoring everything else */
			      while (ota_header_len_ < 6 && payload_size > 0) {
              ota_header_[ota_header_len_] = payload[0];
              ota_header_len_++;
				      payload++;
				      payload_size--;
			      }

			      if (!ota_upgrade_subelement_ && ota_header_len_ == 6) {
				      if (ota_header_[0] == 0 && ota_header_[1] == 0) {
					      ota_upgrade_subelement_ = true;
					      ota_data_len_ =
						      (((int)ota_header_[5] & 0xFF) << 24)
					      | (((int)ota_header_[4] & 0xFF) << 16)
						    | (((int)ota_header_[3] & 0xFF) << 8 )
						    |  ((int)ota_header_[2] & 0xFF);
					      ESP_LOGD(TAG, "OTA sub-element size %zu", ota_data_len_);
				      } else {
					      ESP_LOGE(TAG, "OTA sub-element type %02x%02x not supported", ota_header_[0], ota_header_[1]);
					      return ESP_FAIL;
				      }
			      }

            if (ota_data_len_) {
              payload_size = min(ota_data_len_, payload_size);
				      ota_data_len_ -= payload_size;

              if (message.payload_size && message.payload) {
                ret = esp_ota_write(s_ota_handle, payload, payload_size);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write OTA data to partition, status: %s", esp_err_to_name(ret));
              }
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "-- OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(TAG, "-- OTA Finish");
            ESP_LOGI(TAG, "-- OTA Information: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
                     message.ota_header.file_version, message.ota_header.manufacturer_code, message.ota_header.image_type,
                     message.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);
            ret = esp_ota_end(s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to end OTA partition, status: %s", esp_err_to_name(ret));
            ret = esp_ota_set_boot_partition(s_ota_partition);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set OTA boot partition, status: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Prepare to restart system");
            esp_restart();
            break;
        default:
            ESP_LOGI(TAG, "OTA status: %d", message.upgrade_status);
            break;
      }
    }
    return ret;
}

void send_notify(uint8_t state) {
  esp_zb_zcl_ias_zone_status_change_notif_cmd_t req = { };
  esp_zb_zcl_attr_t *id;
  esp_zb_zcl_attr_t *enrolledp;
  esp_zb_zcl_attr_t *address;

  address = esp_zb_zcl_get_attribute(IAS_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IAS_ZONE_IAS_CIE_ADDRESS_ID);
  id = esp_zb_zcl_get_attribute(IAS_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONEID_ID);

  req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
  req.extend_status = 0;
  req.zone_status = state;
  req.zone_id = *((uint8_t*) id->data_p);
      //req.zcl_basic_cmd.dst_addr_u.addr_short = 0;
  req.zcl_basic_cmd.dst_addr_u.addr_long[7] = ((uint8_t*)address->data_p)[7];
  req.zcl_basic_cmd.dst_addr_u.addr_long[6] = ((uint8_t*)address->data_p)[6];
  req.zcl_basic_cmd.dst_addr_u.addr_long[5] = ((uint8_t*)address->data_p)[5];
  req.zcl_basic_cmd.dst_addr_u.addr_long[4] = ((uint8_t*)address->data_p)[4];
  req.zcl_basic_cmd.dst_addr_u.addr_long[3] = ((uint8_t*)address->data_p)[3];
  req.zcl_basic_cmd.dst_addr_u.addr_long[2] = ((uint8_t*)address->data_p)[2];
  req.zcl_basic_cmd.dst_addr_u.addr_long[1] = ((uint8_t*)address->data_p)[1];
  req.zcl_basic_cmd.dst_addr_u.addr_long[0] = ((uint8_t*)address->data_p)[0];
  log_i("Sent to %x:%x:%x:%x:%x:%x:%x:%x", 
    ((uint8_t*)address->data_p)[7],
    ((uint8_t*)address->data_p)[6],
    ((uint8_t*)address->data_p)[5],
    ((uint8_t*)address->data_p)[4],
    ((uint8_t*)address->data_p)[3],
    ((uint8_t*)address->data_p)[2],
    ((uint8_t*)address->data_p)[1],
    ((uint8_t*)address->data_p)[0]);

  req.zcl_basic_cmd.src_endpoint = IAS_ENDPOINT;
  req.zcl_basic_cmd.dst_endpoint = 1;

  esp_zb_zcl_ias_zone_status_change_notif_cmd_req(&req);
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  esp_err_t ret = ESP_OK;

  log_i("Action");
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: 
      ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message); 
      break;
    case ESP_ZB_CORE_IAS_ZONE_ENROLL_RESPONSE_VALUE_CB_ID: 
      log_w("Enrolled");
      break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
        ret = zb_ota_upgrade_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
        break;
    default:
      log_w("Receive Zigbee action(0x%x) callback", callback_id); 
      break;
  }
  return ret;
}

static void esp_zb_task(void *pvParameters) {

  uint16_t status=0;
  uint32_t water_attr=ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_WATER_SENSOR;
  
  // Zigbee stack initialization
  esp_zb_platform_config_t platformConfig = {};
  platformConfig.radio_config.radio_mode = ZB_RADIO_MODE_NATIVE;
  platformConfig.host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE;
  ESP_ERROR_CHECK(esp_zb_platform_config(&platformConfig));

  esp_zb_cfg_t zb_nwk_cfg = {};
  zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER;
  zb_nwk_cfg.install_code_policy = INSTALLCODE_POLICY_ENABLE;
  zb_nwk_cfg.nwk_cfg.zczr_cfg.max_children = MAX_CHILDREN;
  esp_zb_init(&zb_nwk_cfg);

  esp_zb_ias_zone_cluster_cfg_t ias_cfg = {
    .zone_type=ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_WATER_SENSOR,
  //  .zone_status=1,
  };

  // Create Endpoint List
  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

  // Create Basic Cluster
  esp_zb_basic_cluster_cfg_t basic_cfg = {
    .zcl_version=ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    .power_source=1
  };
  esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *) ESP_MANUFACTURER_NAME);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *) ESP_MODEL_IDENTIFIER);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, (void *) FIRMWARE_VERSION);
  esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, (void *) FIRMWARE_DATE);
 
  // Create Identify Cluster
  esp_zb_identify_cluster_cfg_t identify_cfg = {
    .identify_time=ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);

  // Create IAS Zone Cluster
  esp_zb_attribute_list_t *esp_zb_ias_zone_cluster = esp_zb_ias_zone_cluster_create(&ias_cfg);
  
   // Create OTA Cluster
  esp_zb_ota_cluster_cfg_t ota_cfg = {
    .ota_upgrade_file_version=OTA_VERSION,
    .ota_upgrade_manufacturer=ESP_ZB_OTA_UPGRADE_MANUFACTURER_CODE_DEF_VALUE,
    .ota_upgrade_image_type=OTA_UPGRADE_IMAGE_TYPE,
    .ota_min_block_reque=ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_DEF_VALUE,
    .ota_upgrade_file_offset=ESP_ZB_ZCL_OTA_UPGRADE_FILE_OFFSET_DEF_VALUE,
    .ota_upgrade_downloaded_file_ver=ESP_ZB_ZCL_OTA_UPGRADE_DOWNLOADED_FILE_VERSION_DEF_VALUE,
    .ota_upgrade_server_id=ESP_ZB_ZCL_OTA_UPGRADE_SERVER_DEF_VALUE,
    .ota_image_upgrade_status=ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_DEF_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_ota_cluster = esp_zb_ota_cluster_create(&ota_cfg);
  esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_UPGRADE_HW_VERSION,
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
    };
  uint16_t ota_upgrade_server_addr = 0xffff;
  uint8_t ota_upgrade_server_ep = 0xff;
  esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config);
  esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, (void *)&ota_upgrade_server_addr);
  esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, (void *)&ota_upgrade_server_ep);

  // Create Cluster List
  esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_cluster_list, esp_zb_ias_zone_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list, esp_zb_ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

  // Create IAS Endpoint
  esp_zb_endpoint_config_t endpoint_config = {
    .endpoint = IAS_ENDPOINT, 
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
    .app_device_id = ESP_ZB_HA_IAS_ZONE_ID, 
    .app_device_version = 0
  };

  esp_zb_ep_list_add_ep(ep_list, esp_zb_cluster_list, endpoint_config);

 // Create Identify Cluster R1
  esp_zb_identify_cluster_cfg_t identify_cfg_r1 = {
    .identify_time=ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_identify_cluster_r1 = esp_zb_identify_cluster_create(&identify_cfg_r1);
   
  // Create Groups Cluster R1
  esp_zb_groups_cluster_cfg_t groups_cfg_r1 = {
    .groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_groups_cluster_r1 = esp_zb_groups_cluster_create(&groups_cfg_r1);

  // Create Scenes Cluster R1
  esp_zb_scenes_cluster_cfg_t scenes_cfg_r1 = {
    .scenes_count = ESP_ZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE,
    .current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,
    .current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,
    .scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,
    .name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_scenes_cluster_r1 = esp_zb_scenes_cluster_create(&scenes_cfg_r1);

  // Create OnOff Cluster R1
  esp_zb_on_off_cluster_cfg_t on_off_cfg_r1 = {
    .on_off=ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_on_off_cluster_r1 = esp_zb_on_off_cluster_create(&on_off_cfg_r1);
  
  // Create Cluster List R1
  esp_zb_cluster_list_t *esp_zb_cluster_list_r1 = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list_r1, esp_zb_identify_cluster_r1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list_r1, esp_zb_groups_cluster_r1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list_r1, esp_zb_scenes_cluster_r1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list_r1, esp_zb_on_off_cluster_r1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  // Create ON_OFF Endpoint R1
  esp_zb_endpoint_config_t endpoint_config_r1 = {
    .endpoint = RELAY1_ENDPOINT, 
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
    .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, 
    .app_device_version = 0
  };

  esp_zb_ep_list_add_ep(ep_list, esp_zb_cluster_list_r1, endpoint_config_r1);

 // Create Identify Cluster R2
  esp_zb_identify_cluster_cfg_t identify_cfg_r2 = {
    .identify_time=ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_identify_cluster_r2 = esp_zb_identify_cluster_create(&identify_cfg_r2);
   
  // Create Groups Cluster R2
  esp_zb_groups_cluster_cfg_t groups_cfg_r2= {
    .groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_groups_cluster_r2 = esp_zb_groups_cluster_create(&groups_cfg_r2);

  // Create Scenes Cluster R2
  esp_zb_scenes_cluster_cfg_t scenes_cfg_r2 = {
    .scenes_count = ESP_ZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE,
    .current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,
    .current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,
    .scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,
    .name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_scenes_cluster_r2 = esp_zb_scenes_cluster_create(&scenes_cfg_r2);

  // Create OnOff Cluster R2
  esp_zb_on_off_cluster_cfg_t on_off_cfg_r2 = {
    .on_off=ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_on_off_cluster_r2 = esp_zb_on_off_cluster_create(&on_off_cfg_r2);
  
  // Create Cluster List R1
  esp_zb_cluster_list_t *esp_zb_cluster_list_r2 = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list_r2, esp_zb_identify_cluster_r2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list_r2, esp_zb_groups_cluster_r2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list_r2, esp_zb_scenes_cluster_r2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list_r2, esp_zb_on_off_cluster_r2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  // Create ON_OFF Endpoint R2
  esp_zb_endpoint_config_t endpoint_config_r2 = {
    .endpoint = RELAY2_ENDPOINT, 
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
    .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, 
    .app_device_version = 0
  };

  esp_zb_ep_list_add_ep(ep_list, esp_zb_cluster_list_r2, endpoint_config_r2);

 // Create Identify Cluster R3
  esp_zb_identify_cluster_cfg_t identify_cfg_r3 = {
    .identify_time=ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_identify_cluster_r3 = esp_zb_identify_cluster_create(&identify_cfg_r3);
   
  // Create Groups Cluster R3
  esp_zb_groups_cluster_cfg_t groups_cfg_r3 = {
    .groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_groups_cluster_r3 = esp_zb_groups_cluster_create(&groups_cfg_r3);

  // Create Scenes Cluster R3
  esp_zb_scenes_cluster_cfg_t scenes_cfg_r3 = {
    .scenes_count = ESP_ZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE,
    .current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,
    .current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,
    .scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,
    .name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_scenes_cluster_r3 = esp_zb_scenes_cluster_create(&scenes_cfg_r3);

  // Create OnOff Cluster R3
  esp_zb_on_off_cluster_cfg_t on_off_cfg_r3 = {
    .on_off=ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_on_off_cluster_r3 = esp_zb_on_off_cluster_create(&on_off_cfg_r3);
  
  // Create Cluster List R3
  esp_zb_cluster_list_t *esp_zb_cluster_list_r3 = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list_r3, esp_zb_identify_cluster_r3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list_r3, esp_zb_groups_cluster_r3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list_r3, esp_zb_scenes_cluster_r3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list_r3, esp_zb_on_off_cluster_r3, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  // Create ON_OFF Endpoint R3
  esp_zb_endpoint_config_t endpoint_config_r3 = {
    .endpoint = RELAY3_ENDPOINT, 
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
    .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, 
    .app_device_version = 0
  };

  esp_zb_ep_list_add_ep(ep_list, esp_zb_cluster_list_r3, endpoint_config_r3);

 // Create Identify Cluster R4
  esp_zb_identify_cluster_cfg_t identify_cfg_r4 = {
    .identify_time=ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_identify_cluster_r4 = esp_zb_identify_cluster_create(&identify_cfg_r4);
   
  // Create Groups Cluster R4
  esp_zb_groups_cluster_cfg_t groups_cfg_r4 = {
    .groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_groups_cluster_r4 = esp_zb_groups_cluster_create(&groups_cfg_r4);

  // Create Scenes Cluster R4
  esp_zb_scenes_cluster_cfg_t scenes_cfg_r4 = {
    .scenes_count = ESP_ZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE,
    .current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,
    .current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,
    .scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,
    .name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_scenes_cluster_r4 = esp_zb_scenes_cluster_create(&scenes_cfg_r4);

  // Create OnOff Cluster R4
  esp_zb_on_off_cluster_cfg_t on_off_cfg_r4 = {
    .on_off=ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE
  };
  esp_zb_attribute_list_t *esp_zb_on_off_cluster_r4 = esp_zb_on_off_cluster_create(&on_off_cfg_r4);
  
  // Create Cluster List R4
  esp_zb_cluster_list_t *esp_zb_cluster_list_r4 = esp_zb_zcl_cluster_list_create();
  esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list_r4, esp_zb_identify_cluster_r4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list_r4, esp_zb_groups_cluster_r4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list_r4, esp_zb_scenes_cluster_r4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list_r4, esp_zb_on_off_cluster_r4, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  // Create ON_OFF Endpoint R4
  esp_zb_endpoint_config_t endpoint_config_r4 = {
    .endpoint = RELAY4_ENDPOINT, 
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
    .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, 
    .app_device_version = 0
  };

  esp_zb_ep_list_add_ep(ep_list, esp_zb_cluster_list_r4, endpoint_config_r4);

  esp_zb_device_register(ep_list);

  // Set Action Handler 
  esp_zb_core_action_handler_register(zb_action_handler);

  // Set Channel Mask
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

  // Erase NVRAM before creating connection to new Coordinator
  //esp_zb_nvram_erase_at_start(true);

  // Start Zigbee
  ESP_ERROR_CHECK(esp_zb_start(false));

  // Run Zugbee Stack
  esp_zb_main_loop_iteration();
}

// Standard Arduino Functions
void setup() {
  Serial.begin(115200);

  // Init Zigbee
  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  // GPIO Setup
  pinMode(GPIO_WATER_SENSOR, INPUT_PULLUP);
  pinMode(GPIO_INPUT_SENSOR, INPUT_PULLUP);
  pinMode(GPIO_TAMPER_SENSOR, INPUT_PULLUP);
  pinMode(GPIO_BATTERY_SENSOR, INPUT_PULLUP);
  pinMode(GPIO_RELAY1, OUTPUT);
  pinMode(GPIO_RELAY2, OUTPUT);
  pinMode(GPIO_RELAY3, OUTPUT);
  pinMode(GPIO_RELAY4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(GPIO_RELAY1, 0);
  digitalWrite(GPIO_RELAY2, 0);
  digitalWrite(GPIO_RELAY3, 0);
  digitalWrite(GPIO_RELAY4, 0);
  digitalWrite(LED_PIN, 0);

  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void loop() {
  static uint8_t old_state=255;
  uint8_t new_state;
  bool new_wstate, new_istate, new_tstate, new_bstate, enrolled;

  esp_zb_zcl_attr_t *enrolledp;

  new_wstate = digitalRead(GPIO_WATER_SENSOR);
  new_istate = digitalRead(GPIO_INPUT_SENSOR);
  new_tstate = digitalRead(GPIO_TAMPER_SENSOR);
  new_bstate = digitalRead(GPIO_BATTERY_SENSOR);

  new_state = (new_wstate?1:0) + (new_istate?2:0) + (new_tstate?4:0) + (new_bstate?8:0);

  if (old_state != new_state) {
    log_i("Status Changed");
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(IAS_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATUS_ID, &new_state, false);
    enrolledp = esp_zb_zcl_get_attribute(IAS_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_IAS_ZONE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_IAS_ZONE_ZONESTATE_ID);
    enrolled = *((uint8_t*) enrolledp->data_p);
    if (enrolled) {
      send_notify(new_state);
      old_state = new_state;
    } else {
      log_i("Not enrolled. Can't send notification");
    }
    esp_zb_lock_release();
  }
  
  if (blink>0) {
    if (blink&1) {
      digitalWrite(LED_PIN,1);
    } else {
      digitalWrite(LED_PIN,0);
    }
    blink--;
  }

  delay(1000);
}