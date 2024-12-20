#include "kilovault_bms_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"


/* MAP
  1.0 KilovaultBmsBle::gattc_event_handler()
    - Main Entry Point
  2.0 KilovaultBmsBle::assemble_()
    2.1 crc()
    2.2 KilovaultBMSBle::ascii_to_int_() should this stay as lambda??
    2.3 Hand off to on_kilovault_bms_ble_data_()
  3.0 KilovaultBmsBle::Update()
    3.1 Hand off to decode_status_data_()
  4.0 decode_status_data_()
    4.1 kilovault_get_16bit() lambda
    4.2 kilovault_get_32bit() lambda
    4.3 Publishes Data
    4.4 decode_cell_voltages_data_()
      4.4.1 decode_cell_voltages_data_()
      4.4.2 Publishes Data
*/

namespace esphome {
namespace kilovault_bms_ble {

static const char *const TAG = "kilovault_bms_ble";

//static const uint16_t KILOVAULT_BMS_SERVICE_UUID = 0xFA00;
static const uint16_t KILOVAULT_BMS_SERVICE_UUID = 0xFFE0;
static const uint16_t KILOVAULT_BMS_NOTIFY_CHARACTERISTIC_UUID = 0xFFE4;   // handle 0x12

static const uint16_t KILOVAULT_BMS_CONTROL_CHARACTERISTIC_UUID = 0xFA02;  // handle 0x15

static const uint16_t MAX_RESPONSE_SIZE = 121;

static const uint8_t KILOVAULT_PKT_START_A = 0xB0;
static const uint8_t KILOVAULT_PKT_START_B = 0xB0;
static const uint8_t KILOVAULT_ADDRESS = 0x16;
static const uint8_t KILOVAULT_PKT_END_1 = 0x52;
static const uint8_t KILOVAULT_PKT_END_2 = 0x52;



/* ========================================================================= */
/* bool crc(const std::vector<uint8_t> &data)
    This function is used to check the CRC of the data. 
    The data is passed as a vector of uint8_t. 
    The function returns a boolean value. 
    If the CRC check fails, the function returns false. 

    It is called inside the assemble_() function and that is the only place.

*/
bool crc(const std::vector<uint8_t> &data) {
  auto kilovault_get_8bit = [&](size_t i) -> uint8_t {
    return ((uint8_t(data[i + 0]) << 4) | (uint8_t(data[i + 1]) << 0));
  };
  const uint16_t frame_size = 110; 
  uint16_t crc = 0;

  //ESP_LOGW(TAG, "CRC data: %s", format_hex_pretty(data).c_str());

  uint16_t remote_crc = (kilovault_get_8bit(frame_size-1) << 8) + kilovault_get_8bit(frame_size+1);
  //ESP_LOGW(TAG, "Remote CRC  0x%04X", remote_crc);

  for (uint16_t i = 1; i < frame_size - 2; i+=2) {
    crc = crc + kilovault_get_8bit(i);
    //ESP_LOGW(TAG, "CRC  0x%04X", crc);
  }
  //ESP_LOGW(TAG, "CRC  0x%04X", crc);

  if (crc != remote_crc) {
    ESP_LOGW(TAG, "CRC check failed! 0x%02X != 0x%02X", crc, remote_crc);
    return false;
  }
  return true;
}

/* GATT:  Generic Attributes. Is the name of the interface used to connect to BTLE devices. 

          The gattc_event_handler() is from the ESP32 BLE API and is called when a GATT event occurs. 
*/
/* ========================================================================= */
/* void KilovaultBmsBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
    This function is used to handle the GATT events. 
    The function is called when a GATT event occurs. 
    The function is called with the following parameters: 
      - event: The event that has occurred. 
      - gattc_if: The interface used to connect to the BTLE device. 
      - param: The parameters of the GATT event. 

    This function is the entry point

*/
void KilovaultBmsBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                      esp_ble_gattc_cb_param_t *param) {
  switch (event) {

    case ESP_GATTC_OPEN_EVT: {  // ESP_GATTC_OPEN_EVT:  Event when a connection to a BLE device is opened.
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {  // ESP_GATTC_DISCONNECT_EVT:  Event when a BLE device is disconnected.
      this->node_state = espbt::ClientState::IDLE;

      // this->publish_state_(this->voltage_sensor_, NAN);
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: { // ESP_GATTC_SEARCH_CMPL_EVT:  Event when the search for services is completed.

      auto *char_notify =
          this->parent_->get_characteristic(KILOVAULT_BMS_SERVICE_UUID, KILOVAULT_BMS_NOTIFY_CHARACTERISTIC_UUID);
      if (char_notify == nullptr) {
        ESP_LOGE(TAG, "[%s] No notify service found at device, not an KILOVAULT BMS..?",
                 this->parent_->address_str().c_str());
        break;
      }
      this->char_notify_handle_ = char_notify->handle;

      auto status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
                                                      char_notify->handle);
      if (status) {
        ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status);
      }

      auto *char_command =
          this->parent_->get_characteristic(KILOVAULT_BMS_SERVICE_UUID, KILOVAULT_BMS_CONTROL_CHARACTERISTIC_UUID);
      if (char_command == nullptr) {
        ESP_LOGE(TAG, "[%s] No control service found at device, not an KILOVAULT BMS..?",
                 this->parent_->address_str().c_str());
        break;
      }
      this->char_command_handle_ = char_command->handle;
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {  // ESP_GATTC_REG_FOR_NOTIFY_EVT:  Event when a notification is registered.
      this->node_state = espbt::ClientState::ESTABLISHED;

      break;
    }

    // This is the main entry point for data from the BMS
    case ESP_GATTC_NOTIFY_EVT: { // ESP_GATTC_NOTIFY_EVT:  Event when a notification is received.
      ESP_LOGVV(TAG, "Notification received (handle 0x%02X): %s", param->notify.handle,
                format_hex_pretty(param->notify.value, param->notify.value_len).c_str());

      // assemble_() is defined below
      this->assemble_(param->notify.value, param->notify.value_len);
      break;
    }

    default:
      break;
  }
}

// KC: Moved ascii_to_int to a seperate function
uint8_t KilovaultBmsBle::ascii_to_int_(const uint8_t c) {
    uint8_t v = c;
    if ((c >= 48) && (c <= 57))
        v = (c - 48);
    else if ((c >= 65) && (c <= 70))
        v = (c - 65 + 10);
    else if ((c >= 97) && (c <= 102))
        v = (c - 97 + 10);   
    return v;
}

/* ========================================================================= */
void KilovaultBmsBle::assemble_(const uint8_t *data, uint16_t length) {

  // auto ascii_to_int = [] (const uint8_t c) {
  //   uint8_t v = c;
  //   if ((c >= 48) && (c <= 57))
  //     v = (c - 48);
  //   else if ((c >= 65) && (c <= 70))
  //     v = (c - 65 + 10);
  //   else if ((c >= 97) && (c <= 102))
  //     v = (c - 97 + 10);   
  //   return v;
  // };

  // Check the frame buffer size.
  if (this->frame_buffer_.size() > MAX_RESPONSE_SIZE) {
    ESP_LOGW(TAG, "Maximum response size exceeded");
    this->frame_buffer_.clear();
  }

  // Flush buffer on every preamble. This indicated the start of a new packet.
  if (data[0] == KILOVAULT_PKT_START_A || data[0] == KILOVAULT_PKT_START_B) {
    this->frame_buffer_.clear();
  }

  // Append the data to the frame buffer.
  this->frame_buffer_.insert(this->frame_buffer_.end(), data, data + length);
  //ESP_LOGW(TAG, "frame_buffer: %s", format_hex_pretty(this->frame_buffer_).c_str());

  /* Process the frame buffer. 
    if the size of frame_buffer_ is equal to MAX_RESPONSE_SIZE,
    then convert the ascii to int and check the crc
  */
  if (this->frame_buffer_.size() == MAX_RESPONSE_SIZE) {

    for(int i=0; i < this->frame_buffer_.size(); i++){
      this->frame_buffer_[i] = this->ascii_to_int_(this->frame_buffer_[i]);
    }

    // Check the CRC. If the CRC check fails, clear the frame buffer and return.
    if (!crc(this->frame_buffer_)) {
      //ESP_LOGW(TAG, "frame_buffer: %s", format_hex_pretty(this->frame_buffer_).c_str());
      this->frame_buffer_.clear();
      return;
    }

    // Hand off to on_kilovault_bms_ble_data_() for processing.
    this->on_kilovault_bms_ble_data_(this->frame_buffer_);
    this->frame_buffer_.clear();
  }
}

/* ========================================================================= */
void KilovaultBmsBle::update() {
  if (this->node_state != espbt::ClientState::ESTABLISHED ) {
    ESP_LOGW(TAG, "[%s] Not connected", this->parent_->address_str().c_str());
    return;
  }

}

/* ========================================================================= */
/*
  This can most likely be removed and call decode_status_data_ directly from 
  the assemble_ function. Verified with copilot. There is a chance that on_kilovault_bms_ble_data
  is some sort of built in from a library, but copilot does not seem to think so.
*/
void KilovaultBmsBle::on_kilovault_bms_ble_data_(const std::vector<uint8_t> &data) {

  this->decode_status_data_(data);

}

/* ========================================================================= */
void KilovaultBmsBle::decode_status_data_(const std::vector<uint8_t> &data) {
  /*
    &data is an array that contains all of the data from the battery. This 
    function processes each element of the array and publishes it appropriately.

    TODO: Add debug code to see the data in the array. It would be great to 
    dump this out and document what all the fields are. Maybe there are ID's in it?
  */

  // Log each value of the array.
  // for (size_t i = 0; i < data.size(); i++) {
  //   ESP_LOGW(TAG, "data[%d] = %d", i, data[i]);
  // }


  /* kilovault_get_16bit lambda function
    This function captures all variables from the enclosing scope by reference,
    specifically &data. It takes in a size_t i and returns a uint16_t.

    It does bitwise shifts of each element in the &data array.
    The bitwise OR (|) effectively concats the bits from the four elements in the array
    into a single 16bit value. 
  
  */
  auto kilovault_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 2]) << 12) | (uint16_t(data[i + 3]) << 8) | (uint16_t(data[i + 0]) << 4) | (uint16_t(data[i + 1]) << 0);
  };

  /* kilovault_get_32bit lambda function
    Does the same thing as above, but combines two 16 bit values into a single 32 bit value.
  
  */
  auto kilovault_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(kilovault_get_16bit(i + 4)) << 16) | (uint32_t(kilovault_get_16bit(i + 0)) << 0);
  };

  //ESP_LOGI(TAG, "Status frame (%d+4 bytes):", data.size());
  //ESP_LOGD(TAG, "  %s", format_hex_pretty(&data.front(), data.size()).c_str());
  //this->publish_state_(this->message_text_sensor_, format_hex_pretty(&data.front(), 80).c_str());
  ESP_LOGI(TAG, "AFESTATUS (RAW): %X", data[41]);
  ESP_LOGI(TAG, "AFESTATUS (16b): %X", kilovault_get_16bit(41));
  this->publish_state_(this->afestatus_sensor_, kilovault_get_16bit(41));

  int16_t status = kilovault_get_16bit(37);
  this->publish_state_(this->status_sensor_, status);

  if (status == 0) {
    return;
  }

  /*
    If the current is greater than 2147483647, then subtract 4294967295 from the current. 
    This is done to handle the overflow of the current. This is necessary because electrical
    current can have negative values indicating reverse current flow and the system might
    be using unsigned integers to represent these values. This ensures that the current 
    value is correctly interpreted as a signed integer. 
  */
  int32_t current = kilovault_get_32bit(9);
  if (current > 2147483647) {
     current = current - 4294967295;
  } 
  
  // Publish the state of the CURRENT sensor.
  // TODO: ?? Store the value of CURRENT and reuse it for POWER calculation
  this->publish_state_(this->current_sensor_, (float) current * 0.001f);

  //  8    4  0xCE 0x61 0x00 0x00  Total voltage                    V     0.001f
  /*
    Likely that the voltage is retrieved in millivolts. This scales it to volts 
    by multiplying by 0.001. The f on the end of 0.001f indicates that the literal
    is a float and not a double.
  */
  float voltage = kilovault_get_16bit(1) * 0.001f;
  // Publish the state of the VOLTAGE sensor
  this->publish_state_(this->voltage_sensor_, voltage);

  /*
    Power is in watts. Using ohms law we multiple voltage by the current. 
    We could probably store this from the current calculations above and 
    reuse it here. 
  */
  float power = voltage * ((float) current * 0.001f);

  // Publish the state of the POWER sensor
  this->publish_state_(this->power_sensor_, power);
  
  // Publish the state of the CHARGING POWER sensor
  this->publish_state_(this->charging_power_sensor_, std::max(0.0f, power));               // 500W vs 0W -> 500W

  // Publish the state of the DISCHARGING POWER sensor
  this->publish_state_(this->discharging_power_sensor_, std::abs(std::min(0.0f, power)));  // -500W vs 0W -> 500W

  // Publish the state of the TOTAL CAPACITY sensor
  this->publish_state_(this->total_capacity_sensor_, kilovault_get_32bit(17) * 0.001f);

  // Publish the state of the CURRENT CAPACITY sensor
  this->publish_state_(this->current_capacity_sensor_, (kilovault_get_32bit(17) * 0.001f) * (kilovault_get_16bit(29) * 0.01f));
  
  // Publish the state of the CYCLES sensor
  this->publish_state_(this->cycles_sensor_, kilovault_get_16bit(25));

  // Publish the state of the STATE OF CHARGE sensor
  this->publish_state_(this->state_of_charge_sensor_, kilovault_get_16bit(29));
  
  // temp is in Kelvin covert to Celius
  int16_t temp = kilovault_get_16bit(33);

  // Publish the state of the TEMPERATURE sensor
  this->publish_state_(this->temperature_sensor_, (temp * 0.1f) - 273.15);

  // Decode the cell voltages data and handle publishing in that function
  this->decode_cell_voltages_data_(data);

  // Publish the state of the BATTERY MAC text sensor
  this->publish_state_(this->battery_mac_text_sensor_, this->parent_->address_str().c_str());

}

/* ========================================================================= */
void KilovaultBmsBle::decode_cell_voltages_data_(const std::vector<uint8_t> &data) {

/* 
  Maybe create this as a function since it is used in multiple places.
*/
  auto kilovault_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 2]) << 12) | (uint16_t(data[i + 3]) << 8) | (uint16_t(data[i + 0]) << 4) | (uint16_t(data[i + 1]) << 0);
  };

  uint8_t offset = 1;
  uint8_t cells = 4;
  uint8_t start = 41;

  //ESP_LOGI(TAG, "Cell voltages frame (chunk %d, %d+4 bytes):", data[2] - 36, data.size());
  //ESP_LOGD(TAG, "  %s", format_hex_pretty(&data.front(), data.size()).c_str());

  this->min_cell_voltage_ = 100.0f;
  this->max_cell_voltage_ = -100.0f;
  
  /*
    What I think this is doing is getting the cell voltage for each cell. Cell data
    looks like it starts at index 41 and is 4 bytes long in the &data array.

    It then makes sure that the min and max cell voltages are correct. 
    then sends it to publish. 
  */
  for (uint8_t i = 1; i <= cells; i++) {
    float cell_voltage = kilovault_get_16bit(start + (i * 4)) * 0.001f;

    if (cell_voltage > 0 && cell_voltage < this->min_cell_voltage_) {
      this->min_cell_voltage_ = cell_voltage;
      this->min_voltage_cell_ = i ;
    }

    if (cell_voltage > this->max_cell_voltage_) {
      this->max_cell_voltage_ = cell_voltage;
      this->max_voltage_cell_ = i ;
    }
    this->publish_state_(this->cells_[i-1].cell_voltage_sensor_, cell_voltage);
  }

  this->publish_state_(this->min_cell_voltage_sensor_, this->min_cell_voltage_);
  this->publish_state_(this->max_cell_voltage_sensor_, this->max_cell_voltage_);
  this->publish_state_(this->max_voltage_cell_sensor_, (float) this->max_voltage_cell_);
  this->publish_state_(this->min_voltage_cell_sensor_, (float) this->min_voltage_cell_);
  this->publish_state_(this->delta_cell_voltage_sensor_, this->max_cell_voltage_ - this->min_cell_voltage_);
}

/* ========================================================================= */
/*
  This is not called from anywhere. Clearly just a helper function.

  What it does is print all of the calculated values from the battery to the log. 
*/
void KilovaultBmsBle::dump_config() {  // NOLINT(google-readability-function-size,readability-function-size)
  ESP_LOGCONFIG(TAG, "KilovaultBmsBle:");


  LOG_SENSOR("", "Voltage", voltage_sensor_);
  LOG_SENSOR("", "Current", current_sensor_);
  LOG_SENSOR("", "Power", power_sensor_);
  LOG_SENSOR("", "Charging power", charging_power_sensor_);
  LOG_SENSOR("", "Discharging power", discharging_power_sensor_);
  LOG_SENSOR("", "State of charge", state_of_charge_sensor_);
  LOG_SENSOR("", "Cycles", cycles_sensor_);
  LOG_SENSOR("", "Total capacity", total_capacity_sensor_);
  LOG_SENSOR("", "Current capacity", current_capacity_sensor_);
  LOG_SENSOR("", "Status", status_sensor_);
  LOG_SENSOR("", "Status", afestatus_sensor_);
  LOG_SENSOR("", "Min cell voltage", min_cell_voltage_sensor_);
  LOG_SENSOR("", "Max cell voltage", max_cell_voltage_sensor_);
  LOG_SENSOR("", "Min voltage cell", min_voltage_cell_sensor_);
  LOG_SENSOR("", "Max voltage cell", max_voltage_cell_sensor_);
  LOG_SENSOR("", "Delta cell voltage", delta_cell_voltage_sensor_);
  LOG_SENSOR("", "Temperature", temperature_sensor_);
  LOG_SENSOR("", "Cell Voltage 1", this->cells_[0].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 2", this->cells_[1].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 3", this->cells_[2].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 4", this->cells_[3].cell_voltage_sensor_);

  LOG_TEXT_SENSOR("", "Battery MAC", this->battery_mac_text_sensor_);
  LOG_TEXT_SENSOR("", "Message", this->message_text_sensor_);
}

/* ========================================================================= */
/*
  This function is used to publish the state of the binary sensor. 
  The function takes in a binary_sensor::BinarySensor object and a boolean value. 
  If the binary_sensor object is not null, the function publishes the state of the binary sensor. 
*/
void KilovaultBmsBle::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr)
    return;

  binary_sensor->publish_state(state);
}

/* ========================================================================= */
/*
  This function is used to publish the state of the sensor. 
  The function takes in a sensor::Sensor object and a float value. 
  If the sensor object is not null, the function publishes the state of the sensor. 
*/
void KilovaultBmsBle::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;

  sensor->publish_state(value);
}

/* ========================================================================= */
/*
  This function is used to publish the state of the text sensor. 
  The function takes in a text_sensor::TextSensor object and a string value. 
  If the text sensor object is not null, the function publishes the state of the text sensor.
*/
void KilovaultBmsBle::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;

  text_sensor->publish_state(state);
}

/* ========================================================================= */
/*
  This function is used to publish the state of the switch. 
  The function takes in a switch_::Switch object and a boolean value. 
  If the switch object is not null, the function publishes the state of the switch.
*/
void KilovaultBmsBle::publish_state_(switch_::Switch *obj, const bool &state) {
  if (obj == nullptr)
    return;

  obj->publish_state(state);
}

/* ========================================================================= */
/*
  This clearly does nothing, and is not called from anywhere. 
*/
void KilovaultBmsBle::write_register(uint8_t address, uint16_t value) {
  // this->send_command_(KILOVAULT_CMD_WRITE, KILOVAULT_CMD_MOS);  // @TODO: Pass value
}

/* ========================================================================= */
/*
  This is not called anywhere other than write_register.  Maybe it was an experiement 
  to write data to the battery. Maybe the original author tried it and it didnt work.

  It was attempting to use the ESP BLE api to send data to the battery.
*/
bool KilovaultBmsBle::send_command_(uint8_t start_of_frame, uint8_t function, uint8_t value) {
  uint8_t frame[9];
  uint8_t data_len = 1;

  frame[0] = start_of_frame;
  frame[1] = KILOVAULT_ADDRESS;
  frame[2] = function;
  frame[3] = data_len;
  frame[4] = value;
  auto crc = chksum_(frame + 1, 4);
  frame[5] = crc >> 0;
  frame[6] = crc >> 8;
  frame[7] = KILOVAULT_PKT_END_1;
  frame[8] = KILOVAULT_PKT_END_2;

  ESP_LOGV(TAG, "Send command (handle 0x%02X): %s", this->char_command_handle_,
           format_hex_pretty(frame, sizeof(frame)).c_str());

  auto status =
      esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->char_command_handle_,
                               sizeof(frame), frame, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);

  if (status) {
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char failed, status=%d", this->parent_->address_str().c_str(), status);
  }

  return (status == 0);
}



}  // namespace kilovault_bms_ble
}  // namespace esphome
