#include "ce2727a.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#define STATE_BOOTUP_WAIT "Waiting to boot up"
#define STATE_METER_NOT_FOUND "Meter not found"
#define STATE_METER_FOUND "Meter found"
#define STATE_OK "OK"
#define STATE_PARTIAL_OK "Read partial data"
#define STATE_DATA_FAIL "Unable to read data"

namespace esphome {
namespace ce2727a {

static const char *TAG = "CE2727A";

static constexpr uint8_t FRAME_START_MAGIC = 0x02;

static constexpr uint8_t MASK_GOT_DATE_TIME = 0b001;
static constexpr uint8_t MASK_GOT_ACTIVE_POWER = 0b010;
static constexpr uint8_t MASK_GOT_ENERGY = 0b100;

static constexpr uint16_t MESSAGE_CRC_IEC = 0x0F47;

static constexpr uint32_t NO_GOOD_READS_TIMEOUT_MS = 5 * 60 * 1000;  // 5 minutes

static constexpr int32_t MAX_TIME_CORRECTION = 125;
static constexpr int32_t SECONDS_IN_24H = 24 * 3600;

static inline int bcd2dec(uint8_t hex) {
  assert(((hex & 0xF0) >> 4) < 10);  // More significant nybble is valid
  assert((hex & 0x0F) < 10);         // Less significant nybble is valid
  int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
  return dec;
}

#pragma pack(1)
typedef struct {
  uint8_t two;
  uint8_t length;
  uint32_t address;
  uint32_t password;
  uint8_t com;
  uint8_t cmd_id;
} ce2727a_frame_header_t;

typedef struct {
  ce2727a_frame_header_t header;
  uint16_t crc16;
} ce2727a_request_command_t;

typedef struct {
  ce2727a_frame_header_t header;
  uint16_t fw_ver;
  uint16_t error_code_1;
  uint16_t error_code_2;
  uint16_t error_code_3;
  uint8_t diag_info[4];
  uint32_t serial_number;
  uint32_t network_address;
  uint8_t physical_address[16];
  uint8_t hw_version_bcd;
  uint8_t options_version_bcd;
  uint16_t status;
  uint16_t crc16;
} ce2727a_response_info_t;

typedef struct {
  ce2727a_frame_header_t header;
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint8_t year;
  uint8_t weekday_and_summer;
  uint8_t dst_allowed;
  int8_t correction_seconds_left;
  uint16_t crc16;
} ce2727a_response_date_time_t;

typedef struct {
  ce2727a_frame_header_t header;
  uint32_t active_power;
  uint16_t crc16;
} ce2727a_response_active_power_t;

typedef struct {
  ce2727a_frame_header_t header;
  uint8_t current_tariff;
  uint32_t total;
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;
  uint32_t t4;
  uint16_t crc16;
} ce2727a_response_consumed_energy_t;

typedef struct {
  ce2727a_frame_header_t header;
  int8_t correction_seconds;
  uint16_t crc16;
} ce2727a_request_correct_time_t;

typedef struct {
  ce2727a_frame_header_t header;
  uint16_t crc16;
} ce2727a_response_correct_time_t;
#pragma pack(0)

static_assert(sizeof(ce2727a_request_command_t) == 0x0e, "Wrong structure size: ce2727a_request_command_t");
static_assert(sizeof(ce2727a_response_info_t) == 0x36, "Wrong structure size: ce2727a_response_info_t");
static_assert(sizeof(ce2727a_response_date_time_t) == 0x17, "Wrong structure size: ce2727a_response_date_time_t");
static_assert(sizeof(ce2727a_response_active_power_t) == 0x12, "Wrong structure size: ce2727a_response_active_power_t");
static_assert(sizeof(ce2727a_response_consumed_energy_t) == 0x23,
              "Wrong structure size: ce2727a_response_consumed_energy_t");

float CE2727aComponent::get_setup_priority() const { return setup_priority::AFTER_WIFI; }

void CE2727aComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "CE2727a:");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Meter address requested: %u", this->requested_meter_address_);
  ESP_LOGCONFIG(TAG, "  Receive timeout: %.1fs", this->receive_timeout_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Update interval: %.1fs", this->update_interval_ / 1e3f);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  LOG_SENSOR("  ", "Active power", this->active_power_sensor_);
  LOG_SENSOR("  ", "Energy total", this->energy_total_sensor_);
  LOG_SENSOR("  ", "Energy consumed total", this->energy_total_sensor_);
  LOG_SENSOR("  ", "Energy consumed tariff 1", this->energy_t1_sensor_);
  LOG_SENSOR("  ", "Energy consumed tariff 2", this->energy_t2_sensor_);
  LOG_SENSOR("  ", "Energy consumed tariff 3", this->energy_t3_sensor_);
  LOG_SENSOR("  ", "Energy consumed tariff 4", this->energy_t4_sensor_);
  LOG_TEXT_SENSOR("  ", "Electricity tariff", this->tariff_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Date", this->date_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Time", this->time_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Network address", this->network_address_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Serial number", this->serial_nr_text_sensor_);
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
  ESP_LOGCONFIG(TAG, "Data errors %d, proper reads %d", this->data_.read_errors, this->data_.proper_reads);
}

void CE2727aComponent::setup() {
  if (this->reading_state_text_sensor_ != nullptr) {
    this->reading_state_text_sensor_->publish_state(STATE_BOOTUP_WAIT);
  }

  this->set_timeout(1000, [this]() { this->state_ = State::IDLE; });
}

void CE2727aComponent::loop() {
  if (!this->is_ready())
    return;

  switch (this->state_) {
    case State::NOT_INITIALIZED: {
      this->log_state_();
    } break;

    case State::IDLE: {
      this->log_state_();
      uint32_t now = millis();
      if (now - this->data_.last_good_read_ms > NO_GOOD_READS_TIMEOUT_MS) {
        ESP_LOGE(TAG, "Rebooting due to no good reads from the meter for 5 minutes...");
        delay(1000);
        App.reboot();
      }
    } break;

    case State::WAITING_FOR_RESPONSE: {
      this->log_state_(&this->next_state_);
      read_response();
    } break;

    case State::GET_METER_INFO: {
      this->log_state_();
      if (this->data_.meter_found) {
        this->state_ = State::GET_DATE_TIME;  // Skip to next state if meter is already found
      } else {
        prepare_enq_request(EnqCmd::Info);
        send_prepared_request();
        ReadFunction read_fn = [this]() { return this->read_meter_info(); };
        read_reply_and_go_next_state_(sizeof(ce2727a_response_info_t), read_fn, State::GET_DATE_TIME);
      }
    } break;

    case State::GET_DATE_TIME: {
      this->log_state_();
      this->data_.date_str[0] = 0;
      this->data_.time_str[0] = 0;
      prepare_enq_request(EnqCmd::DateTime);
      send_prepared_request();
      ReadFunction read_fn = [this]() { return this->read_date_time(); };
      read_reply_and_go_next_state_(
          sizeof(ce2727a_response_date_time_t), read_fn,
          this->got_time_correction_request_ ? State::CORRECT_DATE_TIME : State::GET_ACTIVE_POWER);
    } break;

    case State::CORRECT_DATE_TIME: {
      this->log_state_();
      this->next_state_ = State::GET_ACTIVE_POWER;

      if (this->time_source_ == nullptr || !this->got_time_correction_request_ ||
          !(this->data_.got & MASK_GOT_DATE_TIME))
        return;

      auto time_now = this->time_source_->now();
      if (!time_now.is_valid()) {
        ESP_LOGE(TAG, "Time sync requested, but time provider is not yet ready");
        return;
      }

      if (!this->data_.datetime.is_valid()) {
        ESP_LOGE(TAG, "Time sync requested, but no valid date/time data available");
        return;
      }

      this->got_time_correction_request_ = false;

      time_now.recalc_timestamp_local();
      int32_t correction_seconds = time_now.timestamp - this->data_.datetime.timestamp;
      if (correction_seconds > -2 && correction_seconds < 2) {
        ESP_LOGI(TAG, "No time correction needed (less than 2 seconds)");
        return;
      }

      ESP_LOGI(TAG, "Time correction needed: %d seconds", correction_seconds);

      // if correction is more than 24 hours,
      // it is serious meter failure, meter shall be replaced or time is completely wrong
      if (correction_seconds > SECONDS_IN_24H || correction_seconds < -SECONDS_IN_24H) {
        ESP_LOGE(TAG, "Time correction is more than 24 hours, meter is broken or time is completely wrong.");
        return;
      }

      if (correction_seconds > MAX_TIME_CORRECTION) {
        correction_seconds = MAX_TIME_CORRECTION;
      } else if (correction_seconds < -MAX_TIME_CORRECTION) {
        correction_seconds = -MAX_TIME_CORRECTION;
      }
      ESP_LOGD(TAG, "Setting time correction within +/- %d seconds: %d", MAX_TIME_CORRECTION, correction_seconds);

      prepare_rec_time_correction_request(static_cast<int8_t>(correction_seconds));
      send_prepared_request();
      ReadFunction read_fn = [this]() { return this->read_time_correction(); };
      read_reply_and_go_next_state_(sizeof(ce2727a_response_correct_time_t), read_fn, State::GET_ACTIVE_POWER);
    } break;

    case State::GET_ACTIVE_POWER: {
      this->log_state_();
      prepare_enq_request(EnqCmd::ActivePower);
      send_prepared_request();
      ReadFunction read_fn = [this]() { return this->read_active_power(); };
      read_reply_and_go_next_state_(sizeof(ce2727a_response_active_power_t), read_fn, State::GET_ENERGY);
    } break;

    case State::GET_ENERGY: {
      this->log_state_();
      prepare_enq_request(EnqCmd::ConsumedEnergy);
      send_prepared_request();
      ReadFunction read_fn = [this]() { return this->read_consumed_energy(); };
      read_reply_and_go_next_state_(sizeof(ce2727a_response_consumed_energy_t), read_fn, State::PUBLISH_INFO);
    } break;

    case State::PUBLISH_INFO: {
      this->log_state_();
      // Update status text sensor based on the data we were able to collect
      if (this->data_.got == (MASK_GOT_DATE_TIME | MASK_GOT_ACTIVE_POWER | MASK_GOT_ENERGY)) {
        this->data_.failure = false;
        this->data_.initialized = true;
        if (this->reading_state_text_sensor_ != nullptr) {
          this->reading_state_text_sensor_->publish_state(STATE_OK);
        }
        this->data_.last_good_read_ms = millis();
      } else {
        ESP_LOGW(TAG, "Got no or partial data %o", this->data_.got);
        this->data_.failure = true;
        if (this->reading_state_text_sensor_ != nullptr) {
          this->reading_state_text_sensor_->publish_state((this->data_.got == 0) ? STATE_DATA_FAIL : STATE_PARTIAL_OK);
        }
      }

      // Publish meter info if available
      if (this->data_.meter_found) {
        if (this->network_address_text_sensor_ != nullptr) {
          this->network_address_text_sensor_->publish_state(to_string(this->data_.network_address));
        }
        if (this->serial_nr_text_sensor_ != nullptr) {
          this->serial_nr_text_sensor_->publish_state(to_string(this->data_.serial_number));
        }
        if (this->reading_state_text_sensor_ != nullptr && !this->data_.initialized) {
          this->reading_state_text_sensor_->publish_state(STATE_METER_FOUND);
        }
      } else if (this->reading_state_text_sensor_ != nullptr && !this->data_.initialized) {
        this->reading_state_text_sensor_->publish_state(STATE_METER_NOT_FOUND);
      }

      // Publish date/time if available
      if (this->data_.got & MASK_GOT_DATE_TIME) {
        if (this->date_text_sensor_ != nullptr) {
          this->date_text_sensor_->publish_state(this->data_.date_str);
        }
        if (this->time_text_sensor_ != nullptr) {
          this->time_text_sensor_->publish_state(this->data_.time_str);
        }
      }

      // Publish active power if available
      if (this->data_.got & MASK_GOT_ACTIVE_POWER) {
        if (this->active_power_sensor_ != nullptr) {
          this->active_power_sensor_->publish_state(this->data_.active_power);
        }
      }

      // Publish energy data if available
      if (this->data_.got & MASK_GOT_ENERGY) {
        if (this->tariff_text_sensor_ != nullptr) {
          char tariff_str[3];
          tariff_str[0] = 'T';
          tariff_str[1] = '0' + (this->data_.energy.current_tariff & 0b11);
          tariff_str[2] = 0;
          this->tariff_text_sensor_->publish_state(tariff_str);
        }

        if (this->energy_total_sensor_ != nullptr) {
          this->energy_total_sensor_->publish_state(this->data_.energy.total);
        }
        if (this->energy_t1_sensor_ != nullptr) {
          this->energy_t1_sensor_->publish_state(this->data_.energy.t1);
        }
        if (this->energy_t2_sensor_ != nullptr) {
          this->energy_t2_sensor_->publish_state(this->data_.energy.t2);
        }
        if (this->energy_t3_sensor_ != nullptr) {
          this->energy_t3_sensor_->publish_state(this->data_.energy.t3);
        }
        if (this->energy_t4_sensor_ != nullptr) {
          this->energy_t4_sensor_->publish_state(this->data_.energy.t4);
        }
      }

      ESP_LOGD(TAG, "Data errors %d, proper reads %d", this->data_.read_errors, this->data_.proper_reads);
      this->state_ = State::IDLE;
    } break;

    default:
      break;
  }
}

void CE2727aComponent::update() {
  if (this->is_ready() && this->state_ == State::IDLE) {
    ESP_LOGV(TAG, "Update: Initiating new data collection");
    this->data_.got = 0;
    this->state_ = State::GET_METER_INFO;
  } else {
    ESP_LOGV(TAG, "Update: Component not ready yet");
  }
}

void CE2727aComponent::sync_device_time() {
#ifdef USE_TIME
  if (this->time_source_ != nullptr) {
    this->got_time_correction_request_ = true;
    ESP_LOGI(TAG, "Got time correction request, waiting for next update");
  }
#endif
}

void CE2727aComponent::prepare_enq_request(uint8_t cmd) {
  ce2727a_request_command_t *out = (ce2727a_request_command_t *) this->buffers_.out;
  out->header.two = FRAME_START_MAGIC;
  out->header.length = sizeof(ce2727a_request_command_t);
  out->header.address = this->requested_meter_address_;
  out->header.password = 0x0;
  out->header.com = static_cast<uint8_t>(ComType::Enq);
  out->header.cmd_id = static_cast<uint8_t>(cmd);

  out->crc16 = crc_16_iec(this->buffers_.out, sizeof(ce2727a_request_command_t) - 2);
  this->buffers_.amount_out = sizeof(ce2727a_request_command_t);
}

void CE2727aComponent::prepare_rec_time_correction_request(int8_t correction_seconds) {
  ce2727a_request_correct_time_t *out = (ce2727a_request_correct_time_t *) this->buffers_.out;
  out->header.two = FRAME_START_MAGIC;
  out->header.length = sizeof(ce2727a_request_correct_time_t);
  out->header.address = this->requested_meter_address_;
  out->header.password = 0x0;
  out->header.com = static_cast<uint8_t>(ComType::Rec);
  out->correction_seconds = correction_seconds;
  out->header.cmd_id = static_cast<uint8_t>(RecCmd::TimeCorrection);

  out->crc16 = crc_16_iec(this->buffers_.out, sizeof(ce2727a_request_correct_time_t) - 2);  // minus 2 bytes for CRC
  this->buffers_.amount_out = sizeof(ce2727a_request_correct_time_t);
}

void CE2727aComponent::read_reply_and_go_next_state_(size_t expected_size, ReadFunction read_fn, State next_state) {
  this->next_state_ = next_state;
  this->buffers_.amount_in = 0;
  this->request_tracker_.read_fn = read_fn;
  this->request_tracker_.expected_size = static_cast<uint16_t>(expected_size);
  this->state_ = State::WAITING_FOR_RESPONSE;
}

void CE2727aComponent::send_prepared_request() {
  this->request_tracker_.start_time_ms = millis();

  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
  }

  write_array(this->buffers_.out, this->buffers_.amount_out);
  flush();

  ESP_LOGVV(TAG, "TX: %s", format_hex_pretty(this->buffers_.out, this->buffers_.amount_out).c_str());

  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }
}

bool CE2727aComponent::read_response() {
  auto now = millis();

  // Check timeout
  if (now - this->request_tracker_.start_time_ms > this->receive_timeout_) {
    ESP_LOGE(TAG, "Response timeout");
    this->data_.read_errors++;
    this->state_ = this->next_state_;
    return false;
  }

  // Read available data
  while (available() > 0 && this->buffers_.amount_in < this->request_tracker_.expected_size) {
    int current_byte = read();
    if (current_byte >= 0) {
      this->buffers_.in[this->buffers_.amount_in++] = static_cast<uint8_t>(current_byte);
    }
    yield();
  }

  // Check if we have received all expected bytes
  if (this->buffers_.amount_in > 0 && this->buffers_.amount_in == this->request_tracker_.expected_size) {
    ESP_LOGV(TAG, "Received all bytes, validating...");
    ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(this->buffers_.in, this->buffers_.amount_in).c_str());

    this->state_ = this->next_state_;

    // CRC of message + its CRC = 0x0F47
    if (crc_16_iec(this->buffers_.in, this->buffers_.amount_in) != MESSAGE_CRC_IEC) {
      ESP_LOGE(TAG, "CRC check failed");
      this->data_.read_errors++;
      return false;
    }

    // Process the received data
    this->data_.proper_reads++;
    this->request_tracker_.read_fn();

    return true;
  }

  // Not enough data yet, stay in WAITING_FOR_RESPONSE state
  return false;
}

size_t CE2727aComponent::read_meter_info() {
  ce2727a_response_info_t &res = *(ce2727a_response_info_t *) this->buffers_.in;
  this->data_.serial_number = res.serial_number;
  this->data_.network_address = res.network_address;

  ESP_LOGI(TAG,
           "Got reply from meter with s/n %u (0x%08X), network address %u, "
           "fw ver. %04X",
           res.serial_number, res.serial_number, res.network_address, res.fw_ver);

  if (!this->data_.meter_found) {
    this->data_.meter_found = true;
    requested_meter_address_ = this->data_.network_address;
  }
  return this->buffers_.amount_in;
}

size_t CE2727aComponent::read_date_time() {
  ce2727a_response_date_time_t &res = *(ce2727a_response_date_time_t *) this->buffers_.in;

  snprintf(this->data_.time_str, sizeof(this->data_.time_str), "%02d:%02d:%02d", bcd2dec(res.hour), bcd2dec(res.minute),
           bcd2dec(res.second));
  snprintf(this->data_.date_str, sizeof(this->data_.date_str), "%02d/%02d/20%02d", bcd2dec(res.day), bcd2dec(res.month),
           bcd2dec(res.year));

  this->data_.datetime.day_of_week = 1;
  this->data_.datetime.day_of_year = 1;
  this->data_.datetime.year = 2000 + bcd2dec(res.year);
  this->data_.datetime.month = bcd2dec(res.month);
  this->data_.datetime.day_of_month = bcd2dec(res.day);
  this->data_.datetime.hour = bcd2dec(res.hour);
  this->data_.datetime.minute = bcd2dec(res.minute);
  this->data_.datetime.second = bcd2dec(res.second);
  this->data_.datetime.recalc_timestamp_local();

  ESP_LOGI(TAG, "Got date/time: Date %s Time %s; Available corrections: %d", this->data_.date_str, this->data_.time_str,
           res.correction_seconds_left);

  this->data_.got |= MASK_GOT_DATE_TIME;
  return this->buffers_.amount_in;
}

size_t CE2727aComponent::read_active_power() {
  ce2727a_response_active_power_t &res = *(ce2727a_response_active_power_t *) this->buffers_.in;
  this->data_.active_power = res.active_power;

  ESP_LOGI(TAG, "Got active power: %d", res.active_power);
  this->data_.got |= MASK_GOT_ACTIVE_POWER;
  return this->buffers_.amount_in;
}

size_t CE2727aComponent::read_consumed_energy() {
  ce2727a_response_consumed_energy_t &res = *(ce2727a_response_consumed_energy_t *) this->buffers_.in;
  this->data_.energy.current_tariff = res.current_tariff;
  this->data_.energy.total = res.total;
  this->data_.energy.t1 = res.t1;
  this->data_.energy.t2 = res.t2;
  this->data_.energy.t3 = res.t3;
  this->data_.energy.t4 = res.t4;

  ESP_LOGI(TAG, "Got energy: T%d, T1=%d, T2=%d, T3=%d, T4=%d", res.current_tariff, res.t1, res.t2, res.t3, res.t4);
  this->data_.got |= MASK_GOT_ENERGY;
  return this->buffers_.amount_in;
}

size_t CE2727aComponent::read_time_correction() {
  ce2727a_response_correct_time_t &res = *(ce2727a_response_correct_time_t *) this->buffers_.in;

  if (res.header.com == 0x0B && res.header.cmd_id == 0x00) {
    ESP_LOGD(TAG, "Time correction successful");
  } else if (res.header.com == 0x0A) {
    if (res.header.cmd_id == 0x09) {
      ESP_LOGE(TAG, "Time correction failed, already performed this day");
    } else {
      ESP_LOGE(TAG, "Time correction failed, error code: %04X", res.header.cmd_id);
    }
    this->data_.read_errors++;
  } else {
    ESP_LOGE(TAG, "Unexpected response for time correction request: com=%u, cmd_id=%u", res.header.com,
             res.header.cmd_id);
    this->data_.read_errors++;
  }
  // check error code

  return this->buffers_.amount_in;
}

const char *CE2727aComponent::state_to_string(State state) {
  switch (state) {
    case State::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case State::IDLE:
      return "IDLE";
    case State::WAITING_FOR_RESPONSE:
      return "WAITING_FOR_RESPONSE";
    case State::GET_METER_INFO:
      return "GET_METER_INFO";
    case State::GET_DATE_TIME:
      return "GET_DATE_TIME";
    case State::CORRECT_DATE_TIME:
      return "CORRECT_DATE_TIME";
    case State::GET_ACTIVE_POWER:
      return "GET_ACTIVE_POWER";
    case State::GET_ENERGY:
      return "GET_ENERGY";
    case State::PUBLISH_INFO:
      return "PUBLISH_INFO";
    default:
      return "UNKNOWN_STATE";
  }
}

void CE2727aComponent::log_state_(State *next_state) {
  if (this->state_ != this->last_reported_state_) {
    if (next_state == nullptr) {
      ESP_LOGV(TAG, "State::%s", this->state_to_string(this->state_));
    } else {
      ESP_LOGV(TAG, "State::%s -> %s", this->state_to_string(this->state_), this->state_to_string(*next_state));
    }
    this->last_reported_state_ = this->state_;
  }
}
//---------------------------------------------------------------------------------------
//  The standard 16-bit CRC polynomial specified in ISO/IEC 3309 is used.
//             16   12   5
//  Which is: x  + x  + x + 1
//----------------------------------------------------------------------------
uint16_t CE2727aComponent::crc_16_iec(const uint8_t *buffer, uint16_t len) {
  uint16_t crc = 0xffff;
  uint8_t d;
  do {
    d = *buffer++ ^ (crc & 0xFF);
    d ^= d << 4;
    crc = (d << 3) ^ (d << 8) ^ (crc >> 8) ^ (d >> 4);
  } while (--len);
  crc ^= 0xFFFF;
  return crc;
}
}  // namespace ce2727a
}  // namespace esphome