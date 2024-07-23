#include "esphome/core/log.h"
#include "ce2727a.h"

#define STATE_BOOTUP_WAIT "Waiting to boot up"
#define STATE_METER_NOT_FOUND "Meter not found"
#define STATE_METER_FOUND "Meter found"
#define STATE_OK "OK"
#define STATE_PARTIAL_OK "Read partial data"
#define STATE_DATA_FAIL "Unable to read data"

#define MASK_GOT_DATE_TIME 0b001
#define MASK_GOT_ACTIVE_POWER 0b010
#define MASK_GOT_ENERGY 0b100

#define MESSAGE_CRC_IEC 0x0F47

namespace esphome {
namespace ce2727a {

static const char *TAG = "CE2727A";

static constexpr size_t rxBufferSize = 64u;
static std::array<uint8_t, rxBufferSize> rxBuffer;

constexpr uint8_t bootupWaitUpdate = 10;  // avoid communications until properly booted

static inline int bcd2dec(uint8_t hex) {
  assert(((hex & 0xF0) >> 4) < 10);  // More significant nybble is valid
  assert((hex & 0x0F) < 10);         // Less significant nybble is valid
  int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
  return dec;
}

enum class ComType : uint8_t { Enq = 0x01, Rec = 0x03, Drj = 0x0a, OK = 0x0b };

#pragma pack(1)
typedef struct {
  uint8_t two;
  uint8_t length;
  uint32_t address;
  uint32_t password;
  uint8_t com;
  uint8_t id;
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
  uint8_t weekdayAndSummer;
  uint8_t dstAllowed;
  uint8_t correctionSecondsLeft;
  uint16_t crc16;
} ce2727a_response_date_time_t;

typedef struct {
  ce2727a_frame_header_t header;
  uint32_t activePower;
  uint16_t crc16;
} ce2727a_response_active_power_t;

typedef struct {
  ce2727a_frame_header_t header;
  uint8_t currentTariff;
  uint32_t total;
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;
  uint32_t t4;
  uint16_t crc16;
} ce2727a_response_consumed_energy_t;
#pragma pack(0)

static ce2727a_request_command_t txBuffer;
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
  LOG_SENSOR("  ", "Active power", this->active_power_);
  LOG_SENSOR("  ", "Energy total", this->energy_total_);
  LOG_SENSOR("  ", "Energy consumed total", this->energy_total_);
  LOG_SENSOR("  ", "Energy consumed tariff 1", this->energy_t1_);
  LOG_SENSOR("  ", "Energy consumed tariff 2", this->energy_t2_);
  LOG_SENSOR("  ", "Energy consumed tariff 3", this->energy_t3_);
  LOG_SENSOR("  ", "Energy consumed tariff 4", this->energy_t4_);
  LOG_TEXT_SENSOR("  ", "Electricity tariff", this->tariff_);
  LOG_TEXT_SENSOR("  ", "Date", this->date_);
  LOG_TEXT_SENSOR("  ", "Time", this->time_);
  LOG_TEXT_SENSOR("  ", "Network address", this->network_address_);
  LOG_TEXT_SENSOR("  ", "Serial number", this->serial_nr_);
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
  ESP_LOGCONFIG(TAG, "Data errors %d, proper reads %d", this->data_.readErrors, this->data_.properReads);
}

void CE2727aComponent::setup() {
  if (this->reading_state_ != nullptr) {
    this->reading_state_->publish_state(STATE_BOOTUP_WAIT);
  }

  this->set_timeout(1000, [this]() { this->fsm_state_ = State::IDLE; });
}

void CE2727aComponent::loop() {
  if (!this->is_ready())
    return;

  switch (this->fsm_state_) {
    case State::NOT_INITIALIZED:
    case State::IDLE:
      break;

    case State::GET_METER_INFO: {
      if (!this->data_.meterFound) {
        flush();
        if (get_meter_info()) {
          ESP_LOGI(TAG, "Found meter with s/n %u, we will be working with it from now on.", this->data_.networkAddress);
          this->data_.meterFound = true;
          requested_meter_address_ = this->data_.networkAddress;

          if (this->network_address_ != nullptr) {
            this->network_address_->publish_state(to_string(this->data_.networkAddress));
          }
          if (this->serial_nr_ != nullptr) {
            this->serial_nr_->publish_state(to_string(this->data_.serialNumber));
          }
          if (this->reading_state_ != nullptr) {
            this->reading_state_->publish_state(STATE_METER_FOUND);
          }
        } else {
          if (this->reading_state_ != nullptr) {
            this->reading_state_->publish_state(STATE_METER_NOT_FOUND);
          }
        }
      }
      this->fsm_state_ = this->data_.meterFound ? State::GET_DATE_TIME : State::PUBLISH_INFO;
    } break;
    case State::GET_DATE_TIME: {
      flush();
      if (get_date_time()) {
        if (this->date_ != nullptr) {
          this->date_->publish_state(this->data_.dateStr);
        }
        if (this->time_ != nullptr) {
          this->time_->publish_state(this->data_.timeStr);
        }
        this->data_.got |= MASK_GOT_DATE_TIME;
      }
      this->fsm_state_ = State::GET_ACTIVE_POWER;
    } break;

    case State::GET_ACTIVE_POWER: {
      flush();
      if (get_active_power()) {
        if (this->active_power_ != nullptr) {
          this->active_power_->publish_state(this->data_.activePower);
        }
        this->data_.got |= MASK_GOT_ACTIVE_POWER;
      }
      this->fsm_state_ = State::GET_ENERGY;
    } break;

    case State::GET_ENERGY: {
      flush();
      if (get_energy_by_tariff()) {
        if (this->tariff_ != nullptr) {
          char tariff_str[3];
          tariff_str[0] = 'T';
          tariff_str[1] = '0' + (this->data_.energy.currentTariff & 0b11);
          tariff_str[2] = 0;
          this->tariff_->publish_state(tariff_str);
        }

        if (this->energy_total_ != nullptr) {
          this->energy_total_->publish_state(this->data_.energy.total);
        }
        if (this->energy_t1_ != nullptr) {
          this->energy_t1_->publish_state(this->data_.energy.t1);
        }
        if (this->energy_t2_ != nullptr) {
          this->energy_t2_->publish_state(this->data_.energy.t2);
        }
        if (this->energy_t3_ != nullptr) {
          this->energy_t3_->publish_state(this->data_.energy.t3);
        }
        if (this->energy_t4_ != nullptr) {
          this->energy_t4_->publish_state(this->data_.energy.t4);
        }
        this->data_.got |= MASK_GOT_ENERGY;
      }
      this->fsm_state_ = State::PUBLISH_INFO;
    } break;

    case State::PUBLISH_INFO: {
      if (this->data_.got == 0b111) {
        this->data_.failure = false;
        this->data_.initialized = true;
        if (this->reading_state_ != nullptr) {
          this->reading_state_->publish_state(STATE_OK);
        }
      } else {
        ESP_LOGI(TAG, "Got no or partial data %o", this->data_.got);
        this->data_.failure = true;
        if (this->reading_state_ != nullptr) {
          this->reading_state_->publish_state((this->data_.got == 0) ? STATE_DATA_FAIL : STATE_PARTIAL_OK);
        }
      }
      ESP_LOGD(TAG, "Data errors %d, proper reads %d", this->data_.readErrors, this->data_.properReads);
      this->fsm_state_ = State::IDLE;
    } break;
    default:
      break;
  }
}

void CE2727aComponent::update() {
  if (this->is_ready() && this->fsm_state_ == State::IDLE) {
    ESP_LOGV(TAG, "Update: Initiating new data collection");
    this->data_.got = 0;
    this->fsm_state_ = State::GET_METER_INFO;
  } else {
    ESP_LOGV(TAG, "Update: Component not ready yet");
  }
}

void CE2727aComponent::send_enquiry_command(EnqCmd cmd) {
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  txBuffer.header.two = 0x02;  // magic
  txBuffer.header.length = sizeof(ce2727a_request_command_t);
  txBuffer.header.address = this->requested_meter_address_;
  txBuffer.header.password = 0x0;
  txBuffer.header.com = static_cast<uint8_t>(ComType::Enq);
  txBuffer.header.id = static_cast<uint8_t>(cmd);
  txBuffer.crc16 =
      crc_16_iec((const uint8_t *) &txBuffer, sizeof(ce2727a_request_command_t) - 2);  // minus 2 bytes for CRC

  write_array((const uint8_t *) &txBuffer, sizeof(ce2727a_request_command_t));

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

bool CE2727aComponent::receive_proper_response(uint16_t expectedSize) {
  auto stopWaiting = millis() + this->receive_timeout_;
  uint16_t bytesRead = 0;
  int currentByte = 0;
  ESP_LOGV(TAG, "Expecting %d bytes", expectedSize);
  while ((bytesRead < expectedSize) && (millis() < stopWaiting)) {
    while (available() > 0 && bytesRead < expectedSize) {
      currentByte = read();
      if (currentByte >= 0) {
        rxBuffer[bytesRead++] = (uint8_t) currentByte;
      }
      yield();
    }
    delay(5);
    yield();
    ESP_LOGV(TAG, "Got some bytesRead %d", bytesRead);
  }
  ESP_LOGV(TAG, "Bytes expected/read %d/%d", expectedSize, bytesRead);
  ESP_LOGVV(TAG, "Got reponse: %s", format_hex_pretty((const uint8_t *) rxBuffer.data(), bytesRead).c_str());

  if (bytesRead != expectedSize) {
    ESP_LOGE(TAG, "receiveProperResponse wrong size");
    this->data_.readErrors++;
    return false;
  };

  // CRC of message + its CRC = 0x0F47
  if (crc_16_iec(rxBuffer.data(), expectedSize) != MESSAGE_CRC_IEC) {
    ESP_LOGE(TAG, "receiveProperResponse CRC failed");
    this->data_.readErrors++;
    return false;
  }
  this->data_.properReads++;
  return true;
}

bool CE2727aComponent::get_meter_info() {
  ESP_LOGV(TAG, "get_meter_info()");
  send_enquiry_command(EnqCmd::Info);

  if (!receive_proper_response(sizeof(ce2727a_response_info_t)))
    return false;

  ce2727a_response_info_t &res = *(ce2727a_response_info_t *) rxBuffer.data();
  this->data_.serialNumber = res.serial_number;
  this->data_.networkAddress = res.network_address;

  ESP_LOGI(TAG, "get_meter_info() Got reply from meter with s/n %u (0x%08X), network address %u, fw ver. %04X",
           res.serial_number, res.serial_number, res.network_address, res.fw_ver);

  return true;
}

bool CE2727aComponent::get_date_time() {
  ESP_LOGV(TAG, "get_date_time()");

  this->data_.dateStr[0] = 0;
  this->data_.timeStr[0] = 0;

  send_enquiry_command(EnqCmd::DateTime);
  if (!receive_proper_response(sizeof(ce2727a_response_date_time_t)))
    return false;

  ce2727a_response_date_time_t &res = *(ce2727a_response_date_time_t *) rxBuffer.data();

  snprintf(this->data_.timeStr, sizeof(this->data_.timeStr), "%02d:%02d:%02d", bcd2dec(res.hour), bcd2dec(res.minute),
           bcd2dec(res.second));
  snprintf(this->data_.dateStr, sizeof(this->data_.dateStr), "%02d/%02d/20%02d", bcd2dec(res.day), bcd2dec(res.month),
           bcd2dec(res.year));

  ESP_LOGI(TAG, "get_date_time() Date %s Time %s", this->data_.dateStr, this->data_.timeStr);

  return true;
}

bool CE2727aComponent::get_active_power() {
  ESP_LOGV(TAG, "get_active_power()");
  send_enquiry_command(EnqCmd::ActivePower);

  if (!receive_proper_response(sizeof(ce2727a_response_active_power_t)))
    return false;

  ce2727a_response_active_power_t &res = *(ce2727a_response_active_power_t *) rxBuffer.data();
  this->data_.activePower = res.activePower;
  ESP_LOGI(TAG, "get_active_power() %d", res.activePower);

  return true;
}

bool CE2727aComponent::get_energy_by_tariff() {
  ESP_LOGV(TAG, "get_energy_by_tariff()");
  send_enquiry_command(EnqCmd::ConsumedEnergy);

  if (!receive_proper_response(sizeof(ce2727a_response_consumed_energy_t)))
    return false;

  ce2727a_response_consumed_energy_t &res = *(ce2727a_response_consumed_energy_t *) rxBuffer.data();
  this->data_.energy.currentTariff = res.currentTariff;
  this->data_.energy.total = res.total;
  this->data_.energy.t1 = res.t1;
  this->data_.energy.t2 = res.t2;
  this->data_.energy.t3 = res.t3;
  this->data_.energy.t4 = res.t4;

  ESP_LOGI(TAG, "get_energy_by_tariff() T%d, T1=%d, T2=%d, T3=%d, T4=%d", res.currentTariff, res.t1, res.t2, res.t3,
           res.t4);
  return true;
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