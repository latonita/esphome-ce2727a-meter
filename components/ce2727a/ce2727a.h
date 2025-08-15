#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

namespace esphome {
namespace ce2727a {

using ReadFunction = std::function<size_t()>;

struct InternalDataState {
  struct Readings {
    uint8_t current_tariff;
    uint32_t total{0};
    uint32_t t1{0};
    uint32_t t2{0};
    uint32_t t3{0};
    uint32_t t4{0};
  } energy;

  uint32_t active_power{0};

  char time_str[9]{0};   // "23:59:99"
  char date_str[11]{0};  // "30/08/2023"
  ESPTime datetime;
  uint32_t got_datetime_at_ms{0};
  
  uint32_t serial_number{0};
  uint32_t network_address{0};
  uint32_t proper_reads{0};
  uint32_t read_errors{0};

  bool meter_found{false};
  bool initialized{false};
  bool failure{false};
  uint8_t got{0};
  uint32_t last_good_read_ms{0};
};

enum ComType : uint8_t { Enq = 0x01, Rec = 0x03, Drj = 0x0a, OK = 0x0b };
enum EnqCmd : uint8_t { Info = 0x00, DateTime = 0x01, ActivePower = 0x02, ConsumedEnergy = 0x03 };
enum RecCmd : uint8_t { TimeCorrection = 0x02 };

class CE2727aComponent : public PollingComponent, public uart::UARTDevice {
  SUB_SENSOR(active_power)
  SUB_SENSOR(energy_total)
  SUB_SENSOR(energy_t1)
  SUB_SENSOR(energy_t2)
  SUB_SENSOR(energy_t3)
  SUB_SENSOR(energy_t4)
  SUB_TEXT_SENSOR(tariff)
  SUB_TEXT_SENSOR(date)
  SUB_TEXT_SENSOR(time)
  SUB_TEXT_SENSOR(network_address)
  SUB_TEXT_SENSOR(serial_nr)
  SUB_TEXT_SENSOR(reading_state)
 public:
  CE2727aComponent() = default;

  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_receive_timeout(uint32_t receive_timeout) { this->receive_timeout_ = receive_timeout; }
  void set_requested_meter_address(uint32_t address) { this->requested_meter_address_ = address; }

  float get_setup_priority() const override;

  void dump_config() override;
  void setup() override;

  void loop() override;
  void update() override;

#ifdef USE_TIME
  void set_time_source(time::RealTimeClock *rtc) { this->time_source_ = rtc; };
  void sync_device_time();  // set current time from RTC
#endif

 protected:
  GPIOPin *flow_control_pin_{nullptr};
  uint32_t receive_timeout_{0};
  uint32_t requested_meter_address_{0};
  bool got_time_correction_request_{false};

#ifdef USE_TIME
  time::RealTimeClock *time_source_{nullptr};
#endif

  InternalDataState data_{};

  // Tracker for the current command and response
  struct {
    uint32_t start_time_ms{0};
    uint16_t expected_size{0};
    ReadFunction read_fn{nullptr};
  } request_tracker_{};

  static constexpr size_t RX_BUFFER_SIZE = 64u;
  static constexpr size_t TX_BUFFER_SIZE = 64u;

  struct {
    uint8_t out[TX_BUFFER_SIZE]{};
    size_t amount_out{0};
    uint8_t in[RX_BUFFER_SIZE]{};
    size_t amount_in{0};
  } buffers_{};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    WAITING_FOR_RESPONSE,
    GET_METER_INFO,
    GET_DATE_TIME,
    CORRECT_DATE_TIME,
    GET_ACTIVE_POWER,
    GET_ENERGY,
    PUBLISH_INFO,
  } state_{State::NOT_INITIALIZED};

  State next_state_{State::IDLE};
  State last_reported_state_{State::NOT_INITIALIZED};

  void prepare_enq_request(uint8_t cmd);
  void prepare_rec_time_correction_request(int8_t correction_seconds);

  void send_prepared_request();
  void read_reply_and_go_next_state_(size_t expected_size, ReadFunction read_fn, State next_state);

  bool read_response();

  size_t read_meter_info();
  size_t read_date_time();
  size_t read_active_power();
  size_t read_consumed_energy();
  size_t read_time_correction();

  uint16_t crc_16_iec(const uint8_t *buffer, uint16_t len);

  const char *state_to_string(State state);
  void log_state_(State *next_state = nullptr);
};

}  // namespace ce2727a
}  // namespace esphome