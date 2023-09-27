#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome
{
    namespace ce2727a
    {

        typedef struct
        {
            struct Readings
            {
                uint8_t currentTariff;
                uint32_t total{0};
                uint32_t t1{0};
                uint32_t t2{0};
                uint32_t t3{0};
                uint32_t t4{0};
            } energy;
            uint32_t activePower{0};
            char timeStr[9]{0}; //"23:59:99"
            char dateStr[9]{0}; // "30/08/23"
            uint32_t serialNumber{0};
            uint32_t networkAddress{0};
            uint32_t properReads{0};
            uint32_t readErrors{0};
        } InternalDataState;

        enum class EnqCmd : uint8_t
        {
            Info = 0x00,
            DateTime = 0x01,
            ActivePower = 0x02,
            ConsumedEnergy = 0x03
        };

        class CE2727aComponent : public PollingComponent, public uart::UARTDevice
        {
        public:
            CE2727aComponent() = default;

            void set_active_power_sensor(sensor::Sensor *active_power) { active_power_ = active_power; }
            void set_energy_total_sensor(sensor::Sensor *energy_total) { energy_total_ = energy_total; }
            void set_energy_t1_sensor(sensor::Sensor *energy_t1) { energy_t1_ = energy_t1; }
            void set_energy_t2_sensor(sensor::Sensor *energy_t2) { energy_t2_ = energy_t2; }
            void set_energy_t3_sensor(sensor::Sensor *energy_t3) { energy_t3_ = energy_t3; }
            void set_energy_t4_sensor(sensor::Sensor *energy_t4) { energy_t4_ = energy_t4; }

            void set_electricity_tariff_text_sensor(text_sensor::TextSensor *tariff) { tariff_ = tariff; }
            void set_date_text_sensor(text_sensor::TextSensor *date) { date_ = date; }
            void set_time_text_sensor(text_sensor::TextSensor *time) { time_ = time; }
            void set_network_address_text_sensor(text_sensor::TextSensor *address) { network_address_ = address; }
            void set_serial_nr_text_sensor(text_sensor::TextSensor *serial_nr) { serial_nr_ = serial_nr; }

            void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
            void set_receive_timeout(uint32_t receive_timeout) { this->receive_timeout_ = receive_timeout; }
            void set_requested_meter_address(uint32_t address) { this->requested_meter_address_ = address; }

            float get_setup_priority() const override;

            void dump_config() override;
            void setup() override;

            void loop() override;
            void update() override;

        protected:
            sensor::Sensor *active_power_{nullptr};
            sensor::Sensor *energy_total_{nullptr};
            sensor::Sensor *energy_t1_{nullptr};
            sensor::Sensor *energy_t2_{nullptr};
            sensor::Sensor *energy_t3_{nullptr};
            sensor::Sensor *energy_t4_{nullptr};

            text_sensor::TextSensor *tariff_{nullptr};
            text_sensor::TextSensor *date_{nullptr};
            text_sensor::TextSensor *time_{nullptr};
            text_sensor::TextSensor *network_address_{nullptr};
            text_sensor::TextSensor *serial_nr_{nullptr};

            GPIOPin *flow_control_pin_{nullptr};
            uint32_t receive_timeout_{0};
            uint32_t requested_meter_address_{0};

            InternalDataState data_{};

            void send_enquiry_command(EnqCmd cmd);
            bool receive_proper_response(uint16_t expectedSize);

            bool get_meter_info();
            bool get_date_time();
            bool get_active_power();
            bool get_energy_by_tariff();

            uint16_t crc_16_iec(const uint8_t *buffer, uint16_t len);
        };

    }
}