#include "esphome/core/log.h"
#include "ce2727a.h"

namespace esphome
{
    namespace ce2727a
    {

        static const char *TAG = "CE2727A";

        static constexpr size_t rxBufferSize = 64u;
        static std::array<uint8_t, rxBufferSize> rxBuffer;

        static inline int bcd2dec(uint8_t hex)
        {
            assert(((hex & 0xF0) >> 4) < 10); // More significant nybble is valid
            assert((hex & 0x0F) < 10);        // Less significant nybble is valid
            int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
            return dec;
        }

        enum class ComType : uint8_t
        {
            Enq = 0x01,
            Rec = 0x03,
            Drj = 0x0a,
            OK = 0x0b
        };

#pragma pack(1)
        typedef struct
        {
            uint8_t two;
            uint8_t length;
            uint32_t address;
            uint32_t password;
            uint8_t com;
            uint8_t id;
        } ce2727a_frame_header_t;

        typedef struct
        {
            ce2727a_frame_header_t header;
            uint16_t crc16;
        } ce2727a_request_command_t;

        typedef struct
        {
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

        typedef struct
        {
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

        typedef struct
        {
            ce2727a_frame_header_t header;
            uint32_t activePower;
            uint16_t crc16;
        } ce2727a_response_active_power_t;

        typedef struct
        {
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
        static_assert(sizeof(ce2727a_response_consumed_energy_t) == 0x23, "Wrong structure size: ce2727a_response_consumed_energy_t");

        float CE2727aComponent::get_setup_priority() const { return setup_priority::DATA; }

        void CE2727aComponent::dump_config()
        {
            ESP_LOGCONFIG(TAG, "CE2727a:");
            LOG_UPDATE_INTERVAL(this);
            ESP_LOGCONFIG(TAG, "  Meter address requested: %u", this->requested_meter_address_);
            ESP_LOGCONFIG(TAG, "  Receive timeout: %.1fs", this->receive_timeout_ / 1e3f);
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
        }

        void CE2727aComponent::setup()
        {
        }

        void CE2727aComponent::loop()
        {
        }

        void CE2727aComponent::update()
        {
            if (millis() < 10 * 1000) // wait for everything to settle down
                return;

            flush();
            get_meter_info();
            flush();
            get_date_time();
            flush();
            get_active_power();
            flush();
            get_energy_by_tariff();

            if (this->network_address_ != nullptr)
                this->network_address_->publish_state(to_string(data_.networkAddress));
            if (this->serial_nr_ != nullptr)
                this->serial_nr_->publish_state(to_string(data_.serialNumber));

            if (this->date_ != nullptr)
                this->date_->publish_state(data_.dateStr);
            if (this->time_ != nullptr)
                this->time_->publish_state(data_.timeStr);

            if (this->active_power_ != nullptr)
                this->active_power_->publish_state(data_.activePower);

            if (this->tariff_ != nullptr)
                this->tariff_->publish_state(to_string(data_.energy.currentTariff));

            if (this->energy_total_ != nullptr)
                this->energy_total_->publish_state(data_.energy.total);
            if (this->energy_t1_ != nullptr)
                this->energy_t1_->publish_state(data_.energy.t1);
            if (this->energy_t2_ != nullptr)
                this->energy_t2_->publish_state(data_.energy.t2);
            if (this->energy_t3_ != nullptr)
                this->energy_t3_->publish_state(data_.energy.t3);
            if (this->energy_t4_ != nullptr)
                this->energy_t4_->publish_state(data_.energy.t4);
        }

        void CE2727aComponent::send_enquiry_command(EnqCmd cmd)
        {
            if (this->flow_control_pin_ != nullptr)
                this->flow_control_pin_->digital_write(true);

            txBuffer.header.two = 0x02; // magic
            txBuffer.header.length = sizeof(ce2727a_request_command_t);
            txBuffer.header.address = this->requested_meter_address_;
            txBuffer.header.password = 0x0;
            txBuffer.header.com = static_cast<uint8_t>(ComType::Enq);
            txBuffer.header.id = static_cast<uint8_t>(cmd);
            txBuffer.crc16 = crc_16_iec((const uint8_t *)&txBuffer, sizeof(ce2727a_request_command_t) - 2); // minus 2 bytes for CRC

            write_array((const uint8_t *)&txBuffer, sizeof(ce2727a_request_command_t));

            if (this->flow_control_pin_ != nullptr)
                this->flow_control_pin_->digital_write(false);
        }

        bool CE2727aComponent::receive_proper_response(uint16_t expectedSize)
        {
            auto stopWaiting = millis() + this->receive_timeout_;
            uint16_t bytesRead = 0;
            int currentByte = 0;
            ESP_LOGV(TAG, "Expecting %d bytes", expectedSize);
            while ((bytesRead < expectedSize) && (millis() < stopWaiting))
            {
                while (available() > 0 && bytesRead < expectedSize)
                {
                    currentByte = read();
                    if (currentByte >= 0)
                    {
                        rxBuffer[bytesRead++] = (uint8_t)currentByte;
                    }
                    yield();
                }
                delay(5);
                yield();
                ESP_LOGV(TAG, "Got some bytesRead %d", bytesRead);
            }
            ESP_LOGV(TAG, "Bytes expected/read %d/%d", expectedSize, bytesRead);
            // ESP_LOGV(TAG, "Got reponse: %s", format_hex_pretty((const uint8_t *)rxBuffer.data(), bytesRead).c_str());

            if (bytesRead != expectedSize)
            {
                ESP_LOGE(TAG, "receiveProperResponse wrong size");
                data_.readErrors++;
                return false;
            };

            // CRC of message + its CRC = 0x0F47
            if (crc_16_iec(rxBuffer.data(), expectedSize) != 0x0F47)
            {
                ESP_LOGE(TAG, "receiveProperResponse CRC failed");
                data_.readErrors++;
                return false;
            }
            data_.properReads++;
            return true;
        }

        bool CE2727aComponent::get_meter_info()
        {
            ESP_LOGV(TAG, "get_meter_info()");
            send_enquiry_command(EnqCmd::Info);

            if (!receive_proper_response(sizeof(ce2727a_response_info_t)))
                return false;

            ce2727a_response_info_t &res = *(ce2727a_response_info_t *)rxBuffer.data();
            data_.serialNumber = res.serial_number;
            data_.networkAddress = res.network_address;

            ESP_LOGI(TAG, "get_meter_info() Got reply from meter with s/n %u (0x%08X), network address %u, fw ver. %04X",
                     res.serial_number, res.serial_number, res.network_address, res.fw_ver);

            return true;
        }

        bool CE2727aComponent::get_date_time()
        {
            ESP_LOGV(TAG, "get_date_time()");

            data_.dateStr[0] = 0;
            data_.timeStr[0] = 0;

            send_enquiry_command(EnqCmd::DateTime);
            if (!receive_proper_response(sizeof(ce2727a_response_date_time_t)))
                return false;

            ce2727a_response_date_time_t &res = *(ce2727a_response_date_time_t *)rxBuffer.data();

            snprintf(data_.timeStr, sizeof(data_.timeStr), "%02d:%02d:%02d",
                     bcd2dec(res.hour), bcd2dec(res.minute), bcd2dec(res.second));
            snprintf(data_.dateStr, sizeof(data_.dateStr), "%02d/%02d/%02d",
                     bcd2dec(res.day), bcd2dec(res.month), bcd2dec(res.year));

            ESP_LOGI(TAG, "get_date_time() Date %s Time %s", data_.dateStr, data_.timeStr);

            return true;
        }

        bool CE2727aComponent::get_active_power()
        {
            ESP_LOGV(TAG, "get_active_power()");
            send_enquiry_command(EnqCmd::ActivePower);

            if (!receive_proper_response(sizeof(ce2727a_response_active_power_t)))
                return false;

            ce2727a_response_active_power_t &res = *(ce2727a_response_active_power_t *)rxBuffer.data();
            data_.activePower = res.activePower;
            ESP_LOGI(TAG, "get_active_power() %d", res.activePower);

            return true;
        }

        bool CE2727aComponent::get_energy_by_tariff()
        {
            ESP_LOGV(TAG, "get_energy_by_tariff()");
            send_enquiry_command(EnqCmd::ConsumedEnergy);

            if (!receive_proper_response(sizeof(ce2727a_response_consumed_energy_t)))
                return false;

            ce2727a_response_consumed_energy_t &res = *(ce2727a_response_consumed_energy_t *)rxBuffer.data();
            data_.energy.currentTariff = res.currentTariff;
            data_.energy.total = res.total;
            data_.energy.t1 = res.t1;
            data_.energy.t2 = res.t2;
            data_.energy.t3 = res.t3;
            data_.energy.t4 = res.t4;

            ESP_LOGI(TAG, "get_energy_by_tariff() T%d, T1=%d, T2=%d, T3=%d, T4=%d", res.currentTariff, res.t1, res.t2, res.t3, res.t4);
            return true;
        }

        //---------------------------------------------------------------------------------------
        //  The standard 16-bit CRC polynomial specified in ISO/IEC 3309 is used.
        //             16   12   5
        //  Which is: x  + x  + x + 1
        //----------------------------------------------------------------------------
        uint16_t CE2727aComponent::crc_16_iec(const uint8_t *buffer, uint16_t len)
        {
            uint16_t crc = 0xffff;
            uint8_t d;
            do
            {
                d = *buffer++ ^ (crc & 0xFF);
                d ^= d << 4;
                crc = (d << 3) ^ (d << 8) ^ (crc >> 8) ^ (d >> 4);
            } while (--len);
            crc ^= 0xFFFF;
            return crc;
        }
    }
}