#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace daikin_s21 {

// Scoped, type-safe enums
enum class DaikinClimateMode : uint8_t {
  Off = '0',			// 0x30: Off (when power_on = false)
  Auto = '1',			// 0x30: Auto mode (0x30:HEAT_COOL, when power_on = true)
  Dry = '2',			// 0x32: Dehumidifying mode
  Cool = '3',			// 0x33: Cooling mode
  Heat = '4',			// 0x34: Heating mode
  FanOnly = '6',	// 0x36: Fan only mode
};

enum class DaikinFanMode : uint8_t {
  Auto = 'A',			// 0x41: Automatic fan speed
  Silent = 'B',   // 0x42: Silent mode
  Speed1 = '3',   // 0x33: Fan speed level 1
  Speed2 = '4',   // 0x34: Fan speed level 2
  Speed3 = '5',   // 0x35: Fan speed level 3
  Speed4 = '6',   // 0x36: Fan speed level 4
  Speed5 = '7',   // 0x37: Fan speed level 5
};

enum class DaikinSwingMode : uint8_t {
  OFF = 0,				// 0x00: OFF 
  VERTICAL = 1,		// 0x01: VERTICAL Swing
  HORIZONTAL = 2,	// 0x02: HORIZONTAL Swing
  BOTH = 7, 			// 0x07: BOTH Swing
};

class DaikinS21 : public PollingComponent {
 public:
  void update() override;
  void dump_config() override;

  void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx);
  void set_debug_protocol(bool set) { this->debug_protocol = set; }
  bool is_ready() const { return this->ready; }

  bool is_power_on() const { return this->power_on; }
  DaikinClimateMode get_climate_mode() const { return this->mode; }
  DaikinFanMode get_fan_mode() const { return this->fan; }
  float get_setpoint() const { return this->setpoint / 10.0; }
  float get_temp_inside() const { return this->temp_inside / 10.0; }
  float get_temp_outside() const { return this->temp_outside / 10.0; }
  float get_temp_coil() const { return this->temp_coil / 10.0; }
  uint16_t get_fan_rpm() const { return this->fan_rpm; }
  bool is_idle() const { return this->idle; }
  DaikinSwingMode get_swing_mode() const { return this->swing_mode; }

  void set_daikin_climate_settings(bool power_on, DaikinClimateMode mode, float setpoint, DaikinFanMode fan_mode);
  void set_swing_mode(DaikinSwingMode swing_mode);
  
  bool get_powerful() const { return this->powerful; }
  bool get_econo() const { return this->econo; }
  void set_powerful_settings(bool value);
  void set_econo_settings(bool value);
  void set_has_presets(bool value) { this->has_presets = value; }

 protected:
  bool read_frame(std::vector<uint8_t> &payload);
  void write_frame(std::vector<uint8_t> payload);
  bool s21_query(std::vector<uint8_t> code);
	bool send_cmd(std::vector<uint8_t> code, std::vector<uint8_t> payload);
  bool parse_response(std::vector<uint8_t> rcode, std::vector<uint8_t> payload);
  bool run_queries(std::vector<std::string> queries);
  void dump_state();
  void check_uart_settings();
  bool wait_byte_available(uint32_t timeout);

 private:
  uart::UARTComponent *tx_uart{nullptr};
  uart::UARTComponent *rx_uart{nullptr};
  bool ready{false};
  bool debug_protocol{false};

  bool power_on{false};
  DaikinClimateMode mode{DaikinClimateMode::Off};
  DaikinFanMode fan{DaikinFanMode::Auto};
  int16_t setpoint{230};
  DaikinSwingMode swing_mode{DaikinSwingMode::OFF};
  int16_t temp_inside{0};
  int16_t temp_outside{0};
  int16_t temp_coil{0};
  uint16_t fan_rpm{0};
  bool idle{true};

  bool powerful{false};
  bool econo{false};
  bool has_presets{true};
};

class DaikinS21Client {
 public:
  void set_s21(DaikinS21 *s21) { this->s21 = s21; }

 protected:
  DaikinS21 *s21; 
};

}  // namespace daikin_s21
}  // namespace esphome
		