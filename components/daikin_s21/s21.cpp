#include "s21.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

#define STX 2
#define ETX 3
#define ACK 6
#define NAK 21

#define S21_BAUD_RATE 2400
#define S21_STOP_BITS 2
#define S21_DATA_BITS 8
#define S21_PARITY uart::UART_CONFIG_PARITY_EVEN

#define S21_RESPONSE_TIMEOUT 250 // Giảm từ 500 xuống 250 khi sửa hàm s21_query() với retry logic

static const char *const TAG = "daikin_s21";


// ------ Utility function -------------
// -------------------------------------

inline float c10_c(int16_t c10) { return c10 / 10.0; } // inline: nhúng nội dung hàm vào chỗ được gọi (thay vì gọi qua stack), giảm chi phí gọi hàm.
inline float c10_f(int16_t c10) { return c10_c(c10) * 1.8 + 32.0; }
std::string daikin_climate_mode_to_string(DaikinClimateMode mode) {
  switch (mode) {
    case DaikinClimateMode::Off:
      return "Disabled";
    case DaikinClimateMode::Auto:
      return "Auto";
    case DaikinClimateMode::Dry:
      return "Dry";
    case DaikinClimateMode::Cool:
      return "Cool";
    case DaikinClimateMode::Heat:
      return "Heat";
    case DaikinClimateMode::FanOnly:
      return "Fan";
    default:
      return "UNKNOWN";
  }
}

std::string daikin_fan_mode_to_string(DaikinFanMode mode) {
  switch (mode) {
    case DaikinFanMode::Auto:
      return "Auto";
    case DaikinFanMode::Silent:
      return "Silent";
    case DaikinFanMode::Speed1:
      return "1";
    case DaikinFanMode::Speed2:
      return "2";
    case DaikinFanMode::Speed3:
      return "3";
    case DaikinFanMode::Speed4:
      return "4";
    case DaikinFanMode::Speed5:
      return "5";
    default:
      return "UNKNOWN";
  }
}

uint8_t s21_checksum(uint8_t *bytes, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum += bytes[i];
  }
  return checksum;
}
uint8_t s21_checksum(std::vector<uint8_t> bytes) {
  return s21_checksum(&bytes[0], bytes.size());
}

// Command: RL / Response: SL / Payload length: ?
// Fan speed
int16_t bytes_to_num(uint8_t *bytes, size_t len) {
  // <ones><tens><hundreds><neg/pos>
  int16_t val = 0;
  val = bytes[0] - '0';
  val += (bytes[1] - '0') * 10;
  val += (bytes[2] - '0') * 100;
  if (len > 3 && bytes[3] == '-')
    val *= -1;
  return val;
}
// -----------------------------
 
/* 
ASCII character representing a setpoint temperature that a remote can send to the unit.
	Value	Meaning
	"@"		18.0C
	"A"		18.5C
	…	
	"W"		29.5C
	"X"		30.0C
	0x80	N/A (for dry mode)
 */
int16_t bytes_to_num(std::vector<uint8_t> &bytes) {
  return bytes_to_num(&bytes[0], bytes.size());
}
int16_t temp_bytes_to_c10(uint8_t *bytes) { 
	return bytes_to_num(bytes, 4); 
}
int16_t temp_bytes_to_c10(std::vector<uint8_t> &bytes) {
  return temp_bytes_to_c10(&bytes[0]);
}
// -----------------------------------------------------

// 
/* 
ASCII character representing a setpoint temperature that a remote can send to the unit.
	Value	Meaning
	"@"		18.0C
	"A"		18.5C
	…	
	"W"		29.5C
	"X"		30.0C
	0x80	N/A (for dry mode)
 */
inline uint8_t c10_to_setpoint_byte(int16_t setpoint) { return (setpoint + 3) / 5 + 28; }
// --------------------------------------------------------------------------------------

/* 
int16_t temp_f9_byte_to_c10(uint8_t *bytes) { return (*bytes / 2 - 64) * 10; }

uint8_t c10_to_setpoint_byte(int16_t setpoint) {
  return (setpoint + 3) / 5 + 28;
}

void DaikinS21::set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx) {
  this->tx_uart = tx;
  this->rx_uart = rx;
}
 */
 
// Command: F9 / Response: G9 / Payload length: 4 byte
/* Format: 
bytes 0, 1 are 0x80 + value/2.
Byte 2 is '0' + value. 
Meaning:
Byte 0: indoor temperature in C (granularity of 1C, rounded down), 
byte 1: outdoor temperature in C (granularity of 1C, rounded down), 
byte 2: indoor humidity in units of 1% (granularity of 5%, rounded down). 
e.g. 
0xac:aa:4e:30 22C inside, 21C outside, 30% humidity
 */
inline int16_t temp_f9_byte_to_c10(uint8_t *bytes) { return (*bytes / 2 - 64) * 10; }
// -----------------------------------------------------------------------------------


// Adapated from ESPHome UART debugger
std::string hex_repr(uint8_t *bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (i > 0)
      res += ':';
    sprintf(buf, "%02X", bytes[i]);
    res += buf;
  }
  return res;
}

// Adapated from ESPHome UART debugger
std::string str_repr(uint8_t *bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (bytes[i] == 7) {
      res += "\\a";
    } else if (bytes[i] == 8) {
      res += "\\b";
    } else if (bytes[i] == 9) {
      res += "\\t";
    } else if (bytes[i] == 10) {
      res += "\\n";
    } else if (bytes[i] == 11) {
      res += "\\v";
    } else if (bytes[i] == 12) {
      res += "\\f";
    } else if (bytes[i] == 13) {
      res += "\\r";
    } else if (bytes[i] == 27) {
      res += "\\e";
    } else if (bytes[i] == 34) {
      res += "\\\"";
    } else if (bytes[i] == 39) {
      res += "\\'";
    } else if (bytes[i] == 92) {
      res += "\\\\";
    } else if (bytes[i] < 32 || bytes[i] > 127) {
      sprintf(buf, "\\x%02X", bytes[i]);
      res += buf;
    } else {
      res += bytes[i];
    }
  }
  return res;
}

std::string str_repr(std::vector<uint8_t> &bytes) {
  return str_repr(&bytes[0], bytes.size());
}
 
// Thêm daikin_swing_mode_to_string: Hàm này chuyển DaikinSwingMode thành chuỗi để hiển thị trong dump_state
std::string daikin_swing_mode_to_string(DaikinSwingMode mode) {
  switch (mode) {
    case DaikinSwingMode::OFF:
      return "OFF";
    case DaikinSwingMode::VERTICAL:
      return "VERTICAL";
    case DaikinSwingMode::HORIZONTAL:
      return "HORIZONTAL";
    case DaikinSwingMode::BOTH:
      return "BOTH";
    default:
      return "UNKNOWN";
  }
}


// ------ Class function implentation -------------
// ------------------------------------------------
void DaikinS21::set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx) {
  this->tx_uart = tx;
  this->rx_uart = rx;
}
void DaikinS21::check_uart_settings() {
  for (auto uart : {this->tx_uart, this->rx_uart}) {
    if (uart->get_baud_rate() != S21_BAUD_RATE) {
      ESP_LOGE(
          TAG,
          "  Invalid baud_rate: Integration requested baud_rate %u but you "
          "have %u!",
          S21_BAUD_RATE, uart->get_baud_rate());
    }
    if (uart->get_stop_bits() != S21_STOP_BITS) {
      ESP_LOGE(
          TAG,
          "  Invalid stop bits: Integration requested stop_bits %u but you "
          "have %u!",
          S21_STOP_BITS, uart->get_stop_bits());
    }
    if (uart->get_data_bits() != S21_DATA_BITS) {
      ESP_LOGE(TAG,
               "  Invalid number of data bits: Integration requested %u data "
               "bits but you have %u!",
               S21_DATA_BITS, uart->get_data_bits());
    }
    if (uart->get_parity() != S21_PARITY) {
      ESP_LOGE(
          TAG,
          "  Invalid parity: Integration requested parity %s but you have %s!",
          LOG_STR_ARG(parity_to_str(S21_PARITY)),
          LOG_STR_ARG(parity_to_str(uart->get_parity())));
    }
  }
}

void DaikinS21::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
  this->check_uart_settings();
}

bool DaikinS21::wait_byte_available(uint32_t  timeout)
{
  uint32_t start = millis();
  bool reading = false;
  while (true) {
    if (millis() - start > timeout) {
      ESP_LOGW(TAG, "Timeout waiting for byte");
      return false;
    }
    if(this->rx_uart->available())
      return true;
    yield();
  }
}

bool DaikinS21::read_frame(std::vector<uint8_t> &payload) {
  uint8_t byte;
  std::vector<uint8_t> bytes;
  uint32_t start = millis();
  bool reading = false;
  while (true) {
    if (millis() - start > S21_RESPONSE_TIMEOUT) {
      ESP_LOGW(TAG, "Timeout waiting for frame");
      return false;
    }
    while (this->rx_uart->available()) {
      this->rx_uart->read_byte(&byte);
      if (byte == ACK) {
        ESP_LOGW(TAG, "Unexpected ACK waiting to read start of frame");
        continue;
      } else if (!reading && byte != STX) {
        ESP_LOGW(TAG, "Unexpected byte waiting to read start of frame: %x",
                 byte);
        continue;
      } else if (byte == STX) {
        reading = true;
        continue;
      }
      if (byte == ETX) {
        reading = false;
        uint8_t frame_csum = bytes[bytes.size() - 1];
        bytes.pop_back();
        uint8_t calc_csum = s21_checksum(bytes);
        if (calc_csum != frame_csum) {
          // This sometimes happens with G9 reply, no idea why
          if (bytes[0] == 0x47 && bytes[1] == 0x39) {
            calc_csum += 2;
          }
          if (calc_csum != frame_csum) {
            ESP_LOGW(TAG, "Checksum mismatch: %x (frame) != %x (calc from %s)",
            frame_csum, calc_csum,
            hex_repr(&bytes[0], bytes.size()).c_str());
            return false;
          }
        }
        break;
      }
      bytes.push_back(byte);
    }
    if (bytes.size() && !reading)
      break;
    yield();
  }
  payload.assign(bytes.begin(), bytes.end());
  return true;
}

void DaikinS21::write_frame(std::vector<uint8_t> frame) {
  this->tx_uart->write_byte(STX);
  this->tx_uart->write_array(frame);
  this->tx_uart->write_byte(s21_checksum(&frame[0], frame.size()));
  this->tx_uart->write_byte(ETX);
  this->tx_uart->flush();
}
bool DaikinS21::s21_query(std::vector<uint8_t> code) {
  std::string c(code.begin(), code.end());
  constexpr int MAX_RETRIES = 3;
  int attempt = 0;

  while (attempt++ < MAX_RETRIES) {
    ESP_LOGD(TAG, "Sending query %s (attempt %d/%d)", c.c_str(), attempt, MAX_RETRIES);
    this->write_frame(code);

    if (!this->wait_byte_available(S21_RESPONSE_TIMEOUT)) {
      ESP_LOGW(TAG, "No byte received for %s query", c.c_str());
      continue;
    }

    uint8_t byte;
    if (!this->rx_uart->read_byte(&byte)) {
      ESP_LOGW(TAG, "Failed to read first byte for %s", c.c_str());
      continue;
    }

    if (byte == NAK) {
      ESP_LOGW(TAG, "NAK received for %s", c.c_str());
      continue;
    }

    if (byte != ACK) {
      ESP_LOGW(TAG, "Unexpected byte (0x%02X) instead of ACK for %s", byte, c.c_str());
      continue;
    }

    std::vector<uint8_t> frame;
    if (!this->read_frame(frame)) {
      ESP_LOGW(TAG, "Failed to read frame for %s", c.c_str());
      continue;
    }

    this->tx_uart->write_byte(ACK);

    std::vector<uint8_t> rcode(frame.begin(), frame.begin() + code.size());
    std::vector<uint8_t> payload(frame.begin() + code.size(), frame.end());

    return parse_response(rcode, payload);
  }

  ESP_LOGE(TAG, "Failed query %s after %d attempts", c.c_str(), MAX_RETRIES);
  return false;
}

/* 
📋 Tóm tắt từng truy vấn
Mã	Parse_response	Giải thích chức năng
F1	G1	Trạng thái chính: power, mode, fan, setpoint
F5	G5	Trạng thái swing H/V
F6	G6	Powerful mode → this->powerful
F7	G7	Econo mode → this->econo
F9	G9	Temp inside & outside (bằng byte)
RH	SH	Nhiệt độ bên trong
RI	SI	Nhiệt độ cuộn dây
Ra	Sa	Nhiệt độ  ngoài trời
RL	SL	Fan RPM
Rd	Sd	Compressor idle/trạng thái chạy
 */
bool DaikinS21::parse_response(std::vector<uint8_t> rcode, std::vector<uint8_t> payload) {
  if (this->debug_protocol) {
    ESP_LOGD(TAG, "S21: %s -> %s (%d)", str_repr(rcode).c_str(), str_repr(payload).c_str(), payload.size());
  }
  switch (rcode[0]) {
    case 'G':
      switch (rcode[1]) {
				case '1':  // F1 -> Basic State
          if (payload.size() < 4) {
            ESP_LOGW(TAG, "Invalid G1 payload size: %d", payload.size());
            return false;
          }
          this->power_on = (payload[0] == '1');
          if (payload[0] == '0' && payload[1] == '0') { // payload [0,1] ='00' --> Off
            this->mode = DaikinClimateMode::Off;
          } else if (payload[0] == '1') {
            switch (payload[1]) {
              case '0':
                this->mode = DaikinClimateMode::Auto;		// payload [0,1] ='10' --> Auto
                break;
              case '2':
                this->mode = DaikinClimateMode::Dry;
                break;
              case '3':
                this->mode = DaikinClimateMode::Cool;
                break;
              case '4':
                this->mode = DaikinClimateMode::Heat;
                break;
              case '6':
                this->mode = DaikinClimateMode::FanOnly;
                break;
              default:
                ESP_LOGW(TAG, "Unknown climate mode value: %c", payload[1]);
                return false;
            }
          } else {
            ESP_LOGW(TAG, "Invalid power state: %c", payload[0]);
            return false;
          }
          this->setpoint = ((payload[2] - 28) * 5);  // Celsius * 10
          this->fan = (DaikinFanMode) payload[3];
          return true;
				case '5': // F5 -> G5 -- Swing state
					switch (payload[0]) {
						case 1:
							this->swing_mode = DaikinSwingMode::VERTICAL;
							break;
						case 2:
							this->swing_mode = DaikinSwingMode::HORIZONTAL;
							break;
						case 7:
							this->swing_mode = DaikinSwingMode::BOTH;
							break;
						case '0':
							this->swing_mode = DaikinSwingMode::OFF;
							break;
						default:
							ESP_LOGW(TAG, "Unknown swing mode value: %d", payload[0]);
							return false;
					}
					return true;
        case '6':                // F6 -> G6 - "powerful" mode
          this->powerful = (payload[0] == '2') ? 1 : 0;
          return true;
        case '7':                // F7 - G7 - "eco" mode
          this->econo = (payload[1] == '2') ? 1 : 0;
          return true;
        case '9':  							 // F9 -> G9 -- Inside temperature
          this->temp_inside = temp_f9_byte_to_c10(&payload[0]);
          this->temp_outside = temp_f9_byte_to_c10(&payload[1]);
          return true;					
      }
      break;
			
    case 'S':      // R -> S
      switch (rcode[1]) {
        case 'H':  // Inside temperature
          this->temp_inside = temp_bytes_to_c10(payload);
          return true;
        case 'I':  // Coil temperature
          this->temp_coil = temp_bytes_to_c10(payload);
          return true;
        case 'a':  // Outside temperature
          this->temp_outside = temp_bytes_to_c10(payload);
          return true;
        case 'L':  // Fan speed
          this->fan_rpm = bytes_to_num(payload) * 10;
          return true;
        case 'd':  // Compressor state / frequency? Idle if 0.
          this->idle =
              (payload[0] == '0' && payload[1] == '0' && payload[2] == '0');
          return true;
        default:
          if (payload.size() > 3) {
            int8_t temp = temp_bytes_to_c10(payload);
            ESP_LOGD(TAG, "Unknown temp: %s -> %s -> %.1f C (%.1f F)",
                     str_repr(rcode).c_str(), str_repr(payload).c_str(),
                     c10_c(temp), c10_f(temp));
          }
          return false;
      }
  }
  ESP_LOGD(TAG, "Unknown response %s -> \"%s\"", str_repr(rcode).c_str(),
           str_repr(payload).c_str());
  return false;
}

bool DaikinS21::run_queries(std::vector<std::string> queries) {
  bool success = true;

  for (auto q : queries) {
    std::vector<uint8_t> code(q.begin(), q.end());
    success = this->s21_query(code) && success;
  }

  return success;  // True if all queries successful
}

void DaikinS21::update() {
  std::vector<std::string> queries = {"F1", "F5", "RH", "RI", "Ra", "RL", "Rd"};
  // These queries might fail but they won't affect the basic functionality
  std::vector<std::string> failable_queries = {"F3"};
  if (this->run_queries(queries)) {
    this->run_queries(failable_queries);
    if(!this->ready) {
      ESP_LOGI(TAG, "Daikin S21 Ready");
      this->ready = true;
    }
  }
  if (this->debug_protocol) {
    this->dump_state();
  }
}
/* 
#ifdef S21_EXPERIMENTS
  ESP_LOGD(TAG, "** UNKNOWN QUERIES **");
  // auto experiments = {"F2", "F3", "F4", "F8", "F9", "F0", "FA", "FB", "FC",
  //                     "FD", "FE", "FF", "FG", "FH", "FI", "FJ", "FK", "FL",
  //                     "FM", "FN", "FO", "FP", "FQ", "FR", "FS", "FT", "FU",
  //                     "FV", "FW", "FX", "FY", "FZ"};
  // Observed BRP device querying these.
  std::vector<std::string> experiments = {"F2", "F3", "F4", "RN",
                                          "RX", "RD", "M",  "FU0F"};
  this->run_queries(experiments);
#endif
 */


void DaikinS21::dump_state() {
  ESP_LOGV(TAG, "** BEGIN STATE *****************************");

  ESP_LOGV(TAG, "  Power: %s", ONOFF(this->power_on));
  ESP_LOGV(TAG, "   Mode: %s (%s)",
           daikin_climate_mode_to_string(this->mode).c_str(),
           this->idle ? "idle" : "active");
  float degc = this->setpoint / 10.0;
  float degf = degc * 1.8 + 32.0;
  ESP_LOGV(TAG, " Target: %.1f C (%.1f F)", degc, degf);
  ESP_LOGV(TAG, "    Fan: %s (%d rpm)",
           daikin_fan_mode_to_string(this->fan).c_str(), this->fan_rpm);
  ESP_LOGV(TAG, "  Swing: %s",
           daikin_swing_mode_to_string(this->swing_mode).c_str());
  ESP_LOGV(TAG, " Inside: %.1f C (%.1f F)", c10_c(this->temp_inside),
           c10_f(this->temp_inside));
  ESP_LOGV(TAG, "Outside: %.1f C (%.1f F)", c10_c(this->temp_outside),
           c10_f(this->temp_outside));
  ESP_LOGV(TAG, "   Coil: %.1f C (%.1f F)", c10_c(this->temp_coil),
           c10_f(this->temp_coil));

  ESP_LOGV(TAG, "** END STATE *****************************");
}
void DaikinS21::set_swing_mode(DaikinSwingMode swing_mode) {
  uint8_t swing_value = 0;
  switch (swing_mode) {
    case DaikinSwingMode::OFF:
      swing_value = '0';
      break;
    case DaikinSwingMode::VERTICAL:
      swing_value = '1';
      break;
    case DaikinSwingMode::HORIZONTAL:
      swing_value = '2';
      break;
    case DaikinSwingMode::BOTH:
      swing_value = '7'; 
      break;
    default:
      ESP_LOGW(TAG, "Unsupported swing mode: %d", static_cast<int>(swing_mode));
      return;
  }

  std::vector<uint8_t> cmd = {swing_value, (swing_value == '0' ? '0' : '?'), '0', '0'};
  ESP_LOGD(TAG, "Sending swing CMD (D5): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '5'}, cmd)) {
    ESP_LOGW(TAG, "Failed swing CMD");
  } else {
    this->swing_mode = swing_mode;
    this->update();
  }
}

void DaikinS21::set_daikin_climate_settings(bool power_on, DaikinClimateMode mode, float setpoint, DaikinFanMode fan_mode) {
  uint32_t start = millis();
  ESP_LOGD(TAG, "[set_daikin_climate_settings] Start ");
  if (std::isnan(setpoint)) {
    ESP_LOGW(TAG, "Invalid setpoint: nan, using default 23.0°C");
    setpoint = 23.0;
  }
  uint8_t power_byte = power_on ? '1' : '0';
  uint8_t mode_byte = '0';  // Default for Off // payload [0,1] ='00' --> Off
  if (power_on) {
    switch (mode) {
      case DaikinClimateMode::Auto:	// payload [0,1] ='10' --> Auto
        mode_byte = '0';
        break;
      case DaikinClimateMode::Dry:
        mode_byte = '2';
        break;
      case DaikinClimateMode::Cool:
        mode_byte = '3';
        break;
      case DaikinClimateMode::Heat:
        mode_byte = '4';
        break;
      case DaikinClimateMode::FanOnly:
        mode_byte = '6';
        break;
      default:
        ESP_LOGW(TAG, "Unsupported climate mode: %d", static_cast<int>(mode));
        return;
    }
  }
  std::vector<uint8_t> cmd = {
      power_byte,
      mode_byte,
      c10_to_setpoint_byte(lroundf(round(setpoint * 2) / 2 * 10.0)),
      (uint8_t) fan_mode
  };
  ESP_LOGD(TAG, "Sending basic climate CMD (D1): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '1'}, cmd)) {
    ESP_LOGW(TAG, "Failed basic climate CMD");
  } else {
    this->power_on = power_on;
    this->mode = mode;
    this->setpoint = lroundf(round(setpoint * 2) / 2 * 10.0);
    this->fan = fan_mode;
    this->s21_query({'F', '1'});
  }
  ESP_LOGD(TAG, "[set_daikin_climate_settings] Done (%lu ms total)", millis() - start);
}
 
void DaikinS21::set_powerful_settings(bool value)
{
  std::vector<uint8_t> cmd = {
      (uint8_t) ('0' + (value ? 2 : 0)), '0', '0', '0'};
  ESP_LOGD(TAG, "Sending swing CMD (D6): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '6'}, cmd)) {
    ESP_LOGW(TAG, "Failed powerful CMD");
  } else {
    this->update();
  }
}

void DaikinS21::set_econo_settings(bool value)
{
  std::vector<uint8_t> cmd = {
      '0', (uint8_t) ('0' + (value ? 2 : 0)), '0', '0'};
  ESP_LOGD(TAG, "Sending swing CMD (D7): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '7'}, cmd)) {
    ESP_LOGW(TAG, "Failed econo CMD");
  } else {
    this->update();
  }
}

bool DaikinS21::send_cmd(std::vector<uint8_t> code, std::vector<uint8_t> payload) {
  std::vector<uint8_t> frame;
  for (auto b : code) frame.push_back(b);
  for (auto b : payload) frame.push_back(b);
  ESP_LOGV(TAG, "Sending frame: %s", str_repr(frame).c_str());
  delay(50);
  this->write_frame(frame);
  ESP_LOGD(TAG, "Waiting for byte, timeout: %d ms", S21_RESPONSE_TIMEOUT);
  if (!this->wait_byte_available(S21_RESPONSE_TIMEOUT)) {
    ESP_LOGW(TAG, "Timeout waiting for byte after %d ms", S21_RESPONSE_TIMEOUT);
    return false;
  }
  ESP_LOGD(TAG, "Byte available, reading...");
  uint8_t byte;
  if (!this->rx_uart->read_byte(&byte)) {
    ESP_LOGW(TAG, "Failed to read byte after timeout");
    return false;
  }
  ESP_LOGD(TAG, "Received byte: 0x%02X", byte);
  if (byte == NAK) {
    ESP_LOGW(TAG, "Got NAK for frame: %s", str_repr(frame).c_str());
    return false;
  }
  if (byte != ACK) {
    ESP_LOGW(TAG, "Unexpected byte waiting for ACK: 0x%02X", byte);
    return false;
  }
  ESP_LOGD(TAG, "Received ACK, command successful");
  return true;
}

}  // namespace daikin_s21
}  // namespace esphome
