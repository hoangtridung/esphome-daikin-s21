#include "daikin_s21_sensor.h"

namespace esphome {
namespace daikin_s21 {

static const char *const TAG = "daikin_s21.sensor";

void DaikinS21Sensor::setup() {
		if (this->s21 == nullptr) {
			 ESP_LOGE(TAG, "s21 is not set!");
			 this->mark_failed();
		}
	}

void DaikinS21Sensor::update() {
  if (this->s21 == nullptr || !this->s21->is_ready() || 						// Kiểm tra `is_ready()` để đảm bảo `DaikinS21` đã sẵn sàng.
      (!this->temp_inside_sensor && !this->temp_outside_sensor &&   // Publish trạng thái của từng sensor (nếu không null)
       !this->temp_coil_sensor && !this->fan_speed_sensor)) {
    return;
  }
  if (this->temp_inside_sensor != nullptr) {
    float temp = this->s21->get_temp_inside();
    if (!std::isnan(temp)) {
      this->temp_inside_sensor->publish_state(temp);
    }
  }
  if (this->temp_outside_sensor != nullptr) {
    float temp = this->s21->get_temp_outside();
    if (!std::isnan(temp)) {
      this->temp_outside_sensor->publish_state(temp);
    }
  }
  if (this->temp_coil_sensor != nullptr) {
    float temp = this->s21->get_temp_coil();
    if (!std::isnan(temp)) {
      this->temp_coil_sensor->publish_state(temp);
    }
  }
  if (this->fan_speed_sensor != nullptr) {
    float rpm = this->s21->get_fan_rpm();
    if (!std::isnan(rpm)) {
      this->fan_speed_sensor->publish_state(rpm);
    }
  }
}

void DaikinS21Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Sensor:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->get_update_interval());
  LOG_SENSOR("  ", "Temperature Inside", this->temp_inside_sensor);
  LOG_SENSOR("  ", "Temperature Outside", this->temp_outside_sensor);
  LOG_SENSOR("  ", "Temperature Coil", this->temp_coil_sensor);
  LOG_SENSOR("  ", "Fan Speed", this->fan_speed_sensor);
}

}  // namespace daikin_s21
}  // namespace esphome