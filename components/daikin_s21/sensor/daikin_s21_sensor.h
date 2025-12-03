#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "../s21.h"

namespace esphome {
namespace daikin_s21 {

class DaikinS21Sensor : public PollingComponent, public DaikinS21Client {
 public:
	void setup() override;
  void update() override;
  void dump_config() override;

  /// Set the sensor for inside temperature.
  void set_temp_inside_sensor(sensor::Sensor *sensor) { this->temp_inside_sensor = sensor; }
  /// Set the sensor for outside temperature.
  void set_temp_outside_sensor(sensor::Sensor *sensor) { this->temp_outside_sensor = sensor; }
  /// Set the sensor for coil temperature.
  void set_temp_coil_sensor(sensor::Sensor *sensor) { this->temp_coil_sensor = sensor; }
  /// Set the sensor for fan speed (in RPM).
  void set_fan_speed_sensor(sensor::Sensor *sensor) { this->fan_speed_sensor = sensor; }

 protected:
  sensor::Sensor *temp_inside_sensor{nullptr};
  sensor::Sensor *temp_outside_sensor{nullptr};
  sensor::Sensor *temp_coil_sensor{nullptr};
  sensor::Sensor *fan_speed_sensor{nullptr};
};

}  // namespace daikin_s21
}  // namespace esphome