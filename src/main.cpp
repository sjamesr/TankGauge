#include <Arduino.h>
#include <sensors/digital_input.h>
#include <sensors/digital_output.h>

#include "sensesp_app.h"

namespace {

// Represents a HC-SR04 acoustic rangefinder. Emits the range value in meters.
class HCSR04RangeFinder : public Sensor, public NumericProducer {
public:
  /**
   * Construct a new HCSR04RangeFinder sensor.
   *
   * @param trigger_pin The GPIO pin used to emit the trigger pulse to commence
   * a measurement.
   * @param echo_pin The GPIO pin used to read the result from the sensor.
   * @param read_delay_ms How often the sensor emits the last computed distance.
   * @param measurement_interval_ms The interval between measurements.
   * @param measurement_count Number of measurements to be averaged to compute
   * the distance.
   * @param config_path The path to configure sensor parameters in the UI.
   */
  HCSR04RangeFinder(uint8_t trigger_pin, uint8_t echo_pin,
                    unsigned int read_delay_ms,
                    unsigned int measurement_interval_ms,
                    unsigned int measurement_count, String config_path)
      : Sensor(std::move(config_path)), NumericProducer(),
        trigger_pin_(new DigitalOutput(trigger_pin)), echo_pin_{echo_pin},
        read_delay_ms_{read_delay_ms},
        measurement_interval_ms_{measurement_interval_ms},
        measurement_count_{measurement_count} {
    assert(measurement_count > 0);
  }

  void enable() override {
    pinMode(echo_pin_, INPUT);

    app.onInterrupt(echo_pin_, CHANGE, [this]() { this->isr(); });

    app.onRepeat(measurement_interval_ms_, [this]() { beginMeasurement(); });

    app.onRepeat(read_delay_ms_, [this]() {
      noInterrupts();
      output = static_cast<float>(computed_distance_m_);
      interrupts();
      notify();
    });
  }

private:
  const static int kInitialPulseWidth_us = 150;

  // Initiate a single measurement.
  void beginMeasurement() {
    trigger_pin_->set_input(true);

    app.onDelayMicros(kInitialPulseWidth_us,
                      [this]() { trigger_pin_->set_input(false); });
  }

  void isr() {
    int_count_++;
    unsigned long now = micros();
    if (waiting) {
      //      // We got the falling edge, our measurement is complete.
      total_flight_time_us_ += now - pulse_began_us_;
      current_measurement_count_++;
      waiting = false;
    } else {
      //      // We received a rising edge, our measurement is beginning.
      waiting = true;
      pulse_began_us_ = now;
    }
    //
    if (current_measurement_count_ == measurement_count_) {
      // In this case, we've completed our samples, time to
      // update our computed distance.
      computed_distance_m_ = (static_cast<double>(total_flight_time_us_) /
                              (current_measurement_count_)) *
                             0.000172;

      current_measurement_count_ = 0;
      total_flight_time_us_ = 0;
    }
  }

  DigitalOutput *trigger_pin_;
  const uint8_t echo_pin_;
  const unsigned int read_delay_ms_;
  const unsigned int measurement_interval_ms_;
  const unsigned int measurement_count_;

  volatile unsigned int int_count_ = 0;

  // How many measurements we've taken in the current distance computation.
  volatile unsigned int current_measurement_count_ = 0;

  // The total flight time of all measurements.
  volatile unsigned long total_flight_time_us_ = 0;

  // Whether we're waiting for a falling edge interrupt from the sensor.
  bool waiting = false;
  unsigned long pulse_began_us_ = 0;

  // The final computed distance.
  volatile double computed_distance_m_ = 0.0f;
};

} // namespace

ReactESP app([]() {
  sensesp_app = new SensESPApp();

  SetupSerialDebug(115200);

  {
    const uint8_t trigger_pin = 32;
    const uint8_t echo_pin = 25;
    const unsigned long read_delay_ms = 1000;
    const unsigned long measurement_interval_ms = 100;
    const unsigned int measurement_count = 8;

    auto *range_finder =
        new HCSR04RangeFinder(trigger_pin, echo_pin, read_delay_ms,
                              measurement_interval_ms, measurement_count, "");
    range_finder->connect_to(new LambdaConsumer<float>([](float distance) {
      debugD("Got a computed distance of %f m", distance);
    }));
  }

  sensesp_app->enable();
});