#pragma once

class PID {

  float d_t;
  float maximum_regulator_value;
  float minimum_regulator_value;
  float Kp;
  float Ki;
  float Kd;

  float current_error;
  float last_error;
  float error_integral_value;
  float error_derivative_value;

  float maximum_integral_value;
  float minimum_integral_value;

  float p_term;
  float i_term;
  float d_term;

public:
  PID(float delta_t, float max_out_value, float min_out_value, float p_gain,
      float i_gain, float d_gain)
      : d_t{delta_t}, maximum_regulator_value{max_out_value},
        minimum_regulator_value{min_out_value}, Kp{p_gain}, Ki{i_gain},
        Kd{d_gain} {
    maximum_integral_value = max_out_value / Ki;
    minimum_integral_value = min_out_value / Ki;
  };

  void reset(void) {
    current_error = 0.0f;
    error_integral_value = 0.0f;
    last_error = 0.0f;
    p_term = 0.0f;
    i_term = 0.0f;
    d_term = 0.0f;
  }

  float calculate(float actual, float set) {

    auto calculated_controler_value = 0.0f;
    current_error = set - actual;
    error_integral_value += (current_error * d_t);

    if (error_integral_value > maximum_integral_value) {
      error_integral_value = maximum_integral_value;
    } else if (error_integral_value < minimum_integral_value) {
      error_integral_value = minimum_integral_value;
    }

    p_term = Kp * current_error;
    i_term = Ki * error_integral_value;
    d_term = Kd * ((actual - last_error) / d_t);
    last_error = current_error;

    calculated_controler_value = p_term + i_term + d_term;

    if (calculated_controler_value > maximum_regulator_value)
      calculated_controler_value = maximum_regulator_value;
    else if (calculated_controler_value < minimum_regulator_value) {
      calculated_controler_value = minimum_regulator_value;
    }

    return calculated_controler_value;
  }
};