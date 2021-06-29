#include <ros/ros.h>

/* saturation function : saturates value to [min, max] */
float saturate(float value, float min, float max){
  if (value > max){
    return max;
  }
  if (value < min){
    return min;
  }
  return value;
  
}

class PID_controller{
  public:
    float _dt;
    float _Kp;
    float _Ki;
    float _Kd;

    float _out_limit;
    float _integ_limit;
    float _integral = 0.0f;
    float _prev_err = 0.0f;
    float _prev_time;
    bool _init_flag = false;

    float control(float setpoint, float curpoint, float feedforward);
    void set_parameter(float Kp, float Ki, float Kd, float out_limit, float integ_limit);

    PID_controller() {}
};

void PID_controller::set_parameter(float Kp, float Ki, float Kd, float out_limit, float integ_limit){
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _out_limit = out_limit;
  _integ_limit = integ_limit;
}

float PID_controller::control(float setpoint, float curpoint, float feedforward){
  float err;
  float Pout, Iout, Dout;
  float output, output_sat;

  if (_init_flag){
    _dt = ros::Time::now().toSec() - _prev_time;
  }
  else{
    _dt = 0.0f;
  }
  _init_flag = true;
  _prev_time = ros::Time::now().toSec();
  
  err = setpoint - curpoint;
  
  Pout = _Kp*err;
  
  _integral += err*_dt;
  _integral = saturate(_integral,-_integ_limit, _integ_limit);
  Iout = _Ki*_integral;
  
  if(_dt != 0.0f){ Dout = _Kd*(err - _prev_err)/_dt;}
  else{ Dout = 0;}
  _prev_err = err;

  output = Pout + Iout + Dout + feedforward;
  output_sat = saturate(output,-_out_limit,_out_limit);
  
  return output_sat;
}