void motor@@_speed_callback(const void * msgin)     // set speed or power
{
    const std_msgs__msg__Int16 * motor@@_speed_val = (const std_msgs__msg__Int16 *)msgin;
<S
    pup_motor_set_speed(motor@@, motor@@_speed_val->data);
<S
<P
    pup_motor_set_power(motor@@, motor@@_speed_val->data);
<P
}

void motor@@_stop_callback(const void * msgin)
{
    const @pkg_name@__msg__MotorStopReset * motor@@_stop_reset_val = (const @pkg_name@__msg__MotorStopReset *)msgin;

    if (motor@@_stop_reset_val->motor_reset)    pup_motor_reset_count(motor@@);

    if (motor@@_stop_reset_val->motor_stop_hold == 1)       pup_motor_stop(motor@@);
    else if (motor@@_stop_reset_val->motor_stop_hold == 2)  pup_motor_hold(motor@@);
}