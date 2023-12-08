void ultrasonic@@_mode_callback(const void * msgin)
{
    const std_msgs__msg__Int8 * ultrasonic@@_sensor_mode = (const std_msgs__msg__Int8 *)msgin;    
    current_ultrasonic@@_mode = ultrasonic@@_sensor_mode->data;
}
<L
void ultra_light@@_callback(const void * msgin)
{
    const @pkg_name@__msg__UltrasonicLightMessage * ultrasonic@@_light_val = (const @pkg_name@__msg__UltrasonicLightMessage *)msgin;

    pup_ultrasonic_sensor_light_set(ultrasonic@@, ultrasonic@@_light_val->light1, ultrasonic@@_light_val->light2, ultrasonic@@_light_val->light3, ultrasonic@@_light_val->light4);
}
<L