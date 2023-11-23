void color@@_mode_callback(const void * msgin)
{
    const std_msgs__msg__Int8 * color@@_sensor_mode = (const std_msgs__msg__Int8 *)msgin;
    
    current_color@@_mode = color@@_sensor_mode->data;
}
<L
void color@@_light_callback(const void * msgin)
{
    const spike_ros_msg__msg__ColorLightMessage * color@@_light_val = (const spike_ros_msg__msg__ColorLightMessage *)msgin;

    pup_color_sensor_light_set(color@@, color@@_light_val->light1, color@@_light_val->light2, color@@_light_val->light3);
}
<L