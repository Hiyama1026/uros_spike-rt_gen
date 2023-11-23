inline void get_color_sensor@@_value(void)
{
    spike_status_val.color_@_mode_id = get_color_sensor_value(current_color@@_mode, color@@,
                                                              &spike_status_val.color_@_sensor_value_1, 
                                                              &spike_status_val.color_@_sensor_value_2, 
                                                              &spike_status_val.color_@_sensor_value_3);
}