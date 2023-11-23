inline void get_ultrasonic_sensor@@_value(void)
{
    get_ultrasonic_sensor_value(current_ultrasonic@@_mode, ultrasonic@@, 
                                &spike_status_val.ultrasonic_@_sensor,
                                &spike_status_val.ultrasonic_@_mode_id);
}