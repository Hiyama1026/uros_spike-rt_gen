<G
            hub_imu_get_angular_velocity(&hub_angular_velocity[0]);
            spike_status_val.x_angular_velocity = hub_angular_velocity[0];
            spike_status_val.y_angular_velocity = hub_angular_velocity[1];
            spike_status_val.z_angular_velocity = hub_angular_velocity[2];
<G
<T
            spike_status_val.temperature = hub_imu_get_temperature();
<T