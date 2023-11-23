        if (pow_tim_count == 10000){     // 10s
            power_val.voltage = hub_battery_get_voltage();
            power_val.current = hub_battery_get_current();
            RCSOFTCHECK(rcl_publish(&power_status_publisher, &power_val, NULL));
            pow_tim_count = 0;
        }
        else    pow_tim_count++;