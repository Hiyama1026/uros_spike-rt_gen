        if (but_tim_count == 100){      // 100 ms
            button_state = wait_for_hub_buttons(HUB_BUTTON_RIGHT|HUB_BUTTON_LEFT|HUB_BUTTON_CENTER|HUB_BUTTON_BT);
            if(pre_button_state != button_state){
                hub_button_val.data = button_state;
                RCCHECK(rcl_publish(&button_status_publisher, (const void*)&hub_button_val, NULL));
            }
            pre_button_state = button_state;

            but_tim_count = 0;
        }
        else {
            but_tim_count++;
        }