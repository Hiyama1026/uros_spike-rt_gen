#include <lib.h>

#if USE_ULTRASONIC
void get_ultrasonic_sensor_value(int8_t ultrasonic_mode, pup_device_t *u_port, int16_t *ult_val, int8_t *u_send_mode){
	switch (ultrasonic_mode){
            case 1:
                *ult_val = pup_ultrasonic_sensor_distance(u_port);
                if(*ult_val < 0)   *ult_val = -1;
                *u_send_mode = 1;
                break;
            case 2:
                if(pup_ultrasonic_sensor_presence(u_port))      *ult_val = 1;
                else                                            *ult_val = 0;
                *u_send_mode = 2;
                break;
            default:
                *u_send_mode = 0;
        }
}
#endif

#if USE_COLOR
void get_color_code(pup_device_t *u_port, int16_t *c_val_1){
    pup_color_hsv_t tmp_color_val;

    tmp_color_val = pup_color_sensor_color(u_port, true);

    switch(tmp_color_val.h){
        case 0:            
            if(tmp_color_val.s == 100){             //RED (PBIO_COLOR_HUE_RED=0)
                *c_val_1 = 1;
            }
            else if(tmp_color_val.v == 100){        //WHITE
                *c_val_1 = 5;
            }
            else if(tmp_color_val.v == 10){         //BRACK
                *c_val_1 = 6;
            }
            else{
                *c_val_1 = 0;                       //NONE
            }
            break;
        case PBIO_COLOR_HUE_YELLOW:
            *c_val_1 = 2;
            break;
        case PBIO_COLOR_HUE_GREEN:
            *c_val_1 = 3;
            break;
        case PBIO_COLOR_HUE_BLUE:
            *c_val_1 = 4;
            break;
        default:
            *c_val_1 = -2;                //err
    }

    return;
}

int8_t get_color_sensor_value(int8_t color_mode, pup_device_t *u_port, int16_t *c_val_1, int16_t *c_val_2, int16_t *c_val_3){
    int8_t current_col_mode;

	switch (color_mode){
		pup_color_rgb_t color_rgb;
    	pup_color_hsv_t color_hsv;
        case 1:
            *c_val_1 = pup_color_sensor_ambient(u_port);
            *c_val_2 = 0;
            *c_val_3 = 0;
            current_col_mode = 1;
            break;
        case 2:
            get_color_code(u_port, c_val_1);
            *c_val_2 = 0;
            *c_val_3 = 0;
            current_col_mode = 2;
            break;
        case 3:
            *c_val_1 = pup_color_sensor_reflection(u_port);
            *c_val_2 = 0;
            *c_val_3 = 0;
            current_col_mode = 3;
            break;
        case 4:
            color_rgb = pup_color_sensor_rgb(u_port);
            color_hsv = pup_color_sensor_hsv(u_port, true);
            if(color_hsv.h==0 && color_hsv.s==0 && color_hsv.v==0)
            {
            }
            else{
                *c_val_1 = (int16_t)(color_rgb.r / 4);  
                *c_val_2 = (int16_t)(color_rgb.g / 4);          
                *c_val_3 = (int16_t)(color_rgb.b / 4);
                current_col_mode = 4;
            }                
            break;
        case 5:
            color_hsv = pup_color_sensor_hsv(u_port, true);
            if(color_hsv.h==0 && color_hsv.s==0 && color_hsv.v==0)
            {
            }
            else{
                *c_val_1 = (int16_t)(color_hsv.h / 4);  
                *c_val_2 = (int16_t)(color_hsv.s / 4);          
                *c_val_3 = (int16_t)(color_hsv.v / 4);
                current_col_mode = 5;
            }                
            break;
        default:
            current_col_mode = 0;
            break;        
    }
    return current_col_mode;

}
#endif

#if USE_FORCE
void get_force_value(pup_device_t *use_port, bool *prassed_val, int8_t *force_val)
{
    *prassed_val = pup_force_sensor_touched(use_port);

    if (*prassed_val)   *force_val = pup_force_sensor_force(use_port);
    else                *force_val = 0;
    
}
#endif

#if USE_BUTTON
inline hub_button_t hub_buttons_pressed(hub_button_t button_candidates)
{
  hub_button_t pressed;
  hub_button_is_pressed(&pressed);
  return pressed & button_candidates;
}

int wait_for_hub_buttons(hub_button_t button_candidates)
{
    hub_button_t pressed_button;
    int button_command = 0;

    pressed_button = hub_buttons_pressed(button_candidates);

    if(pressed_button & HUB_BUTTON_LEFT)    button_command += 1;
    if(pressed_button & HUB_BUTTON_RIGHT)   button_command += 2;
    if(pressed_button & HUB_BUTTON_CENTER)  button_command += 4;
    if(pressed_button & HUB_BUTTON_BT)      button_command += 8;

    return button_command;
}
#endif