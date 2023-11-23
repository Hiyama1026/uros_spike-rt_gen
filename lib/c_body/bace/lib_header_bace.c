#include <kernel.h>
#include <t_syslog.h>
#include <stdio.h>

#include "spike/pup/colorsensor.h"
#include "spike/pup/ultrasonicsensor.h"
#include "spike/pup/forcesensor.h"
#include "spike/hub/button.h"

/*enable device mcro*/
#define USE_COLOR @USE_COLOR@
#define USE_ULTRASONIC @USE_ULTRASONIC@
#define USE_FORCE @USE_FORCE@
#define USE_BUTTON @USE_BUTTON@

/*prototype definition*/
#if USE_COLOR
static pup_color_hsv_t raspike_rt_detectable_color[] = {    //detectable color
    { PBIO_COLOR_HUE_RED, 0b01100100, 0b01100100 },
    { PBIO_COLOR_HUE_YELLOW, 0b01100100, 0b01100100 },
    { PBIO_COLOR_HUE_GREEN, 0b01100100, 0b01100100 },
    { PBIO_COLOR_HUE_BLUE, 0b01100100, 0b01100100 }, 
    {0, 0, 0b01100100},     //WHITE
    {0, 0, 0b1010},         //BRACK
    {0, 0, 0} ,             //NON
};

extern void get_color_code(pup_device_t *u_port, int16_t *c_val_1);
extern int8_t get_color_sensor_value(int8_t color_mode, pup_device_t *u_port, int16_t *c_val_1, int16_t *c_val_2, int16_t *c_val_3);
#endif

#if USE_ULTRASONIC
extern void get_ultrasonic_sensor_value(int8_t ultrasonic_mode, pup_device_t *u_port, int16_t *ult_val, int8_t *u_send_mode);
#endif

#if USE_FORCE
extern void get_force_value(pup_device_t *use_port, bool *prassed_val, int8_t *force_val);
#endif

#if USE_BUTTON
extern int wait_for_hub_buttons(hub_button_t button_candidates);
extern inline hub_button_t hub_buttons_pressed(hub_button_t button_candidates);
#endif