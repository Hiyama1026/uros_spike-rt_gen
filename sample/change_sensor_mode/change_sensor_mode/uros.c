#include <kernel.h>
#include <t_syslog.h>
#include <micro_ros_asp.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>

#include <sensor_msgs/msg/range.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

/*spike hub&pup lib*/
#include <spike/hub/display.h>
#include "spike/pup/motor.h"
#include "spike/pup/colorsensor.h"
#include "spike/pup/ultrasonicsensor.h"
#include "spike/hub/button.h"
#include "spike/hub/speaker.h"
#include <spike/hub/light.h>
#include "spike/pup/forcesensor.h"
#include "pbio/color.h"
#include <spike/hub/imu.h>
#include <spike/hub/battery.h>
#include <pbdrv/battery.h>

#include <stdio.h>
#include <time.h>
#include "uros.h"
#include "lib.h"

#include <std_msgs/msg/int8.h>		//int8
#include <std_msgs/msg/int16.h>		//int16
#include <std_msgs/msg/bool.h>		//bool

#include <change_sensor_mode_msg/msg/spike_prime_message.h>



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


rcl_publisher_t spike_status_publisher;
rcl_publisher_t button_status_publisher;


rcl_subscription_t colorA_mode_subscriber;
rcl_subscription_t ultrasonicB_mode_subscriber;


std_msgs__msg__Int8 colorA_sensor_mode;
std_msgs__msg__Int8 ultrasonicB_sensor_mode;
change_sensor_mode_msg__msg__SpikePrimeMessage spike_status_val;
std_msgs__msg__Int8 hub_button_val;


static pup_device_t *colorA;
static pup_device_t *ultrasonicB;


int8_t current_colorA_mode;
int8_t current_ultrasonicB_mode;

int sp_ros_count;

int button_state;
int touch_sensor_state;
int pre_button_state;
int pre_touch_sensor_state;
int8_t but_tim_count;


SYSTIM *p_systim;


void error_loop(rcl_ret_t temp_rc){
    syslog(LOG_NOTICE, "error_loop %d\n", temp_rc);
    
    hub_display_image(img_sad);             //ディスプレイ表示
    hub_light_on_color(PBIO_COLOR_RED);     //ステータスライトを赤く点灯する
    
    while(1){
    	dly_tsk(100);
    }
}

inline void get_color_sensorA_value(void)
{
    spike_status_val.color_a_mode_id = get_color_sensor_value(current_colorA_mode, colorA,
                                                              &spike_status_val.color_a_sensor_value_1, 
                                                              &spike_status_val.color_a_sensor_value_2, 
                                                              &spike_status_val.color_a_sensor_value_3);
}


inline void get_ultrasonic_sensorB_value(void)
{
    get_ultrasonic_sensor_value(current_ultrasonicB_mode, ultrasonicB, 
                                &spike_status_val.ultrasonic_b_sensor,
                                &spike_status_val.ultrasonic_b_mode_id);
}




/*tier callback*/
void 
timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        if (sp_ros_count == SP_ROS_STA_CYC){
            /*color*/
            get_color_sensorA_value();


            /*ultra sonic*/
            get_ultrasonic_sensorB_value();


            /*motor*/

        
            /*force*/


            /*imu*/

            /*publish spike_status_val*/
            RCSOFTCHECK(rcl_publish(&spike_status_publisher, &spike_status_val, NULL));

            sp_ros_count = 0;
        }
        else    sp_ros_count++;

        /*button*/
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

        /*power status*/

        /*speaker*/

	}
}

/*motor subscriber callback*/


/*color-sensor subscriber callback*/

void colorA_mode_callback(const void * msgin)
{
    const std_msgs__msg__Int8 * colorA_sensor_mode = (const std_msgs__msg__Int8 *)msgin;
    
    current_colorA_mode = colorA_sensor_mode->data;
}


/*ultrasonic-sensor subscriber callback*/

void ultrasonicB_mode_callback(const void * msgin)
{
    const std_msgs__msg__Int8 * ultrasonicB_sensor_mode = (const std_msgs__msg__Int8 *)msgin;    
    current_ultrasonicB_mode = ultrasonicB_sensor_mode->data;
}


/*speaker subscriber callback*/

/*imu callback*/

/*
 *  メインタスク
 */
void 
uros_task(intptr_t exinf)
{
    syslog(LOG_NOTICE, "miro-ROS main task : start");

    /*get device pointers*/
    colorA = pup_color_sensor_get_device(PBIO_PORT_ID_A);
    ultrasonicB = pup_ultrasonic_sensor_get_device(PBIO_PORT_ID_B);


    // Set transports
    set_microros_transports(UROS_PORTID);
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "spike_state_node", "", &support));
    
    // Create publisher
    RCCHECK(rclc_publisher_init_best_effort(&spike_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(change_sensor_mode_msg, msg, SpikePrimeMessage), "spike_status"));
    RCCHECK(rclc_publisher_init_best_effort(&button_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "spike_button_status"));

    
    // Create subscriber
    RCCHECK(rclc_subscription_init_default(&colorA_mode_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "color_sensorA_mode"));
    RCCHECK(rclc_subscription_init_default(&ultrasonicB_mode_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "ultrasonic_sensorB_mode"));

    
    // Create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1), timer_callback));
    
    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    RCCHECK(rclc_executor_add_subscription(&executor, &colorA_mode_subscriber, &colorA_sensor_mode, &colorA_mode_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &ultrasonicB_mode_subscriber, &ultrasonicB_sensor_mode, &ultrasonicB_mode_callback, ON_NEW_DATA));

    
    syslog(LOG_NOTICE, "miro-ROS main task : init done.");
    
    pup_color_sensor_detectable_colors(7, raspike_rt_detectable_color);

    
    
    /*set up motor*/

    
    
    #if DISABLE_OPENING
    hub_display_text_scroll(" SPIKE-RT ", 60);
    #endif
    
    hub_display_off();
    hub_display_orientation(PBIO_SIDE_TOP);
    hub_display_image(img_smile);         //ディスプレイ表示
    hub_light_off();
    
    syslog(LOG_NOTICE, "SPIKE init done.");
    
    while(1){
    	rclc_executor_spin(&executor);
    }
    
    /*
     * [Note]
     * 本来はここでリソース解放を行う．
     * 今回は実行を中断するまでプログラムが終了しないため
     * 解放の記述は生成しない．
    */
}
 