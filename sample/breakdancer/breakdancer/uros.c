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

#include <breakdancer_msg/msg/color_light_message.h>
#include <breakdancer_msg/msg/motor_stop_reset.h>
#include <breakdancer_msg/msg/spike_prime_message.h>



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


rcl_publisher_t spike_status_publisher;
rcl_publisher_t button_status_publisher;


rcl_subscription_t colorB_light_subscriber;
rcl_subscription_t colorB_mode_subscriber;
rcl_subscription_t motorC_speed_subscriber;
rcl_subscription_t motorC_stop_reset_subscriber;
rcl_subscription_t motorD_speed_subscriber;
rcl_subscription_t motorD_stop_reset_subscriber;


breakdancer_msg__msg__ColorLightMessage colorB_light_val;
std_msgs__msg__Int8 colorB_sensor_mode;
std_msgs__msg__Int16 motorC_speed_val;
breakdancer_msg__msg__MotorStopReset motorC_stop_reset_val;
std_msgs__msg__Int16 motorD_speed_val;
breakdancer_msg__msg__MotorStopReset motorD_stop_reset_val;
breakdancer_msg__msg__SpikePrimeMessage spike_status_val;
std_msgs__msg__Int8 hub_button_val;


static pup_device_t *colorB;
static pbio_error_t motorC_err;
static pup_motor_t *motorC;
static pbio_error_t motorD_err;
static pup_motor_t *motorD;


int8_t current_colorB_mode;

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

inline void get_color_sensorB_value(void)
{
    spike_status_val.color_b_mode_id = get_color_sensor_value(current_colorB_mode, colorB,
                                                              &spike_status_val.color_b_sensor_value_1, 
                                                              &spike_status_val.color_b_sensor_value_2, 
                                                              &spike_status_val.color_b_sensor_value_3);
}






/*tier callback*/
void 
timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        if (sp_ros_count == SP_ROS_STA_CYC){
            /*color*/
            get_color_sensorB_value();


            /*ultra sonic*/


            /*motor*/
            spike_status_val.motor_c_count = pup_motor_get_count(motorC);
            spike_status_val.motor_d_count = pup_motor_get_count(motorD);

        
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

void motorC_speed_callback(const void * msgin)     // set speed or power
{
    const std_msgs__msg__Int16 * motorC_speed_val = (const std_msgs__msg__Int16 *)msgin;

    pup_motor_set_speed(motorC, motorC_speed_val->data);


}

void motorC_stop_callback(const void * msgin)
{
    const breakdancer_msg__msg__MotorStopReset * motorC_stop_reset_val = (const breakdancer_msg__msg__MotorStopReset *)msgin;

    if (motorC_stop_reset_val->motor_reset)    pup_motor_reset_count(motorC);

    if (motorC_stop_reset_val->motor_stop_hold == 1)       pup_motor_stop(motorC);
    else if (motorC_stop_reset_val->motor_stop_hold == 2)  pup_motor_hold(motorC);
}

void motorD_speed_callback(const void * msgin)     // set speed or power
{
    const std_msgs__msg__Int16 * motorD_speed_val = (const std_msgs__msg__Int16 *)msgin;

    pup_motor_set_speed(motorD, motorD_speed_val->data);


}

void motorD_stop_callback(const void * msgin)
{
    const breakdancer_msg__msg__MotorStopReset * motorD_stop_reset_val = (const breakdancer_msg__msg__MotorStopReset *)msgin;

    if (motorD_stop_reset_val->motor_reset)    pup_motor_reset_count(motorD);

    if (motorD_stop_reset_val->motor_stop_hold == 1)       pup_motor_stop(motorD);
    else if (motorD_stop_reset_val->motor_stop_hold == 2)  pup_motor_hold(motorD);
}


/*color-sensor subscriber callback*/

void colorB_mode_callback(const void * msgin)
{
    const std_msgs__msg__Int8 * colorB_sensor_mode = (const std_msgs__msg__Int8 *)msgin;
    
    current_colorB_mode = colorB_sensor_mode->data;
}

void colorB_light_callback(const void * msgin)
{
    const breakdancer_msg__msg__ColorLightMessage * colorB_light_val = (const breakdancer_msg__msg__ColorLightMessage *)msgin;

    pup_color_sensor_light_set(colorB, colorB_light_val->light1, colorB_light_val->light2, colorB_light_val->light3);
}


/*ultrasonic-sensor subscriber callback*/


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
    colorB = pup_color_sensor_get_device(PBIO_PORT_ID_B);
    motorC = pup_motor_get_device(PBIO_PORT_ID_C);
    motorD = pup_motor_get_device(PBIO_PORT_ID_D);


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
    RCCHECK(rclc_publisher_init_best_effort(&spike_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(breakdancer_msg, msg, SpikePrimeMessage), "spike_status"));
    RCCHECK(rclc_publisher_init_best_effort(&button_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "spike_button_status"));

    
    // Create subscriber
    RCCHECK(rclc_subscription_init_best_effort(&colorB_mode_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "color_sensorB_mode"));
    RCCHECK(rclc_subscription_init_best_effort(&colorB_light_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(breakdancer_msg, msg, ColorLightMessage), "color_sensorB_light"));
    RCCHECK(rclc_subscription_init_best_effort(&motorC_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "motorC_speed"));
    RCCHECK(rclc_subscription_init_best_effort(&motorC_stop_reset_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(breakdancer_msg, msg, MotorStopReset), "motorC_stop_reset"));
    RCCHECK(rclc_subscription_init_best_effort(&motorD_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "motorD_speed"));
    RCCHECK(rclc_subscription_init_best_effort(&motorD_stop_reset_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(breakdancer_msg, msg, MotorStopReset), "motorD_stop_reset"));

    
    // Create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1), timer_callback));
    
    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    RCCHECK(rclc_executor_add_subscription(&executor, &colorB_mode_subscriber, &colorB_sensor_mode, &colorB_mode_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &colorB_light_subscriber, &colorB_light_val, &colorB_light_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorC_speed_subscriber, &motorC_speed_val, &motorC_speed_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorC_stop_reset_subscriber, &motorC_stop_reset_val, &motorC_stop_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorD_speed_subscriber, &motorD_speed_val, &motorD_speed_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorD_stop_reset_subscriber, &motorD_stop_reset_val, &motorD_stop_callback, ON_NEW_DATA));

    
    syslog(LOG_NOTICE, "miro-ROS main task : init done.");
    
    pup_color_sensor_detectable_colors(7, raspike_rt_detectable_color);

    
    
    /*set up motor*/
    for(int i = 0; i < 10; i++)
    {
        motorC_err = pup_motor_setup(motorC, PUP_DIRECTION_COUNTERCLOCKWISE, true);
        pup_motor_reset_count(motorC);
        if(motorC_err != PBIO_ERROR_AGAIN){
            break;
        }
        // Set up failed -> Wait 1s and ry one more
        dly_tsk(1000000);
    }
    for(int i = 0; i < 10; i++)
    {
        motorD_err = pup_motor_setup(motorD, PUP_DIRECTION_CLOCKWISE, true);
        pup_motor_reset_count(motorD);
        if(motorD_err != PBIO_ERROR_AGAIN){
            break;
        }
        // Set up failed -> Wait 1s and ry one more
        dly_tsk(1000000);
    }

    
    
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
 