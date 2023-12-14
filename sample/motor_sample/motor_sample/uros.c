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

#include <motor_sample_msg/msg/motor_stop_reset.h>
#include <motor_sample_msg/msg/spike_prime_message.h>



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


rcl_publisher_t spike_status_publisher;


rcl_subscription_t motorA_speed_subscriber;
rcl_subscription_t motorA_stop_reset_subscriber;
rcl_subscription_t motorD_speed_subscriber;
rcl_subscription_t motorD_stop_reset_subscriber;
rcl_subscription_t motorE_speed_subscriber;
rcl_subscription_t motorE_stop_reset_subscriber;


std_msgs__msg__Int16 motorA_speed_val;
motor_sample_msg__msg__MotorStopReset motorA_stop_reset_val;
std_msgs__msg__Int16 motorD_speed_val;
motor_sample_msg__msg__MotorStopReset motorD_stop_reset_val;
std_msgs__msg__Int16 motorE_speed_val;
motor_sample_msg__msg__MotorStopReset motorE_stop_reset_val;
motor_sample_msg__msg__SpikePrimeMessage spike_status_val;


static pbio_error_t motorA_err;
static pup_motor_t *motorA;
static pbio_error_t motorD_err;
static pup_motor_t *motorD;
static pbio_error_t motorE_err;
static pup_motor_t *motorE;



int sp_ros_count;


SYSTIM *p_systim;


void error_loop(rcl_ret_t temp_rc){
    syslog(LOG_NOTICE, "error_loop %d\n", temp_rc);
    
    hub_display_image(img_sad);             //ディスプレイ表示
    hub_light_on_color(PBIO_COLOR_RED);     //ステータスライトを赤く点灯する
    
    while(1){
    	dly_tsk(100);
    }
}







/*tier callback*/
void 
timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        if (sp_ros_count == SP_ROS_STA_CYC){
            /*color*/


            /*ultra sonic*/


            /*motor*/
            spike_status_val.motor_a_count = pup_motor_get_count(motorA);
            spike_status_val.motor_d_count = pup_motor_get_count(motorD);
            spike_status_val.motor_e_count = pup_motor_get_count(motorE);

        
            /*force*/


            /*imu*/

            /*publish spike_status_val*/
            RCSOFTCHECK(rcl_publish(&spike_status_publisher, &spike_status_val, NULL));

            sp_ros_count = 0;
        }
        else    sp_ros_count++;

        /*button*/

        /*power status*/

        /*speaker*/

	}
}

/*motor subscriber callback*/

void motorA_speed_callback(const void * msgin)     // set speed or power
{
    const std_msgs__msg__Int16 * motorA_speed_val = (const std_msgs__msg__Int16 *)msgin;


    pup_motor_set_power(motorA, motorA_speed_val->data);

}

void motorA_stop_callback(const void * msgin)
{
    const motor_sample_msg__msg__MotorStopReset * motorA_stop_reset_val = (const motor_sample_msg__msg__MotorStopReset *)msgin;

    if (motorA_stop_reset_val->motor_reset)    pup_motor_reset_count(motorA);

    if (motorA_stop_reset_val->motor_stop_hold == 1)       pup_motor_stop(motorA);
    else if (motorA_stop_reset_val->motor_stop_hold == 2)  pup_motor_hold(motorA);
}

void motorD_speed_callback(const void * msgin)     // set speed or power
{
    const std_msgs__msg__Int16 * motorD_speed_val = (const std_msgs__msg__Int16 *)msgin;

    pup_motor_set_speed(motorD, motorD_speed_val->data);


}

void motorD_stop_callback(const void * msgin)
{
    const motor_sample_msg__msg__MotorStopReset * motorD_stop_reset_val = (const motor_sample_msg__msg__MotorStopReset *)msgin;

    if (motorD_stop_reset_val->motor_reset)    pup_motor_reset_count(motorD);

    if (motorD_stop_reset_val->motor_stop_hold == 1)       pup_motor_stop(motorD);
    else if (motorD_stop_reset_val->motor_stop_hold == 2)  pup_motor_hold(motorD);
}

void motorE_speed_callback(const void * msgin)     // set speed or power
{
    const std_msgs__msg__Int16 * motorE_speed_val = (const std_msgs__msg__Int16 *)msgin;

    pup_motor_set_speed(motorE, motorE_speed_val->data);


}

void motorE_stop_callback(const void * msgin)
{
    const motor_sample_msg__msg__MotorStopReset * motorE_stop_reset_val = (const motor_sample_msg__msg__MotorStopReset *)msgin;

    if (motorE_stop_reset_val->motor_reset)    pup_motor_reset_count(motorE);

    if (motorE_stop_reset_val->motor_stop_hold == 1)       pup_motor_stop(motorE);
    else if (motorE_stop_reset_val->motor_stop_hold == 2)  pup_motor_hold(motorE);
}


/*color-sensor subscriber callback*/


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
    motorA = pup_motor_get_device(PBIO_PORT_ID_A);
    motorD = pup_motor_get_device(PBIO_PORT_ID_D);
    motorE = pup_motor_get_device(PBIO_PORT_ID_E);


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
    RCCHECK(rclc_publisher_init_best_effort(&spike_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(motor_sample_msg, msg, SpikePrimeMessage), "spike_status"));

    
    // Create subscriber
    RCCHECK(rclc_subscription_init_best_effort(&motorA_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "motorA_speed"));
    RCCHECK(rclc_subscription_init_best_effort(&motorA_stop_reset_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(motor_sample_msg, msg, MotorStopReset), "motorA_stop_reset"));
    RCCHECK(rclc_subscription_init_best_effort(&motorD_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "motorD_speed"));
    RCCHECK(rclc_subscription_init_best_effort(&motorD_stop_reset_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(motor_sample_msg, msg, MotorStopReset), "motorD_stop_reset"));
    RCCHECK(rclc_subscription_init_best_effort(&motorE_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "motorE_speed"));
    RCCHECK(rclc_subscription_init_best_effort(&motorE_stop_reset_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(motor_sample_msg, msg, MotorStopReset), "motorE_stop_reset"));

    
    // Create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1), timer_callback));
    
    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    RCCHECK(rclc_executor_add_subscription(&executor, &motorA_speed_subscriber, &motorA_speed_val, &motorA_speed_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorA_stop_reset_subscriber, &motorA_stop_reset_val, &motorA_stop_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorD_speed_subscriber, &motorD_speed_val, &motorD_speed_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorD_stop_reset_subscriber, &motorD_stop_reset_val, &motorD_stop_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorE_speed_subscriber, &motorE_speed_val, &motorE_speed_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &motorE_stop_reset_subscriber, &motorE_stop_reset_val, &motorE_stop_callback, ON_NEW_DATA));

    
    syslog(LOG_NOTICE, "miro-ROS main task : init done.");
    
    
    
    /*set up motor*/
    for(int i = 0; i < 10; i++)
    {
        motorA_err = pup_motor_setup(motorA, PUP_DIRECTION_CLOCKWISE, true);
        pup_motor_reset_count(motorA);
        if(motorA_err != PBIO_ERROR_AGAIN){
            break;
        }
        // Set up failed -> Wait 1s and ry one more
        dly_tsk(1000000);
    }
    for(int i = 0; i < 10; i++)
    {
        motorD_err = pup_motor_setup(motorD, PUP_DIRECTION_COUNTERCLOCKWISE, true);
        pup_motor_reset_count(motorD);
        if(motorD_err != PBIO_ERROR_AGAIN){
            break;
        }
        // Set up failed -> Wait 1s and ry one more
        dly_tsk(1000000);
    }
    for(int i = 0; i < 10; i++)
    {
        motorE_err = pup_motor_setup(motorE, PUP_DIRECTION_CLOCKWISE, true);
        pup_motor_reset_count(motorE);
        if(motorE_err != PBIO_ERROR_AGAIN){
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
 