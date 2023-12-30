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

#include <speaker_sample_msg/msg/speaker_message.h>



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}




rcl_subscription_t speaker_subscriber;


speaker_sample_msg__msg__SpeakerMessage speaker_val;





static timer_count = 0;
bool speaker_enabled = false;
int16_t speaker_play_duration = 0;
uint16_t speaker_cnt = 0;

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
            /*color*/


            /*ultra sonic*/


            /*motor*/

        
            /*force*/


            /*imu*/

            /*publish spike_status_val*/
            
        /*button*/

        /*power status*/

        /*speaker*/
        if (speaker_enabled){
            if (speaker_play_duration == speaker_cnt) {
                hub_speaker_stop();
                speaker_enabled = false;
            }
            else {
                speaker_cnt++;
            }
        }

	}
}

/*motor subscriber callback*/


/*color-sensor subscriber callback*/


/*ultrasonic-sensor subscriber callback*/


/*speaker subscriber callback*/
void speaker_callback(const void * msgin)
{
    const speaker_sample_msg__msg__SpeakerMessage * speaker_val = (const speaker_sample_msg__msg__SpeakerMessage *)msgin;
    
    hub_speaker_play_tone(speaker_val->tone, SOUND_MANUAL_STOP);
    speaker_play_duration = speaker_val->duration;
    speaker_enabled = true;
    speaker_cnt = 0;
}

/*imu callback*/

/*
 *  メインタスク
 */
void 
uros_task(intptr_t exinf)
{
    syslog(LOG_NOTICE, "miro-ROS main task : start");

    /*get device pointers*/


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

    
    // Create subscriber
    RCCHECK(rclc_subscription_init_default(&speaker_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(speaker_sample_msg, msg, SpeakerMessage), "speaker_tone"));

    
    // Create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1), timer_callback));
    
    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    RCCHECK(rclc_executor_add_subscription(&executor, &speaker_subscriber, &speaker_val, &speaker_callback, ON_NEW_DATA));

    
    syslog(LOG_NOTICE, "miro-ROS main task : init done.");
    
    
    
    /*set up motor*/

    
    
    hub_speaker_set_volume(40);
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
 