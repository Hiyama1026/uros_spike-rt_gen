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

@custom_message_include@


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


@publisher_def@

@subscriber_def@

@message_obj_def@

@device_obj_def@

@c_global_val_def@

SYSTIM *p_systim;


void error_loop(rcl_ret_t temp_rc){
    syslog(LOG_NOTICE, "error_loop %d\n", temp_rc);
    
    hub_display_image(img_sad);             //ディスプレイ表示
    hub_light_on_color(PBIO_COLOR_RED);     //ステータスライトを赤く点灯する
    
    while(1){
    	dly_tsk(100);
    }
}

@color_port_depend@

@ultrasonic_port_depend@

@force_port_depend@

/*tier callback*/
void 
timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
@spike_status_in@
            /*color*/
@color_func_call@

            /*ultra sonic*/
@ultra_func_call@

            /*motor*/
@motor_func@
        
            /*force*/
@force_func_call@

            /*imu*/
@imu_func@

            /*publish spike_status_val*/
            @spike_status_publisher@
@spike_status_out@
@spike_status_else@

        /*button*/
@button_func@

        /*power status*/
@power_func@

        /*speaker*/
@speaker_stop@

	}
}

/*motor subscriber callback*/
@motor_speed_callback@

/*color-sensor subscriber callback*/
@color_callback@

/*ultrasonic-sensor subscriber callback*/
@ultrasonic_callback@

/*speaker subscriber callback*/
@speaker_callback@

/*imu callback*/
@imu_init_callback@

/*
 *  メインタスク
 */
void 
uros_task(intptr_t exinf)
{
    syslog(LOG_NOTICE, "miro-ROS main task : start");

    /*get device pointers*/
@get_device_port@

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
@create_publisher@
    
    // Create subscriber
@create_subscriber@
    
    // Create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1), timer_callback));
    
    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, @module_num@, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
@add_to_executor@
    
    syslog(LOG_NOTICE, "miro-ROS main task : init done.");
    
@set_detectable_color@
    
    
    /*set up motor*/
@motor_setup@
    
@init_imu@
    
@speaker_volume@
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
 