package_name = 'spike_rt_uros'      # default name
msg_pkg_name = 'spike-rt_uros_msg'

motor_number = 0
color_number = 0
ultra_number = 0
force_number = 0

args = list()
need_spike_prime_msg = False
need_custom_msg = False
motor_exist = False
color_sensor_exist = False
ultrasonic_sensor_exist = False
force_sensor_exist = False
color_light_exist = False
ultrasonic_light_exist = False
speaker_enable = False
battery_enable = False

# 設定内容出力用
print_conf = ''

c_body_string = ''

# カスタムメッセージincludeフォーマット
include_format = '#include <@pkg_name@/msg/@motor_reset_h@>\n'

# pub・sub定義フォーマット
def_publisher_format = "rcl_publisher_t @publisher_name@;\n"
def_subscliber_format = "rcl_subscription_t @subscliber_name@;\n"

# publish関数
msg_publish_func = 'RCSOFTCHECK(rcl_publish(&@publisher_name@, &@msg_val_name@, NULL));\n'

# メッセージ型定義フォーマット
sed_int16_format = "std_msgs__msg__Int16 @message_name@;\n"
sed_int8_format = "std_msgs__msg__Int8 @message_name@;\n"
sed_bool_format = "std_msgs__msg__Bool @message_name@;\n"
motor_stop_reset_type_form = "@pkg_name@__msg__MotorStopReset @value_name@;\n"
color_light_type_form = "@pkg_name@__msg__ColorLightMessage @value_name@;\n"
ultra_light_type_form = "@pkg_name@__msg__UltrasonicLightMessage @ultra_name@;\n"

# エラー型定義フォーマット
motor_err_format = "static pbio_error_t @motor_name@_err;\n"
motor_ptr_format = "static pup_motor_t *@ptr_name@;\n"
sensor_ptr_format = "static pup_device_t *@sensor_name@;\n"

# サブスクライバ初期化関数フォーマット
gen_reliable_subscriber_format = '    RCCHECK(rclc_subscription_init_default(&@sub_name@, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(@msg_arg1@, msg, @msg_arg2@), "@topic_name@"));\n'
gen_b_effort_subscriber_format = '    RCCHECK(rclc_subscription_init_best_effort(&@sub_name@, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(@msg_arg1@, msg, @msg_arg2@), "@topic_name@"));\n'
add_executor_form = '    RCCHECK(rclc_executor_add_subscription(&executor, &@sub_name@, &@val_name@, &@call_back@, ON_NEW_DATA));\n'

# パブリッシャー初期化関数フォーマット
gen_best_effort_subscriber_format = '    RCCHECK(rclc_publisher_init_best_effort(&@publlisher_name@, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(@msg_arg1@, msg, @msg_arg2@), "@topic_name@"));\n'

# デバイスポインタ取得関数生成フォーマット
motor_get_dev_format = '    motor@@ = pup_motor_get_device(PBIO_PORT_ID_@@);\n'
color_get_dev_format = '    color@@ = pup_color_sensor_get_device(PBIO_PORT_ID_@@);\n'
ultra_get_dev_format = '    ultrasonic@@ = pup_ultrasonic_sensor_get_device(PBIO_PORT_ID_@@);\n'
force_get_dev_format = '    force@@ = pup_force_sensor_get_device(PBIO_PORT_ID_@@);\n'


# コールバック関数名フォーマット
motor_speed_callback_form = 'motor@_speed_callback'
motor_stop_callback_form = 'motor@_stop_callback'
color_mode_callback_form = 'color@_mode_callback'
color_light_callback_form = 'color@_light_callback'
ultra_mode_callback_form = 'ultrasonic@_mode_callback'
ultra_light_callback_form = 'ultra_light@_callback'

# トピック名フォーマット
motor_speed_topic = 'motor@_speed'
motor_stop_topic = 'motor@_stop_reset'
color_mode_topic = 'color_sensor@_mode'
color_light_topic = 'color_sensor@_light'     
ultra_mode_topic = 'ultrasonic_sensor@_mode'
ultra_light_topic = 'ultrasonic_sensor@_light'

# C global def
include_msgs = ""
publisher_definitions = ""
subscliber_definitions = ""
msg_obj_definitions = ""
device_pointer_def = ""
c_glb_values = ""

# 共通関数の呼び出し関数名
color_port_dep_func = ''
ultra_port_dep_func = ''
force_port_dep_func = ''

# モータセットアップ処理
motor_setup = ''

# サブスクライバコールバック
motor_callback_func = ''
color_callback_func = ''
ultra_callback_func = ''

# サブスクライバ数
nub_subscriber = 0

# デバイスポインタ取得関数
get_device_pointer = ''

# in timer_callbacl func
motor_timer_func_form = '            spike_status_val.motor_@_count = pup_motor_get_count(motor@@);\n'  #spike_status_val.motor_a_count ....
motor_timer_callback_func = ''
color_timer_func_form = '            get_color_sensor@@_value();\n'
color_timer_callback_func = ''
ultra_timer_func_form = '            get_ultrasonic_sensor@@_value();\n'
ultra_timer_callback_func = ''
force_timer__func_form = '            get_force@@_value();\n'
force_timer_callback_func = ''

# pub, sub, exe init
publishers = ''
subscribers = ''
add_executors = ''

'''
Message
'''

msg_camke_list = ''     # 先頭に空白2つ

spike_ros_msg_list = ''