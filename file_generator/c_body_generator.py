import re
import shutil
import global_def as glb
import config_generator.config as conf

class C_body_val:
    def __init__(self):
        self.current_mode_msg_value_name = ''           # ult & col mode
        self.current_mode_subscriver_name = ''          # ult & col mode
        self.current_light_msg_value_name = ''          # ult & col light
        self.current_light_subscriver_name = ''         # ult & col light
        self.current_speed_subscriver_name = ''         # motor speed
        self.current_speed_msg_value_name = ''          # motor speed
        self.current_stop_subscriver_name = ''          # motor stop
        self.current_stop_msg_value_name = ''           # motor speed
        self.target_dir_path = 'gen/spike_rt_uros/'

cb = C_body_val()

def gen_c_body():

    with open('lib/c_body/bace/c_bace_body.c') as c_body:
        glb.c_body_string = c_body.read()

    if not conf.portA.device == None:
        definition_strings_generator('A')
    if not conf.portB.device == None:
        definition_strings_generator('B')
    if not conf.portC.device == None:
        definition_strings_generator('C')
    if not conf.portD.device == None:
        definition_strings_generator('D')
    if not conf.portE.device == None:
        definition_strings_generator('E')

    
    
    hub_configuration_generator()

    generate_common_functions()
    generate_port_depend_functions()

    # clean
    glb.c_body_string = re.sub('@.*@\n', '', glb.c_body_string)

    # generate C file
    generate_cbody()

    #print(glb.c_body_string)

    return

def generate_cbody():
    lib_uros_path = 'lib/spike_rt/spike_rt_uros'
    shutil.copytree(lib_uros_path, cb.target_dir_path)

    c_f = open(cb.target_dir_path + 'uros.c', 'w')
    c_f.write(glb.c_body_string)
    c_f.close()
    pass
    
def definition_strings_generator(port):
    device_name = ''
    device_name = device_checker(port)
    if port == 'A':
        port_obj = conf.portA
    if port == 'B':
        port_obj = conf.portB
    if port == 'C':
        port_obj = conf.portC
    if port == 'D':
        port_obj = conf.portD
    elif port == 'E':
        port_obj = conf.portE

    if device_name == 'motor':
        motor_global_def_generator(port)
        motor_port_dep_generator(port, port_obj)
    elif device_name == 'color-sensor':
        glb.color_sensor_exist = True
        color_global_def_generator(port, port_obj)
        color_port_dep_generator(port, port_obj)
    elif device_name == 'ultrasonic-sensor':
        glb.ultrasonic_sensor_exist = True
        ultrasonic_global_def_generator(port, port_obj)
        ultra_port_dep_generator(port, port_obj)
    
    elif device_name == 'force-sensor':
        glb.force_sensor_exist = True
        force_global_def_generator(port)
        force_port_dep_generator(port, port_obj)
    
    return

def generate_port_depend_functions():

    # motor
    glb.c_body_string = re.sub('@motor_func@', glb.motor_timer_callback_func, glb.c_body_string)
    glb.c_body_string = re.sub('@motor_speed_callback@', glb.motor_callback_func, glb.c_body_string)
    glb.c_body_string = re.sub('@motor_setup@', glb.motor_setup, glb.c_body_string)

    #color_sensor
    glb.c_body_string = re.sub('@color_port_depend@', glb.color_port_dep_func, glb.c_body_string)
    glb.c_body_string = re.sub('@color_func_call@', glb.color_timer_callback_func, glb.c_body_string)
    glb.c_body_string = re.sub('@color_callback@', glb.color_callback_func, glb.c_body_string)

    #ultrasonic_sensor
    glb.c_body_string = re.sub('@ultrasonic_port_depend@', glb.ultra_port_dep_func, glb.c_body_string)
    glb.c_body_string = re.sub('@ultra_func_call@', glb.ultra_timer_callback_func, glb.c_body_string)
    glb.c_body_string = re.sub('@ultrasonic_callback@', glb.ultra_callback_func, glb.c_body_string)

    #force_sensor
    glb.c_body_string = re.sub('@force_port_depend@', glb.force_port_dep_func, glb.c_body_string)
    glb.c_body_string = re.sub('@force_func_call@', glb.force_timer_callback_func, glb.c_body_string)

    glb.c_body_string = re.sub('@custom_message_include@', glb.include_msgs, glb.c_body_string)
    glb.c_body_string = re.sub('@publisher_def@', glb.publisher_definitions, glb.c_body_string)
    glb.c_body_string = re.sub('@subscriber_def@', glb.subscliber_definitions, glb.c_body_string)
    glb.c_body_string = re.sub('@message_obj_def@', glb.msg_obj_definitions, glb.c_body_string)
    glb.c_body_string = re.sub('@device_obj_def@', glb.device_pointer_def, glb.c_body_string)
    glb.c_body_string = re.sub('@c_global_val_def@', glb.c_glb_values, glb.c_body_string)
    glb.c_body_string = re.sub('@get_device_port@', glb.get_device_pointer, glb.c_body_string)
    glb.c_body_string = re.sub('@create_publisher@', glb.publishers, glb.c_body_string)
    glb.c_body_string = re.sub('@create_subscriber@', glb.subscribers, glb.c_body_string)
    glb.c_body_string = re.sub('@add_to_executor@', glb.add_executors, glb.c_body_string)

    glb.c_body_string = re.sub('@module_num@', str((glb.nub_subscriber+1)), glb.c_body_string)


def hub_configuration_generator():
    mod_path = 'lib/c_body/'

    # spike_prime_msg
    if glb.need_spike_prime_msg == True:
        sp_ros_cb_in = '        if (sp_ros_count == SP_ROS_STA_CYC){'
        sp_ros_cb_out = '            sp_ros_count = 0;\n        }'
        sp_ros_cb_else = '        else    sp_ros_count++;'
        glb.c_glb_values += '\nint sp_ros_count;\n'

        glb.include_msgs += text_replacer('spike_prime_message.h', glb.include_format)
        glb.publisher_definitions += text_replacer('spike_status_publisher', glb.def_publisher_format)
        spike_ros_msg_name =  'spike_ros_msg__msg__SpikePrimeMessage spike_status_val;\n'
        glb.msg_obj_definitions += spike_ros_msg_name
        # pub init
        create_init_b_effort_publisher('spike_status_publisher', spike_ros_msg_name, 'spike_status')        # Note トピック名を変更
        #msg publish
        pub_func = gen_publish_func('spike_status_publisher', 'spike_status_val')
        glb.c_body_string = re.sub('@spike_status_publisher@', pub_func, glb.c_body_string)
        #timer callbacl
        glb.c_body_string = re.sub('@spike_status_in@', sp_ros_cb_in, glb.c_body_string)
        glb.c_body_string = re.sub('@spike_status_out@', sp_ros_cb_out, glb.c_body_string)
        glb.c_body_string = re.sub('@spike_status_else@', sp_ros_cb_else, glb.c_body_string)

    # imu
    if conf.hub.enable_imu == True:
        imu_sub_name = 'imu_init_subscriber'

        glb.subscliber_definitions += text_replacer(imu_sub_name, glb.def_subscliber_format)
        imu_msg_def = text_replacer('imu_init', glb.sed_bool_format)
        glb.msg_obj_definitions += imu_msg_def
        
        # timer calback
        with open(mod_path + 'imu/timer_callback.c') as itb:
            i_tim = itb.read()
        if conf.hub.imu_mode == 'temperature':
            i_tim = re.sub('<T', '', i_tim)
            i_tim = re.sub('<G(.|\s)*?<G', '', i_tim)
            i_tim = i_tim[2:]
        else:
            i_tim = re.sub('<G', '', i_tim)
            i_tim = re.sub('<T(.|\s)*?<T', '', i_tim)
            i_tim = i_tim[1:]
            i_tim = i_tim[:-1]
            glb.c_glb_values += '\nfloat hub_angular_velocity[3];\n'
        glb.c_body_string = re.sub('@imu_func@', i_tim, glb.c_body_string)
        # sub callback
        with open(mod_path + 'imu/callback.c') as icb:
            i_call = icb.read()
        glb.c_body_string = re.sub('@imu_init_callback@', i_call, glb.c_body_string)
        glb.c_body_string = re.sub('@init_imu@', '    hub_imu_init();', glb.c_body_string)

        create_init_hub_subscriber(imu_sub_name, imu_msg_def, 'imu_init')
        create_add_hub_executor(imu_sub_name, 'imu_init', 'imu_init_callback')
    # button
    if conf.hub.enable_button == True:
        button_msg_val_name = 'hub_button_val'
        glb.publisher_definitions += text_replacer('button_status_publisher', glb.def_publisher_format)
        button_msg_name = re.sub('@message_name@', button_msg_val_name, glb.sed_int8_format)
        glb.msg_obj_definitions += button_msg_name
        glb.c_glb_values += '\nint button_state;\n'
        glb.c_glb_values += 'int touch_sensor_state;\n'
        glb.c_glb_values += 'int pre_button_state;\n'
        glb.c_glb_values += 'int pre_touch_sensor_state;\n'
        glb.c_glb_values += 'int8_t but_tim_count;\n'
        #timer_callback
        with open(mod_path + 'button/timer_callback.c') as btb:
            b_tim = btb.read()
        glb.c_body_string = re.sub('@button_func@', b_tim, glb.c_body_string)
        create_init_b_effort_publisher('button_status_publisher', button_msg_name, 'spike_button_status')

    #battery
    if conf.hub.enable_battery_management == True:
        glb.battery_enable = True
        glb.c_glb_values += '\nint16_t pow_tim_count;\n'

        glb.include_msgs += text_replacer('spike_power_status_message.h', glb.include_format)
        glb.publisher_definitions += text_replacer('power_status_publisher', glb.def_publisher_format)
        battery_msg_name = 'spike_ros_msg__msg__SpikePowerStatusMessage power_val;\n'
        glb.msg_obj_definitions += battery_msg_name
        with open(mod_path + 'battery/timer_callback.c') as bat_tcb:
            bat_t = bat_tcb.read()
        glb.c_body_string = re.sub('@power_func@', bat_t, glb.c_body_string)
        create_init_b_effort_publisher('power_status_publisher', battery_msg_name, 'spike_power_status')

    # speaker
    if conf.hub.enable_speaker == True:
        glb.speaker_enable = True

        glb.c_glb_values += '\nstatic timer_count = 0;\n'
        glb.c_glb_values += 'bool speaker_enabled = false;\n'
        glb.c_glb_values += 'int16_t speaker_play_duration = 0;\n'
        glb.c_glb_values += 'uint16_t speaker_cnt = 0;'

        glb.include_msgs += text_replacer('speaker_message.h', glb.include_format)
        glb.subscliber_definitions += text_replacer('speaker_subscriber', glb.def_subscliber_format)
        glb.msg_obj_definitions += 'spike_ros_msg__msg__SpeakerMessage speaker_val;\n'
        with open(mod_path + 'speaker/timer_callback.c') as sp_tcb:
            sp_timer = sp_tcb.read()
        glb.c_body_string = re.sub('@speaker_stop@', sp_timer, glb.c_body_string)
        with open(mod_path + 'speaker/callback.c') as sp_cb:
            sp_call = sp_cb.read()
        glb.c_body_string = re.sub('@speaker_callback@', sp_call, glb.c_body_string)
        sp_volume = re.sub('@volume@', str(conf.hub.speaker_volume), '    hub_speaker_set_volume(@volume@);')
        glb.c_body_string = re.sub('@speaker_volume@', sp_volume, glb.c_body_string)

        create_init_hub_subscriber('speaker_subscriber', 'spike_ros_msg__msg__SpeakerMessage speaker_val', 'speaker_tone')
        create_add_hub_executor('speaker_subscriber', 'speaker_val', 'speaker_callback')

def generate_common_functions():
    if glb.color_sensor_exist == True:
        glb.c_body_string = re.sub('@set_detectable_color@', '    pup_color_sensor_detectable_colors(7, raspike_rt_detectable_color);\n', glb.c_body_string)

def motor_global_def_generator(port):
    # include・subscriberオブジェクト定義の生成
    motor_name = re.sub('@', port, 'motor@')        # motor_name => motorA (etc...)

    glb.include_msgs += motor_include_gen()

    gen_def_text, gen_sub_text = subscliber_definition_generator(port, 'motor@_speed_subscriber')
    glb.subscliber_definitions += gen_def_text
    cb.current_speed_subscriver_name = gen_sub_text
    gen_def_text, gen_sub_text = subscliber_definition_generator(port, 'motor@_stop_reset_subscriber')
    glb.subscliber_definitions += gen_def_text
    cb.current_stop_subscriver_name = gen_sub_text
    # メッセージ型定義の生成
    std_msg_name = re.sub('@', motor_name, '@_speed_val')
    cb.current_speed_msg_value_name = std_msg_name
    glb.msg_obj_definitions += text_replacer(std_msg_name, glb.sed_int16_format)    # std_int16
    sr_msg_type_name = text_replacer(motor_name, "@motor_name@_stop_reset_val")
    cb.current_stop_msg_value_name = sr_msg_type_name
    glb.msg_obj_definitions += text_replacer(sr_msg_type_name, glb.motor_stop_reset_type_form) #custom message
    glb.device_pointer_def += text_replacer(motor_name, glb.motor_err_format)
    glb.device_pointer_def += text_replacer(motor_name, glb.motor_ptr_format)

    return

def color_global_def_generator(port, port_obj):
    # include・subscriberオブジェクト定義の生成
    color_name = re.sub('@', port, 'color@')  
    
    if port_obj.config.enable_lights == True:       # カラーセンサーライトが有効
        glb.include_msgs += color_light_include_gen()

        gen_def_text, gen_sub_text = subscliber_definition_generator(port, 'color@_light_subscriber')
        glb.subscliber_definitions += gen_def_text
        cb.current_light_subscriver_name = gen_sub_text

        c_light_type = text_replacer(color_name, "@color_name@_light_val")
        cb.current_light_msg_value_name = c_light_type
        glb.msg_obj_definitions += text_replacer(c_light_type, glb.color_light_type_form)

    gen_def_text, gen_sub_text = subscliber_definition_generator(port, 'color@_mode_subscriber')
    glb.subscliber_definitions += gen_def_text
    cb.current_mode_subscriver_name = gen_sub_text

    c_mode_type = text_replacer(color_name, "@color_name@_sensor_mode")
    cb.current_mode_msg_value_name = c_mode_type
    glb.msg_obj_definitions += text_replacer(c_mode_type, glb.sed_int8_format)

    glb.device_pointer_def += text_replacer(color_name, glb.sensor_ptr_format)
    glb.c_glb_values += text_replacer(color_name, "int8_t current_@c_name@_mode;\n")

    return

def ultrasonic_global_def_generator(port, port_obj):
    # include・subscriberオブジェクト定義の生成         
    ultrasonic_name = re.sub('@', port, 'ultrasonic@') 

    if port_obj.config.enable_lights == True:           # 距離センサーライトが有効
        glb.include_msgs += ultrasonic_light_include_gen()

        gen_def_text, gen_sub_text = subscliber_definition_generator(port, 'ultrasonic@_light_subscriber')
        glb.subscliber_definitions +=gen_def_text
        cb.current_light_subscriver_name = gen_sub_text

        u_light_type = text_replacer(ultrasonic_name, "@ultrasonic_name@_light_val")        # Note: テストプログラムから名前を変更している
        cb.current_light_msg_value_name = u_light_type
        glb.msg_obj_definitions += text_replacer(u_light_type, glb.ultra_light_type_form)

    gen_def_text, gen_sub_text = subscliber_definition_generator(port, 'ultrasonic@_mode_subscriber')
    glb.subscliber_definitions += gen_def_text
    cb.current_mode_subscriver_name = gen_sub_text

    u_mode_type = text_replacer(ultrasonic_name, "@ultrasonic_name@_sensor_mode")
    cb.current_mode_msg_value_name = u_mode_type
    glb.msg_obj_definitions += text_replacer(u_mode_type, glb.sed_int8_format)

    glb.device_pointer_def += text_replacer(ultrasonic_name, glb.sensor_ptr_format)
    glb.c_glb_values += text_replacer(ultrasonic_name, "int8_t current_@u_name@_mode;\n")

    return


def force_global_def_generator(port):
    force_name = re.sub('@', port, 'force@') 
    glb.device_pointer_def += text_replacer(force_name, glb.sensor_ptr_format)
    return

# includeテキストを生成
def motor_include_gen():
    if glb.motor_exist == False:
        glb.motor_exist = True

        incluse_motor_tex = text_replacer('motor_stop_reset.h', glb.include_format)
        return incluse_motor_tex

    return ""

# includeテキストを生成
def color_light_include_gen():
    if glb.color_light_exist == False:
        glb.color_light_exist = True

        include_color_light_tex = text_replacer('color_light_message.h', glb.include_format)
        return include_color_light_tex

    return  ""
        
# includeテキストを生成
def ultrasonic_light_include_gen():
    if glb.ultrasonic_light_exist == False:
        glb.ultrasonic_light_exist = True
        
        include_ultrasonic_light_tex = text_replacer('ultrasonic_light_message.h', glb.include_format)
        return include_ultrasonic_light_tex
    
    return ""


def subscliber_definition_generator(obj_name, sub_name_format):
    sub_name = re.sub('@', obj_name, sub_name_format)
    cb.current_subscriver_name = sub_name
    def_text = text_replacer(sub_name, glb.def_subscliber_format)

    return def_text, sub_name



def motor_port_dep_generator(port, port_obj):
    p_path = 'lib/c_body/motor/'

    # タイマーコールバック関数内の関数呼び出しを生成
    glb.motor_timer_callback_func += gen_dev_func_in_timer_callback(port, port_obj)

    # コールバック関数の生成
    with open(p_path + 'callback.c') as p_dep:
        m_call_back = p_dep.read()
    
    if port_obj.config.run_mode == 'set-power':
        m_call_back = re.sub('<P', '', m_call_back)
        m_call_back = re.sub('<S(.|\s)*?<S', '', m_call_back)
    else:
        m_call_back = re.sub('<S', '', m_call_back)
        m_call_back = re.sub('<P(.|\s)*?<P', '', m_call_back)
    glb.motor_callback_func += ('\n' + re.sub('@@', port, m_call_back) + '\n')


    # デバイスポインタの取得
    glb.get_device_pointer += re.sub('@@', port, glb.motor_get_dev_format)

    # サブスクライバ生成コードの生成
    create_init_dev_subscriber(port_obj, cb.current_speed_subscriver_name, glb.sed_int16_format, glb.motor_speed_topic)
    create_init_dev_subscriber(port_obj, cb.current_stop_subscriver_name, glb.motor_stop_reset_type_form, glb.motor_stop_topic)

    # add executor 生成
    create_add_dev_executor(port_obj, cb.current_speed_subscriver_name, cb.current_speed_msg_value_name, glb.motor_speed_callback_form)
    create_add_dev_executor(port_obj, cb.current_stop_subscriver_name, cb.current_stop_msg_value_name, glb.motor_stop_callback_form)

    # セットアップ処理の生成
    with open(p_path + 'setup.c') as setup:
        m_setup = setup.read()
    
    m_setup = re.sub('@@', port, m_setup)
    if port_obj.config.wise == 'clock':
        glb.motor_setup += (re.sub('@wise@', 'PUP_DIRECTION_CLOCKWISE', m_setup) + '\n')
    else:
        glb.motor_setup += (re.sub('@wise@', 'PUP_DIRECTION_COUNTERCLOCKWISE', m_setup) + '\n')
    
    return

def color_port_dep_generator(port, port_obj):
    p_path = 'lib/c_body/color_sensor/'

    # タイマーコールバック関数内の関数呼び出しを生成
    glb.color_timer_callback_func +=gen_dev_func_in_timer_callback(port, port_obj)

    # 共通関数の呼び出し関数の生成
    with open(p_path + 'call_common_func.c') as p_dep:
        c_dep_func = p_dep.read()
    c_dep_func = re.sub('@@', port, c_dep_func)
    glb.color_port_dep_func += (lowercase_replacer(port, c_dep_func) + '\n')

    # コールバック関数・subscriber init・add_executorの生成
    with open(p_path + 'callback.c') as p_call:
        c_call_bk_func = p_call.read()
    c_call_bk_func = re.sub('@@', port, c_call_bk_func)
    # mode subscriber init
    create_init_dev_subscriber(port_obj, cb.current_mode_subscriver_name, glb.sed_int8_format, glb.color_mode_topic)
    # add_executorの生成
    create_add_dev_executor(port_obj, cb.current_mode_subscriver_name, cb.current_mode_msg_value_name, glb.color_mode_callback_form)

    if port_obj.config.enable_lights == True:
        glb.color_callback_func += ('\n' + re.sub('<L', '', c_call_bk_func))
        create_init_dev_subscriber(port_obj, cb.current_light_subscriver_name, glb.color_light_type_form, glb.color_light_topic)
        create_add_dev_executor(port_obj, cb.current_light_subscriver_name, cb.current_light_msg_value_name, glb.color_light_callback_form)
    else:
        glb.color_callback_func += ('\n' + re.sub('<L(.|\s)*?<L', '', c_call_bk_func))
    
    # デバイスポインタ生成の生成
    glb.get_device_pointer += re.sub('@@', port, glb.color_get_dev_format)

    return


def ultra_port_dep_generator(port, port_obj):
    p_path = 'lib/c_body/ultrasonic_sensor/'

    # タイマーコールバック関数内の関数呼び出しを生成
    glb.ultra_timer_callback_func +=gen_dev_func_in_timer_callback(port, port_obj)

    # 共通関数の呼び出し関数の生成
    with open(p_path + 'call_common_func.c') as p_dep:
        u_dep_func = p_dep.read()
    u_dep_func = re.sub('@@', port, u_dep_func)
    glb.ultra_port_dep_func += (lowercase_replacer(port, u_dep_func) + '\n')

    # コールバック関数・subscriber init・add_executorの生成
    with open(p_path + 'callback.c') as p_call:
        u_call_bk_func = p_call.read()
    u_call_bk_func = re.sub('@@', port, u_call_bk_func)
    u_call_bk_func = lowercase_replacer(port, u_call_bk_func)
    # mode subscriber init
    create_init_dev_subscriber(port_obj, cb.current_mode_subscriver_name, glb.sed_int8_format, glb.ultra_mode_topic)
    # add_executorの生成
    create_add_dev_executor(port_obj, cb.current_mode_subscriver_name, cb.current_mode_msg_value_name, glb.ultra_mode_callback_form)

    if port_obj.config.enable_lights == True:                           # カラーライト有効
        glb.ultra_callback_func += ('\n' + re.sub('<L', '', u_call_bk_func))
        create_init_dev_subscriber(port_obj, cb.current_light_subscriver_name, glb.ultra_light_type_form, glb.ultra_light_topic)
        create_add_dev_executor(port_obj, cb.current_light_subscriver_name, cb.current_light_msg_value_name, glb.ultra_light_callback_form)
    else:
        glb.ultra_callback_func += ('\n' + re.sub('<L(.|\s)*?<L', '', u_call_bk_func))

    # デバイスポインタ生成の生成
    glb.get_device_pointer += re.sub('@@', port, glb.ultra_get_dev_format)

    return
    
def force_port_dep_generator(port, port_obj):
    p_path = 'lib/c_body/force_sensor/'

    # タイマーコールバック関数内の関数呼び出しを生成
    glb.force_timer_callback_func += gen_dev_func_in_timer_callback(port, port_obj)

    # 共通関数の呼び出し関数の生成
    with open(p_path + 'call_common_func.c') as p_dep:
        f_dep_func = p_dep.read()
    f_dep_func = re.sub('@@', port, f_dep_func)
    glb.force_port_dep_func += (lowercase_replacer(port, f_dep_func) + '\n')

    # デバイスポインタ生成の生成
    glb.get_device_pointer += re.sub('@@', port, glb.force_get_dev_format)

    return


def gen_dev_func_in_timer_callback(port, port_obj):
    gen_text = ''

    if port_obj.device == 'motor':
        gen_text = re.sub('@@', port, glb.motor_timer_func_form)
        gen_text = lowercase_replacer(port, gen_text)
    elif port_obj.device == 'color-sensor':
        gen_text = re.sub('@@', port, glb.color_timer_func_form)
    elif port_obj.device == 'ultrasonic-sensor':
        gen_text = re.sub('@@', port, glb.ultra_timer_func_form)
    elif port_obj.device == 'force-sensor':
        gen_text = re.sub('@@', port, glb.force_timer__func_form)
    return gen_text

def create_init_dev_subscriber(port_obj, sub_name, msg_type_form, topic_name_form):
    gen_text = ''
    parsed_msg_type = msg_type_form.split('__')

    topic_name_form = re.sub('@', port_obj.config.port, topic_name_form)

    if port_obj.config.qos == 'reliable':
        gen_text = re.sub('@sub_name@', sub_name, glb.gen_reliable_subscriber_format)
        gen_text = re.sub('@msg_arg1@', parsed_msg_type[0], gen_text)
        sep_arg_2 = parsed_msg_type[2].split(' ')
        gen_text = re.sub('@msg_arg2@', sep_arg_2[0], gen_text)
        glb.subscribers += re.sub('@topic_name@', topic_name_form, gen_text)
    else:
        gen_text = re.sub('@sub_name@', sub_name, glb.gen_b_effort_subscriber_format)
        gen_text = re.sub('@msg_arg1@', parsed_msg_type[0], gen_text)
        sep_arg_2 = parsed_msg_type[2].split(' ')
        gen_text = re.sub('@msg_arg2@', sep_arg_2[0], gen_text)
        glb.subscribers += re.sub('@topic_name@', topic_name_form, gen_text)

    return

def create_init_b_effort_publisher(pub_name, msg_type_form, topic_name):
    gen_text = ''
    parsed_msg_type = msg_type_form.split('__')

    gen_text = re.sub('@publlisher_name@', pub_name, glb.gen_best_effort_subscriber_format)
    gen_text = re.sub('@msg_arg1@', parsed_msg_type[0], gen_text)
    sep_arg_2 = parsed_msg_type[2].split(' ')
    gen_text = re.sub('@msg_arg2@', sep_arg_2[0], gen_text)
    glb.publishers += re.sub('@topic_name@', topic_name, gen_text)
    return

def create_init_hub_subscriber(sub_name, msg_type_form, topic_name):
    gen_text = ''
    parsed_msg_type = msg_type_form.split('__')

    gen_text = re.sub('@sub_name@', sub_name, glb.gen_reliable_subscriber_format)
    gen_text = re.sub('@msg_arg1@', parsed_msg_type[0], gen_text)
    sep_arg_2 = parsed_msg_type[2].split(' ')
    gen_text = re.sub('@msg_arg2@', sep_arg_2[0], gen_text)
    glb.subscribers += re.sub('@topic_name@', topic_name, gen_text)

    return

def gen_publish_func(pub_name, msg_type_name):
    gen_text = ''

    gen_text = re.sub('@publisher_name@', pub_name, glb.msg_publish_func)
    gen_text = re.sub('@msg_val_name@', msg_type_name, gen_text)
    
    return gen_text

def create_add_dev_executor(port_obj, sub_name, msg_val_name, call_back_form):
    call_back_name = re.sub('@', port_obj.config.port, call_back_form)

    gen_text = re.sub('@sub_name@', sub_name, glb.add_executor_form)
    gen_text = re.sub('@val_name@', msg_val_name, gen_text)
    glb.add_executors += re.sub('@call_back@', call_back_name, gen_text)
    glb.nub_subscriber += 1
    return

def create_add_hub_executor(sub_name, msg_val_name, call_back_name):
    gen_text = re.sub('@sub_name@', sub_name, glb.add_executor_form)
    gen_text = re.sub('@val_name@', msg_val_name, gen_text)
    glb.add_executors += re.sub('@call_back@', call_back_name, gen_text)
    glb.nub_subscriber += 1
    return

def lowercase_replacer(upper, text):
    lower = ''
    if upper == 'A':
        lower = 'a'
    elif upper == 'B':
        lower = 'b'
    elif upper == 'C':
        lower = 'c'
    elif upper == 'D':
        lower = 'd'
    elif upper == 'E':
        lower = 'e'
    else:
        return text
    
    text = re.sub('@', lower, text)
    return text

def device_checker(port):
    dev_name = ''

    if port == 'A':
        if conf.portA.device == 'motor':
            dev_name = 'motor'
        elif conf.portA.device == 'color-sensor':
            dev_name = 'color-sensor'
        elif conf.portA.device == 'ultrasonic-sensor':
            dev_name = 'ultrasonic-sensor'
        elif conf.portA.device == 'force-sensor':
            dev_name = 'force-sensor'
    
    elif port == 'B':
        if conf.portB.device == 'motor':
            dev_name = 'motor'
        elif conf.portB.device == 'color-sensor':
            dev_name = 'color-sensor'
        elif conf.portB.device == 'ultrasonic-sensor':
            dev_name = 'ultrasonic-sensor'
        elif conf.portB.device == 'force-sensor':
            dev_name = 'force-sensor'
    
    elif port == 'C':
        if conf.portC.device == 'motor':
            dev_name = 'motor'
        elif conf.portC.device == 'color-sensor':
            dev_name = 'color-sensor'
        elif conf.portC.device == 'ultrasonic-sensor':
            dev_name = 'ultrasonic-sensor'
        elif conf.portC.device == 'force-sensor':
            dev_name = 'force-sensor'
        
    elif port == 'D':
        if conf.portD.device == 'motor':
            dev_name = 'motor'
        elif conf.portD.device == 'color-sensor':
            dev_name = 'color-sensor'
        elif conf.portD.device == 'ultrasonic-sensor':
            dev_name = 'ultrasonic-sensor'
        elif conf.portD.device == 'force-sensor':
            dev_name = 'force-sensor'

    elif port == 'E':
        if conf.portE.device == 'motor':
            dev_name = 'motor'
        elif conf.portE.device == 'color-sensor':
            dev_name = 'color-sensor'
        elif conf.portE.device == 'ultrasonic-sensor':
            dev_name = 'ultrasonic-sensor'
        elif conf.portE.device == 'force-sensor':
            dev_name = 'force-sensor'

    return dev_name

def single_replacer(insert_tex, ori_tex):
    gen_text = re.sub('@', insert_tex, ori_tex)

    return gen_text

# @から@までを置換する
def text_replacer(insert_tex, ori_tex):
    gen_text = re.sub('@.*@', insert_tex, ori_tex)

    return gen_text

