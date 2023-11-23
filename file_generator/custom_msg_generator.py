import os
import re
import shutil
import global_def as glb
import config_generator.config as conf

class Gen_msg_file:
    def __init__(self):
        self.spike_prime_msg_text = ''
        self.lib_path = 'lib/msg/spike_ros_msg/'
        self.source_path = 'lib/msg/source/'
        self.target_dir_path = 'gen/spike_ros_msg/'
        self.target_msg_dir_path = 'gen/spike_ros_msg/msg/'
        self.motor_target_path = ''
        self.c_light_target_path = ''
        self.u_light_target_path = ''
        self.speaker_target_path = ''
        self.battery_target_path = ''
    
msg_info = Gen_msg_file()

def gen_custom_msg():    
    shutil.copytree(msg_info.lib_path, msg_info.target_dir_path)
    os.mkdir(msg_info.target_msg_dir_path)

    copy_format()
    if glb.need_spike_prime_msg == True:
        gen_spike_ros_msg()
    
    # fix CMakeLists.txt
    with open(msg_info.target_dir_path + 'CMakeLists.txt')as c_make:
            CMake = c_make.read()
    c_make_contents = re.sub('@msg_file_name@', (glb.msg_camke_list[:-1]), CMake)

    CMake = open(msg_info.target_dir_path + 'CMakeLists.txt', 'w')
    CMake.write(c_make_contents)
    CMake.close()

    return

def copy_format():
    
    if glb.motor_exist == True:
        msg_info.motor_target_path = shutil.copy(msg_info.source_path + 'MotorStopReset.msg', msg_info.target_msg_dir_path)
        glb.msg_camke_list += '  \"msg/MotorStopReset.msg\"\n'
    if glb.color_light_exist == True:
        msg_info.c_light_target_path = shutil.copy(msg_info.source_path + 'ColorLightMessage.msg', msg_info.target_msg_dir_path)
        glb.msg_camke_list += '  \"msg/ColorLightMessage.msg\"\n'
    if glb.ultrasonic_light_exist == True:
        msg_info.u_light_target_path = shutil.copy(msg_info.source_path + 'UltrasonicLightMessage.msg', msg_info.target_msg_dir_path)
        glb.msg_camke_list += '  \"msg/UltrasonicLightMessage.msg\"\n'
    if glb.speaker_enable == True:
        msg_info.speaker_target_path = shutil.copy(msg_info.source_path + 'SpeakerMessage.msg', msg_info.target_msg_dir_path)
        glb.msg_camke_list += '  \"msg/SpeakerMessage.msg\"\n'
    if glb.battery_enable == True:
        msg_info.battery_target_path = shutil.copy(msg_info.source_path + 'SpikePowerStatusMessage.msg', msg_info.target_msg_dir_path)
        glb.msg_camke_list += '  \"msg/SpikePowerStatusMessage.msg\"\n'
    
    return
         
def gen_spike_ros_msg():
    glb.msg_camke_list += '  \"msg/SpikePrimeMessage.msg\"\n'
    
    if not conf.portA.device == None:
        message_name_set(conf.portA.device, 'a')
    if not conf.portB.device == None:
        message_name_set(conf.portB.device, 'b')
    if not conf.portC.device == None:
        message_name_set(conf.portC.device, 'c')
    if not conf.portD.device == None:
        message_name_set(conf.portD.device, 'd')
    if not conf.portE.device == None:
        message_name_set(conf.portE.device, 'e')

    if conf.hub.enable_imu == True:
        with open(msg_info.source_path + 'SpikePrimeMessage/Imu.msg')as i_msg:
            imu_msg = i_msg.read()

        if conf.hub.imu_mode == 'gyro':
            imu_msg = re.sub('<G', '', imu_msg)
            imu_msg = re.sub('<T(.|\s)*?<T', '', imu_msg)
            imu_msg = imu_msg[1:]
            imu_msg = imu_msg[:-2]
        else:
            imu_msg = re.sub('<T', '', imu_msg)
            imu_msg = re.sub('<G(.|\s)*?<G', '', imu_msg)
            imu_msg = imu_msg[2:]
            imu_msg = imu_msg[:-1]
        
        glb.spike_ros_msg_list += imu_msg
    
    # gen a file
    srm = open(msg_info.target_msg_dir_path + 'SpikePrimeMessage.msg', 'w')
    srm.write(glb.spike_ros_msg_list)
    srm.close()
    return


def message_name_set(device, port):

    if device == 'motor':
        with open(msg_info.source_path + 'SpikePrimeMessage/Motor.msg')as m_msg:
            motor_msg = m_msg.read()
        glb.spike_ros_msg_list +=  (re.sub('@', port, motor_msg) + '\n')
    if device == 'color-sensor':
        with open(msg_info.source_path + 'SpikePrimeMessage/Color.msg')as c_msg:
            color_msg = c_msg.read()
        glb.spike_ros_msg_list +=  (re.sub('@', port, color_msg) + '\n')
    if device == 'ultrasonic-sensor':
        with open(msg_info.source_path + 'SpikePrimeMessage/Ultrasonic.msg')as u_msg:
            ultrasonic_msg = u_msg.read()
        glb.spike_ros_msg_list +=  (re.sub('@', port, ultrasonic_msg) + '\n')
    if device == 'force-sensor':
        with open(msg_info.source_path + 'SpikePrimeMessage/Force.msg')as f_msg:
            force_msg = f_msg.read()
        glb.spike_ros_msg_list +=  (re.sub('@', port, force_msg) + '\n')
    return