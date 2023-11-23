import yaml
import os
import shutil
import sys
import subprocess
import global_def as glb
import config_generator.config as conf
import config_generator.motor_conf_seter as m_set
import config_generator.color_conf_seter as c_set
import config_generator.ultrasonic_conf_seter as u_set
import config_generator.force_conf_seter as f_set
import config_generator.hub_conf_seter as h_set
import file_generator.c_body_generator as make_c
import file_generator.c_header_generator as make_h
import file_generator.custom_msg_generator as make_msg


#key_names = list()

def set_cfgs(port, cfg_list, cfg_contents):
    
    if port == 'hub':
        h_set.hub_seter(cfg_list, cfg_contents)

    elif 'Port' in port:
        if 'device' in cfg_list:
            device_name = cfg_contents[cfg_list.index('device')]
        else:
            print("err: Couldn't find the \"device\" configuration at " + port)
            exit(1)
        
        if device_name == 'motor':
            m_set.motor_seter(port, cfg_list, cfg_contents)
            glb.motor_number += 1
            glb.need_spike_prime_msg = True
            glb.need_custom_msg = True

        elif device_name == 'color-sensor':
            c_set.color_seter(port, cfg_list, cfg_contents)
            glb.color_number += 1
            glb.need_spike_prime_msg = True
            glb.need_custom_msg = True
        elif device_name == 'ultrasonic-sensor':
            u_set.ultrasonic_seter(port, cfg_list, cfg_contents)
            glb.ultra_number += 1
            glb.need_spike_prime_msg = True
            glb.need_custom_msg = True
        elif device_name == 'force-sensor':
            f_set.force_seter(port, cfg_list, cfg_contents)
            glb.force_number += 1
            glb.need_spike_prime_msg = True
            glb.need_custom_msg = True
        else:
            print("err: \"" + device_name + "\" is unknown device.")
            exit(1)
    else:
        print("err: \"" + port + "\" is unknown key.")
        exit(1)

    return

def arg_build_process():
    if '-lu' in glb.args:
        if os.path.isdir('../external/primehub/firmware/mcu_ws/spike_ros_msg'):
            shutil.rmtree('../external/primehub/firmware/mcu_ws/spike_ros_msg')
        shutil.copytree('gen/spike_ros_msg', '../external/primehub/firmware/mcu_ws/spike_ros_msg')
        try:
            subprocess.run("(cd ../external && make build_firmware)", shell=True, check=True)   # ライブラリのビルドを実行
        except subprocess.CalledProcessError:
            exit(1)
        if os.path.isdir('../spike-rt/spike_rt_uros'):
            shutil.rmtree('../spike-rt/spike_rt_uros')
        shutil.copytree('gen/spike_rt_uros', '../spike-rt/spike_rt_uros')
        try:
            subprocess.run("(cd ../spike-rt/spike_rt_uros && make asp.bin ; make deploy-dfu)", shell=True, check=True)   # uROSアプリのビルド・書き込み（並列ビルドによるエラーの対策の為，二回ビルドする）
        except subprocess.CalledProcessError:
            exit(1)
        return

    if '-l' in glb.args:
        if os.path.isdir('../external/primehub/firmware/mcu_ws/spike_ros_msg'):
            shutil.rmtree('../external/primehub/firmware/mcu_ws/spike_ros_msg')
        shutil.copytree('gen/spike_ros_msg', '../external/primehub/firmware/mcu_ws/spike_ros_msg')
        try:
            subprocess.run("(cd ../external && make build_firmware)", shell=True, check=True)   # ライブラリのビルドを実行
        except subprocess.CalledProcessError:
            exit(1)

    if '-u' in glb.args:
        if os.path.isdir('../spike-rt/spike_rt_uros'):
            shutil.rmtree('../spike-rt/spike_rt_uros')
        shutil.copytree('gen/spike_rt_uros', '../spike-rt/spike_rt_uros')
        try:
            subprocess.run("(cd ../spike-rt/spike_rt_uros && make asp.bin ; make deploy-dfu)", shell=True, check=True)
        except subprocess.CalledProcessError:
            exit(1)
    return

def export_ignore_count():

    if not conf.portA.ignore_conf == 0:
        print('WARN:' + str(conf.portA.ignore_conf) + ' unnecessary coufiguration on PortA was ignored.')
    if not conf.portB.ignore_conf == 0:
        print('WARN:' + str(conf.portB.ignore_conf) + ' unnecessary coufiguration on PortB was ignored.')
    if not conf.portC.ignore_conf == 0:
        print('WARN:' + str(conf.portC.ignore_conf) + ' unnecessary coufiguration on PortC was ignored.')
    if not conf.portD.ignore_conf == 0:
        print('WARN:' + str(conf.portD.ignore_conf) + ' unnecessary coufiguration on PortD was ignored.')
    if not conf.portE.ignore_conf == 0:
        print('WARN:' + str(conf.portE.ignore_conf) + ' unnecessary coufiguration on PortE was ignored.')
    if not conf.hub.ignore_conf == 0:
        print('WARN:' + str(conf.hub.ignore_conf) + ' unnecessary coufiguration on hub was ignored.')

def main():
    valid_args = ['-l', '-u', '-lu', '-c']
    invalid_args = list()

    if os.path.isdir('gen'):
        shutil.rmtree('gen')
    os.mkdir('gen')
    glb.args = sys.argv

    for arg_key in glb.args:
        if not arg_key in valid_args:
            invalid_args.append(arg_key)
    if len(invalid_args) >= 2:
        print('\'' + invalid_args[1] + '\' is invalid option.')
        exit(1)

    with open('uros_config.yml', encoding='utf-8')as f:
        motor_test = yaml.safe_load(f)

    for first_key in motor_test.keys():
            
        device_cfgs = list()
        cfg_contents = list()

        for second_key in motor_test[first_key].keys():
            device_cfgs.append(second_key)
            cfg_contents.append((motor_test[first_key])[second_key])

        set_cfgs(first_key, device_cfgs, cfg_contents)
    
    export_ignore_count()
    make_c.gen_c_body()
    make_h.gen_c_header()
    if glb.need_custom_msg == True:
        make_msg.gen_custom_msg()

    arg_build_process()
    
    if '-c' in glb.args:
        print("\n=======Port Configuration=======")
        print(glb.print_conf)
        print("=======Port Configuration=======\n")


    

# mainルーチン
if __name__ == "__main__":
	main()
