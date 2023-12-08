import yaml
import os
import shutil
import sys
import subprocess
import re
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

def is_alphabet(char):
    if char >= 'a' and char <= 'z':
        return True
    elif char >= 'A' and char <= 'Z':
        return True
    else:
        return False

def is_valid_char(char):
    valid_sp_chars = ['_']

    if char in valid_sp_chars:
        return True
    elif char >= '0' and char <= '9':
        return True
    else:
        return False

def read_name():
    opt_index = 0

    if '-n' in glb.args:
        opt_index = glb.args.index('-n')
        name_index = opt_index + 1
        glb.package_name = glb.args[name_index]
        glb.args.remove(glb.package_name)
        
        if not is_alphabet(glb.package_name[0]):
            print('err: Package name must begin with an alphabet.')
            exit(1)

        for char in glb.package_name:
            if (not is_alphabet(char)) and (not is_valid_char(char)):
                print('err: Package name error. (inviled: ' + char + ')')
                exit(1)
        
        glb.msg_pkg_name = glb.package_name + '_msg'
    return


def arg_build_process():
    target_msg_path = '../external/primehub/firmware/mcu_ws/' + glb.msg_pkg_name
    target_uros_path = '../spike-rt/' + glb.package_name
    msg_source_path = 'gen/' + glb.package_name + '/' + glb.msg_pkg_name
    uros_source_path = 'gen/' + glb.package_name + '/' + glb.package_name
    cmd_build_uros = "(cd " + target_uros_path + " && make deploy-dfu)"
    ros2_target_path = '../../ros2_ws/src/' + glb.msg_pkg_name
    is_built = False

    if '-lu' in glb.args:
        add_include_to_makefile()

        if os.path.isdir(target_msg_path):
            shutil.rmtree(target_msg_path)
        shutil.copytree(msg_source_path, target_msg_path)
        try:
            subprocess.run("(cd ../external && make build_firmware)", shell=True, check=True)   # ライブラリのビルドを実行
        except subprocess.CalledProcessError:
            exit(1)
        if os.path.isdir(target_uros_path):
            shutil.rmtree(target_uros_path)
        shutil.copytree(uros_source_path, target_uros_path)
        try:
            subprocess.run(cmd_build_uros, shell=True, check=True)   
        except subprocess.CalledProcessError:
            try:
                subprocess.run(cmd_build_uros, shell=True, check=True)     # 並列ビルドによるエラー対策のためリトライ
            except subprocess.CalledProcessError:
                exit(1)
        is_built = True

    if '-l' in glb.args and not is_built:
        add_include_to_makefile()
        if os.path.isdir(target_msg_path):
            shutil.rmtree(target_msg_path)
        shutil.copytree(msg_source_path, target_msg_path)
        try:
            subprocess.run("(cd ../external && make build_firmware)", shell=True, check=True)   # ライブラリのビルドを実行
        except subprocess.CalledProcessError:
            exit(1)
    
    if '-u' in glb.args and not is_built:
        add_include_to_makefile()

        if os.path.isdir(target_uros_path):
            shutil.rmtree(target_uros_path)
        shutil.copytree(uros_source_path, target_uros_path)
        try:
            subprocess.run("(cd ../spike-rt/spike_rt_uros && make deploy-dfu)", shell=True, check=True)   
        except subprocess.CalledProcessError:
            try:
                print('\nLog from "uros_spike-rt_gen": Retry "$ make deploy-dfu"\n')
                subprocess.run(cmd_build_uros, shell=True, check=True)     # 並列ビルドによるエラー対策のためリトライ
            except subprocess.CalledProcessError:
                exit(1)  
    
    if '-ro' in glb.args:
        if os.path.isdir(ros2_target_path):
            shutil.rmtree(ros2_target_path)
        shutil.copytree(msg_source_path, ros2_target_path)
    return

def add_include_to_makefile():
    include_form = 'INCLUDES += -I$(MIROROS_ASP3_TOP_DIR)/$(MICROROS_INC)/@pkg_name@/\n'
    search_text = ('INCLUDES += -I$(MIROROS_ASP3_TOP_DIR)/$(MICROROS_INC)/' + glb.msg_pkg_name + '/\n')
    mk_path = '../micro_ros_asp/micro_ros_asp.mk'

    with open(mk_path) as makefile:
        mk = makefile.read()
    
    if not '\n# messages generated by "uros_spike-rt_gen"\n' in mk:
        mk += '\n# messages generated by "uros_spike-rt_gen"\n'
    
    # create include
    if not search_text in mk:
        include_form = re.sub('@pkg_name@', glb.msg_pkg_name, include_form)
        mk += include_form
    
    makefile = open(mk_path, 'w')
    makefile.write(mk)
    makefile.close()

def duplicate_pkg():
    print('WARN: "' + glb.package_name + '" is already exist.')
    while True:
        c = input('Do you want to delete the old package? [Y/n]: ')
        if c in ['y', 'Y', 'yes', 'Yes', 'YES', '']:
            print('Remake "' + glb.package_name + '".')
            return
        elif c in ['n', 'N', 'no', 'No', 'NO']:
            print('Stop generate packages.')
            exit(1)
        else:
            print('Input error.')


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

#
# main()
#
def main():
    valid_args = ['-l', '-u', '-lu', '-c', '-n', '-ro']
    invalid_args = list()

    if not os.path.isdir('gen'):
        os.mkdir('gen')
    glb.args = sys.argv
    
    read_name()
    
    if os.path.isdir('gen/' + glb.package_name):
        duplicate_pkg()
        shutil.rmtree('gen/' + glb.package_name)


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
        print("\n======= Configuration =======")
        print('Package name : ' + glb.package_name + '\n')
        print(glb.print_conf)
        print("======= Configuration =======\n")
    

# mainルーチン
if __name__ == "__main__":
	main()
