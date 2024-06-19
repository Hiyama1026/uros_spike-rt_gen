import yaml
import os
import shutil
import sys
import subprocess
import re
import global_def as glb
import config_generator.config as conf
import config_generator.motor_conf_setter as m_set
import config_generator.color_conf_setter as c_set
import config_generator.ultrasonic_conf_setter as u_set
import config_generator.force_conf_setter as f_set
import config_generator.hub_conf_setter as h_set
import file_generator.c_body_generator as make_c
import file_generator.c_header_generator as make_h
import file_generator.custom_msg_generator as make_msg

class Setup_val:
    def __init__(self):
        self.non_confirm = False

suv = Setup_val()

def show_help():
    with open('lib/usage.txt', encoding='utf-8')as hlp:
        h = hlp.read()
    
    print(h)
    exit(1)

def gen_yaml():
    port_name_code = ord('A')
    dev_candidates = ['motor', 'color-sensor', 'ultrasonic-sensor', 'force-sensor', 'none', '']
    Candedates = "[motor, color-sensor, ultrasonic-sensor, force-sensor, none (or empty)]"
    yml_txt = ''

    print("\"uros_config.yml\" generation mode.\n")

    if os.path.isfile('uros_config.yml'):
        # 設定ファイル削除確認 & 削除
        print("WARN: \"uros_config.yml\" is already exist.")
        while True:
            c = input('Are you sure you want to permanently delete the old configuration file? [Y/n]: ')
            if c in ['y', 'Y', 'yes', 'Yes', 'YES', '']:
                os.remove('uros_config.yml')
                print("Delete old configuration file.\n")
                break
            elif c in ['n', 'N', 'no', 'No', 'NO']:
                print('Generation process stop.')
                exit(1)
            else:
                print('Input error.')

    print("Input device name.")
    print(glb.CYAN + glb.UNDERLINE + "Candedates : " + Candedates + glb.RESET)
    
    while port_name_code <= ord('E'):
        while True:
            dev = input('- Port' + chr(port_name_code) + ' : ')
            if dev in dev_candidates:
                yml_txt += get_yml_txt(chr(port_name_code), dev)
                break
            else:
                print("\"" + dev + "\" is not among the candedates...")
        port_name_code += 1
    
    with open('./lib/yaml/hub.yml', encoding='utf-8')as h_yaml:
        hub_yaml = h_yaml.read()
    yml_txt += hub_yaml

    s_yaml = open('uros_config.yml', 'w')
    s_yaml.write(yml_txt)
    s_yaml.close()

    print("\nConfiguration file template successfully generated.")
    exit(1)

def get_yml_txt(port_name, dev_name):
    yml_lib_path = './lib/yaml/'
    yml_txt = ''

    if (dev_name == 'none') or (dev_name == ''):
        return ''
    else:
        yml_txt = ('Port' + port_name + ':\n')
        yml_lib_path += (dev_name + '.yml')
        with open(yml_lib_path, encoding='utf-8')as lib_yaml:
            l_yaml = lib_yaml.read()
            yml_txt += (l_yaml + '\n\n')
        
        return yml_txt

def set_cfgs(port, cfg_list, cfg_contents):
    
    if port == 'hub':
        h_set.hub_setter(cfg_list, cfg_contents)

    elif 'Port' in port:
        if 'device' in cfg_list:
            device_name = cfg_contents[cfg_list.index('device')]
        else:
            print(glb.RED + "err: Couldn't find the \"device\" configuration at " + port + glb.RESET)
            exit(1)
        
        if device_name == 'motor':
            m_set.motor_setter(port, cfg_list, cfg_contents)
            glb.motor_number += 1
            glb.need_spike_prime_msg = True
            glb.need_custom_msg = True

        elif device_name == 'color-sensor':
            c_set.color_setter(port, cfg_list, cfg_contents)
            glb.color_number += 1
            glb.need_spike_prime_msg = True
            glb.need_custom_msg = True
        elif device_name == 'ultrasonic-sensor':
            u_set.ultrasonic_setter(port, cfg_list, cfg_contents)
            glb.ultra_number += 1
            glb.need_spike_prime_msg = True
            glb.need_custom_msg = True
        elif device_name == 'force-sensor':
            f_set.force_setter(port, cfg_list, cfg_contents)
            glb.force_number += 1
            glb.need_spike_prime_msg = True
            glb.need_custom_msg = True
        else:
            print(glb.RED + "err: \"" + device_name + "\" is unknown device." + glb.RESET)
            exit(1)
    else:
        print(glb.RED + "err: \"" + port + "\" is unknown key." + glb.RESET)
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
            print(glb.RED + 'err: Package name must begin with an alphabet.' + glb.RESET)
            exit(1)

        for char in glb.package_name:
            if (not is_alphabet(char)) and (not is_valid_char(char)):
                print(glb.RED + 'err: Package name error. (inviled: ' + char + ')' + glb.RESET)
                exit(1)
        
        glb.msg_pkg_name = glb.package_name + '_msg'
    
    if '-py'in glb.args:
        create_ros2_pkg('-py')

    if '-cpp' in glb.args:
        create_ros2_pkg('-cpp')
    return

def create_ros2_pkg(opt):
    opt_index = glb.args.index(opt)
    ros2_pkg = glb.args[opt_index + 1]
    glb.args.remove(ros2_pkg)
    ros2_ws_path = '../../ros2_ws/'
    ros2_source_path = ros2_ws_path + 'src/'
    create_cmd = "(cd ../../ros2_ws/src && "

    if not is_alphabet(ros2_pkg[0]):
        print(glb.RED + 'err: ROS2 package name must begin with an alphabet.' + glb.RESET)
        exit(1)

    for char in ros2_pkg:
        if (not is_alphabet(char)) and (not is_valid_char(char)):
            print(glb.RED + 'err: Package name error. (inviled: ' + char + ')' + glb.RESET)
            exit(1)

    if not os.path.isdir(ros2_ws_path):
        print(glb.RED + 'err: Can not find ros2_ws' + glb.RESET)
        exit(1)
    
    if not os.path.isdir(ros2_source_path):
        os.mkdir(ros2_source_path)
    
    if os.path.isdir(ros2_source_path + ros2_pkg):
        duplicate_pkg(ros2_pkg, ros2_source_path, 'ROS2')
        shutil.rmtree(ros2_source_path + ros2_pkg)
    
    if opt == '-py':
        create_cmd += "ros2 pkg create --build-type ament_python " + ros2_pkg + " --dependencies rclpy)"
    else:
        create_cmd += "ros2 pkg create --build-type ament_cmake " + ros2_pkg + ")"

    try:
        subprocess.run(create_cmd, shell=True, check=True)   # ROS2パッケージ生成
    except subprocess.CalledProcessError:
        exit(1)
    print('\nSucceed to create a ROS2 package.\n\n')
    return

def arg_build_process():
    target_msg_path = '../external/primehub/firmware/mcu_ws/' + glb.msg_pkg_name
    target_uros_path = '../spike-rt/' + glb.package_name
    msg_source_path = 'gen/' + glb.package_name + '/' + glb.msg_pkg_name
    uros_source_path = 'gen/' + glb.package_name + '/' + glb.package_name
    cmd_build_uros = "(cd " + target_uros_path + " && make && make deploy-dfu)"
    ros2_target_path = '../../ros2_ws/src/' + glb.msg_pkg_name
    is_built = False

    if '-lu' in glb.args:
        add_include_to_makefile()

        if os.path.isdir(target_msg_path):
            duplicate_pkg(glb.msg_pkg_name, target_msg_path, 'message_def')
            shutil.rmtree(target_msg_path)
        shutil.copytree(msg_source_path, target_msg_path)
        try:
            subprocess.run("(cd ../external && make build_firmware)", shell=True, check=True)   # ライブラリのビルドを実行
        except subprocess.CalledProcessError:
            exit(1)
        if os.path.isdir(target_uros_path):
            duplicate_pkg(glb.package_name, target_uros_path, 'micro-ROS')
            shutil.rmtree(target_uros_path)
        shutil.copytree(uros_source_path, target_uros_path)
        try:
            subprocess.run(cmd_build_uros, shell=True, check=True)   
        except subprocess.CalledProcessError:
            try:
                subprocess.run(cmd_build_uros, shell=True, check=True)
            except subprocess.CalledProcessError:
                exit(1)
        is_built = True

    if '-l' in glb.args and not is_built:
        add_include_to_makefile()
        if os.path.isdir(target_msg_path):
            duplicate_pkg(glb.msg_pkg_name, target_msg_path, 'message_def')
            shutil.rmtree(target_msg_path)
        shutil.copytree(msg_source_path, target_msg_path)
        try:
            subprocess.run("(cd ../external && make build_firmware)", shell=True, check=True)   # ライブラリのビルドを実行
        except subprocess.CalledProcessError:
            exit(1)
    
    if '-u' in glb.args and not is_built:
        add_include_to_makefile()

        if os.path.isdir(target_uros_path):
            duplicate_pkg(glb.package_name, target_uros_path, 'micro-ROS')
            shutil.rmtree(target_uros_path)
        shutil.copytree(uros_source_path, target_uros_path)
        try:
            subprocess.run(cmd_build_uros, shell=True, check=True)   
        except subprocess.CalledProcessError:
            try:
                print('\nLog from "uros_spike-rt_gen": Retry "$ make deploy-dfu"\n')
                subprocess.run(cmd_build_uros, shell=True, check=True)
            except subprocess.CalledProcessError:
                exit(1)  
    
    if '-mc' in glb.args:
        if os.path.isdir(ros2_target_path):
            duplicate_pkg(glb.msg_pkg_name, ros2_target_path, 'message_def')
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

def duplicate_pkg(pkg_name, path, pkg_type):
    if suv.non_confirm:
        return

    print('\nWARN: "' + pkg_name + '" is already exist in "' + path + '".')
    while True:
        c = input('Are you sure you want to permanently delete this old ' + pkg_type + ' package? [Y/n]: ')
        if c in ['y', 'Y', 'yes', 'Yes', 'YES', '']:
            print('Remake "' + pkg_name + '".')
            return
        elif c in ['n', 'N', 'no', 'No', 'NO']:
            print('Stop generate packages.')
            exit(1)
        else:
            print('Input error.')

def ignore_warnning_printer(ig_num, port):
    if ig_num == 1:
        print(glb.YELLOW + '   1 unknown coufiguration on ' + port + ' was ignored.' + glb.RESET)
    else:
        print(glb.YELLOW + '   ' + str(ig_num) + ' unknown coufigurations on ' + port + ' were ignored.' + glb.RESET)

def export_ignore_count():
    ig_count = conf.portA.ignore_conf + conf.portB.ignore_conf + conf.portC.ignore_conf + conf.portD.ignore_conf + conf.portE.ignore_conf + conf.hub.ignore_conf

    if ig_count == 0:
        return

    print(glb.YELLOW + '\n WARN: Ignored configuration exist (Warnning from uros_spike-rt_gen).' + glb.RESET)
    if not conf.portA.ignore_conf == 0:
        ignore_warnning_printer(conf.portA.ignore_conf, 'PortA')
    if not conf.portB.ignore_conf == 0:
        ignore_warnning_printer(conf.portB.ignore_conf, 'PortB')
    if not conf.portC.ignore_conf == 0:
        ignore_warnning_printer(conf.portC.ignore_conf, 'PortC')
    if not conf.portD.ignore_conf == 0:
        ignore_warnning_printer(conf.portD.ignore_conf, 'PortD')
    if not conf.portE.ignore_conf == 0:
        ignore_warnning_printer(conf.portE.ignore_conf, 'PortE')
    if not conf.hub.ignore_conf == 0:
        ignore_warnning_printer(conf.hub.ignore_conf, 'hub')

def export_log():
    ex_path = 'gen/' + glb.package_name + '/' + glb.package_name + '.log'

    shutil.copy('./uros_config.yml', ex_path)
#
# main()
#
def main():
    valid_args = ['-l', '-u', '-lu', '-c', '-n', '-mc', '-py', '-cpp', '-f', '-h', '-setup']
    invalid_args = list()
    gen_path = 'uros_spike-rt_gen/gen'

    glb.args = sys.argv

    if '-h' in glb.args:
        show_help()
    if '-setup' in glb.args:
        gen_yaml()

    if not os.path.isdir('gen'):
        os.mkdir('gen')
    
    if '-f' in glb.args:
        suv.non_confirm = True

    read_name()
    
    if os.path.isdir('gen/' + glb.package_name):
        duplicate_pkg(glb.package_name, gen_path, 'micro-ROS')
        shutil.rmtree('gen/' + glb.package_name)


    for arg_key in glb.args:
        if not arg_key in valid_args:
            invalid_args.append(arg_key)
    if len(invalid_args) >= 2:
        print('\'' + invalid_args[1] + '\' is invalid option.')
        exit(1)


    with open('uros_config.yml', encoding='utf-8')as f:
        first_kval = yaml.safe_load(f)

    for first_key in first_kval.keys():
            
        device_cfgs = list()
        cfg_contents = list()
        if not first_kval[first_key] == None:
            for second_key in first_kval[first_key].keys():
                device_cfgs.append(second_key)
                cfg_contents.append((first_kval[first_key])[second_key])
        else:
            cfg_contents = None

        set_cfgs(first_key, device_cfgs, cfg_contents)
    
    make_c.gen_c_body()
    make_h.gen_c_header()
    if glb.need_custom_msg == True:
        make_msg.gen_custom_msg()

    arg_build_process()

    export_log()
    
    if '-c' in glb.args:
        print("\n======= Configuration =======")
        print('Package name : ' + glb.package_name + '\n')
        print(glb.print_conf)
        print("======= Configuration =======\n")
    
    export_ignore_count()
    
    print('\nFrimware generation succeed!!')

# mainルーチン
if __name__ == "__main__":
	main()
