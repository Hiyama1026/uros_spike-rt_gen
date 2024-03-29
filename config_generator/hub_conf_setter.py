import config_generator.config as conf
import global_def as glb

def hub_setter(config_keys, config_contents):
    hub_conf_count = 0

    if 'hub_program_cycle' in config_keys:
        hub_cycle = config_contents[config_keys.index('hub_program_cycle')]

        if type(hub_cycle) == int:

            if hub_cycle > 0:
                conf.hub.hub_program_cycle = hub_cycle
                hub_conf_count += 1
            else:
                print(glb.RED + 'err: \"hub_program_cycle\" must be a positive value.' + glb.RESET)
                exit(1)

        elif hub_cycle == None:
            conf.hub.hub_program_cycle = 100
            
        else:
            print(glb.RED + 'err: \"hub_program_cycle\" must be an integer.' + glb.RESET)
            exit(1)

    else:
        conf.hub.hub_program_cycle = 100
    
    if 'enable_imu' in config_keys:
        is_enable_imu = config_contents[config_keys.index('enable_imu')]
        if type(is_enable_imu) == bool or is_enable_imu == None:

            if is_enable_imu == None:
                conf.hub.enable_imu = False
            else:
                conf.hub.enable_imu = is_enable_imu
                hub_conf_count += 1

            if is_enable_imu == True:
                glb.need_spike_prime_msg = True
                glb.need_custom_msg = True

                if 'imu_mode' in config_keys:
                    imu_mode_val = config_contents[config_keys.index('imu_mode')]
                    if imu_mode_val == 'gyro' or imu_mode_val == 'temperature':
                        conf.hub.imu_mode = imu_mode_val
                        hub_conf_count += 1
                    elif imu_mode_val == None:
                        conf.hub.imu_mode = 'temperature'   # default = temperature
                    else:
                        setting_err('imu_mode')
                else:
                    conf.hub.imu_mode = 'temperature'   # default = temperature

            else:
                pass
        else:
            print(glb.RED + 'err: \"enable_imu\" must be a boolean.' + glb.RESET)
            exit(1)
    else:
        conf.hub.enable_imu = False
    
    if 'enable_battery_management' in config_keys:
        is_enable_battery = config_contents[config_keys.index('enable_battery_management')]

        if type(is_enable_battery) == bool:
            conf.hub.enable_battery_management = is_enable_battery
            hub_conf_count += 1
            if is_enable_battery == True:
                glb.need_custom_msg = True
        elif is_enable_battery == None:
            conf.hub.enable_battery_management = False
        else:
            print(glb.RED + 'err: \"enable_battery_management\" must be a boolean.' + glb.RESET)
            exit(1)
    else:
        conf.hub.enable_battery_management = False
    
    if 'enable_button' in config_keys:
        is_enable_button = config_contents[config_keys.index('enable_button')]

        if type(is_enable_button) == bool:
            conf.hub.enable_button = is_enable_button
            hub_conf_count += 1
        elif is_enable_button == None:
            conf.hub.enable_button = False
        else:
            print(glb.RED + 'err: \"enable_button\" must be a boolean.' + glb.RESET)
            exit(1)
    else:
        conf.hub.enable_button = False
        
    if 'enable_speaker' in config_keys:
        is_enable_speaker = config_contents[config_keys.index('enable_speaker')]

        if is_enable_speaker == True:
            glb.need_custom_msg = True
            conf.hub.enable_speaker = is_enable_speaker
            hub_conf_count += 1

            if 'speaker_volume' in config_keys:
                volume = config_contents[config_keys.index('speaker_volume')]
                if volume == None:
                    conf.hub.speaker_volume = 50
                elif  not type(volume) == int:
                    print(glb.RED + 'err: \"speaker_volume\" must be an integer.' + glb.RESET)
                    exit(1)
                else:
                    if volume >= 0 and volume <= 100:
                        conf.hub.speaker_volume = volume
                        hub_conf_count += 1
                    else:
                        setting_err('speaker_volume')
            else:
                conf.hub.speaker_volume = 50

        elif is_enable_speaker == False:
            conf.hub.enable_speaker = False
            hub_conf_count += 1
        elif is_enable_speaker == None:
            conf.hub.enable_speaker = False
        else:
            print(glb.RED + 'err: \"enable_speaker\" must be a boolean.' + glb.RESET)
            exit(1)
    else:
        conf.hub.enable_speaker = False

    if 'opening' in config_keys:
        is_enable_opening = config_contents[config_keys.index('opening')]

        if type(is_enable_opening) == bool:
            conf.hub.opening = is_enable_opening
            hub_conf_count += 1
        elif is_enable_opening == None:
            conf.hub.opening = True
        else:
            print(glb.RED + 'err: \"opening\" must be a boolean.' + glb.RESET)
            exit(1)
    else:
        conf.hub.opening = True

    conf.hub.ignore_conf = len(config_keys) - hub_conf_count
    
    if '-c' in glb.args:
        glb.print_conf += ("Hub" + '\n')
        glb.print_conf += (" hub_program_cycle : " + str(conf.hub.hub_program_cycle) + '\n')
        glb.print_conf += (" enable_imu : " + str(conf.hub.enable_imu) + '\n')
        if conf.hub.enable_imu:
            glb.print_conf += (" imu_mode : " + str(conf.hub.imu_mode) + '\n')
        else:
            glb.print_conf += (" imu_mode : ---\n")
        glb.print_conf += (" enable_battery_management : " + str(conf.hub.enable_battery_management) + '\n')
        glb.print_conf += (" enable_button : " + str(conf.hub.enable_button) + '\n')
        glb.print_conf += (" enable_speaker : " + str(conf.hub.enable_speaker) + '\n')
        if conf.hub.enable_speaker:
            glb.print_conf += (" speaker_volume : " + str(conf.hub.speaker_volume) + '\n')
        else:
            glb.print_conf += (" speaker_volume : ---\n")

    return

def setting_err(key):
    print(glb.RED + 'err: Incorrect value for hub ' + key + ' setting.' + glb.RESET)
    exit(1)
