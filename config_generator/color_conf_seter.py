import config_generator.config as conf
import global_def as glb

def color_seter(port, config_keys, config_contents):
    
    if port == 'PortA':
        conf.portA.device = config_contents[config_keys.index('device')]
        conf.portA.port = 'PortA'

        conf.portA.config, conf.portA.ignore_conf = set_color_config_contents(port, config_keys, config_contents)
        conf.portA.config.port = 'A'
        conf_printer(conf.portA)
    
    elif port == 'PortB':
        conf.portB.device = config_contents[config_keys.index('device')]
        conf.portB.port = 'PortB'

        conf.portB.config, conf.portB.ignore_conf = set_color_config_contents(port, config_keys, config_contents)
        conf.portB.config.port = 'B'
        conf_printer(conf.portB)

    elif port == 'PortC':
        conf.portC.device = config_contents[config_keys.index('device')]
        conf.portC.port = 'PortC'

        conf.portC.config, conf.portC.ignore_conf = set_color_config_contents(port, config_keys, config_contents)
        conf.portC.config.port = 'C'
        conf_printer(conf.portC)

    elif port == 'PortD':
        conf.portD.device = config_contents[config_keys.index('device')]
        conf.portD.port = 'PortD'

        conf.portD.config, conf.portD.ignore_conf = set_color_config_contents(port, config_keys, config_contents)
        conf.portD.config.port = 'D'
        conf_printer(conf.portD)

    elif port == 'PortE':
        conf.portE.device = config_contents[config_keys.index('device')]
        conf.portE.port = 'PortE'

        conf.portE.config, conf.portE.ignore_conf = set_color_config_contents(port, config_keys, config_contents)
        conf.portE.config.port = 'E'
        conf_printer(conf.portE)

    else:
        print("err: \"" + port + "\" is unknown port.")
        exit(1)


def set_color_config_contents(port, keys, values):
    color_config_count = 0
    conf_contents = conf.color_sensor_class()
    
    if 'qos' in keys:
        qos_value = values[keys.index('qos')]
        if qos_value == 'best-effort' or qos_value == 'reliable':
            conf_contents.qos = qos_value
            color_config_count += 1
        else:
            setting_err(port, 'qos')
    else:
        conf_contents.qos = "best-effort"

    if 'enable_lights' in keys:
        is_enable_light = values[keys.index('enable_lights')]
        if type(is_enable_light) == bool:
            conf_contents.enable_lights = is_enable_light
            color_config_count += 1
        else:
            print('err: \"enable_lights\" must be a boolean.')
            exit(1)
    else:
        conf_contents.enable_lights = False
        
    if conf_contents.enable_lights == True:
        if 'light_qos' in keys:
            light_qos_value = values[keys.index('light_qos')]
            if light_qos_value == 'best-effort' or light_qos_value == 'reliable':
                conf_contents.light_qos = light_qos_value
                color_config_count += 1
            else:
                setting_err(port, 'light_qos')
        else:
            conf_contents.light_qos = 'best-effort'
    else:
        pass

    return conf_contents, (len(keys)-1) - color_config_count

def setting_err(port, key):
    print('err: Incorrect value for ' + port + ' ' + key + ' setting.')
    exit(1)

def conf_printer(obj):
    if '-c' in glb.args:
        glb.print_conf += (obj.port + '\n')
        glb.print_conf += (" device : " + obj.device + '\n')
        glb.print_conf += (" enable_lights : " + str(obj.config.enable_lights) + '\n')
        glb.print_conf += (" qos : " + obj.config.qos + '\n')
        glb.print_conf += (" light_qos : " + obj.config.light_qos + '\n')
    return