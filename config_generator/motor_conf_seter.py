import config_generator.config as conf
import global_def as glb


def motor_seter(port, config_keys, config_contents):

    if port == 'PortA':
        conf.portA.device = config_contents[config_keys.index('device')]
        conf.portA.port = 'PortA'

        conf.portA.config, conf.portA.ignore_conf = set_motor_config_contents(port, config_keys, config_contents)
        conf.portA.config.port = 'A'

        conf_printer(conf.portA)
    
    elif port == 'PortB':
        conf.portB.device = config_contents[config_keys.index('device')]
        conf.portB.port = 'PortB'

        conf.portB.config, conf.portB.ignore_conf = set_motor_config_contents(port, config_keys, config_contents)
        conf.portB.config.port = 'B'

        conf_printer(conf.portB)
    
    elif port == 'PortC':
        conf.portC.device = config_contents[config_keys.index('device')]
        conf.portC.port = 'PortC'

        conf.portC.config, conf.portC.ignore_conf = set_motor_config_contents(port, config_keys, config_contents)
        conf.portC.config.port = 'C'

        conf_printer(conf.portC)

    elif port == 'PortD':
        conf.portD.device = config_contents[config_keys.index('device')]
        conf.portD.port = 'PortD'

        conf.portD.config, conf.portD.ignore_conf = set_motor_config_contents(port, config_keys, config_contents)
        conf.portD.config.port = 'D'

        conf_printer(conf.portD)

    elif port == 'PortE':
        conf.portE.device = config_contents[config_keys.index('device')]
        conf.portE.port = 'PortE'

        conf.portE.config, conf.portE.ignore_conf = set_motor_config_contents(port, config_keys, config_contents)
        conf.portE.config.port = 'E'

        conf_printer(conf.portE)
    
    else:
        print("err: \"" + port + "\" is unknown port.")
        exit(1)

    


def set_motor_config_contents(port, keys, values):
    motor_config_count = 0
    conf_contents = conf.motor_class()

    if 'wise' in keys:
        wise_val = values[keys.index('wise')]
        if wise_val == 'clock' or wise_val == 'counter-clock':
            conf_contents.wise = wise_val
            motor_config_count += 1
        else:
            setting_err(port, 'wise')
    else:
        conf_contents.wise = "clock"

    if 'qos' in keys:
        qos_value = values[keys.index('qos')]
        if qos_value == 'best-effort' or qos_value == 'reliable':
            conf_contents.qos = qos_value
            motor_config_count += 1
        else:
            setting_err(port, 'qos')
    else:
        conf_contents.qos = "best-effort"
    
    if 'run_mode' in keys:
        run_mode_val = values[keys.index('run_mode')]
        if run_mode_val == 'set-power' or run_mode_val == 'set-speed':
            conf_contents.run_mode = run_mode_val
            motor_config_count += 1
        else:
            setting_err(port, 'run_mode')
    else:
        conf_contents.run_mode = "set-power"

    return conf_contents, ((len(keys)-1) - motor_config_count)

def conf_printer(obj):
    if '-c' in glb.args:
        glb.print_conf += (obj.port + '\n')
        glb.print_conf += (" device : " + obj.device + '\n')
        glb.print_conf += (" wise : " + str(obj.config.wise) + '\n')
        glb.print_conf += (" qos : " + obj.config.qos + '\n')
        glb.print_conf += (" run_mode : " + obj.config.run_mode + '\n')
    return
                  
def setting_err(port, key):
    print('err: Incorrect value for ' + port + ' ' + key + ' setting.')
    exit(1)
