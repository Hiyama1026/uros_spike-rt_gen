import config_generator.config as conf
import global_def as glb

def force_seter(port, config_keys, config_contents):
    
    if port == 'PortA':
        conf.portA.device = config_contents[config_keys.index('device')]
        conf.portA.port = 'PortA'
        conf_printer(conf.portA)

        conf.portA.config = None
        
        conf.portA.config, conf.portA.ignore_conf = set_force_config_contents(config_keys)
        conf.portA.config.port = 'A'

    elif port == 'PortB':
        conf.portB.device = config_contents[config_keys.index('device')]
        conf.portB.port = 'PortB'
        conf_printer(conf.portB)

        conf.portB.config = None
        
        conf.portB.config, conf.portB.ignore_conf = set_force_config_contents(config_keys)
        conf.portB.config.port = 'B'

    elif port == 'PortC':
        conf.portC.device = config_contents[config_keys.index('device')]
        conf.portC.port = 'PortC'
        conf_printer(conf.portC)

        conf.portC.config = None
        
        conf.portC.config, conf.portC.ignore_conf = set_force_config_contents(config_keys)
        conf.portC.config.port = 'C'

    elif port == 'PortD':
        conf.portD.device = config_contents[config_keys.index('device')]
        conf.portD.port = 'PortD'
        conf_printer(conf.portD)

        conf.portD.config = None
        
        conf.portD.config, conf.portD.ignore_conf = set_force_config_contents(config_keys)
        conf.portD.config.port = 'D'

    elif port == 'PortE':
        conf.portE.device = config_contents[config_keys.index('device')]
        conf.portE.port = 'PortE'
        conf_printer(conf.portE)

        conf.portE.config = None
        
        conf.portE.config, conf.portE.ignore_conf = set_force_config_contents(config_keys)
        conf.portE.config.port = 'E'

    else:
        print("err: \"" + port + "\" is unknown port.")
        exit(1)
    
    return

def conf_printer(obj):
    if '-c' in glb.args:
        glb.print_conf += (obj.port + '\n')
        glb.print_conf += (" device : " + obj.device + '\n')
        

def set_force_config_contents( keys):
    force_config_count = 0
    conf_contents = conf.force_sensor_class()

    return conf_contents, (len(keys)-1) - force_config_count