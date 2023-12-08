import re
import config_generator.config as conf
import global_def as glb

def gen_c_header():
    bace_path = 'lib/c_body/bace/c_bace_header.c'
    lib_path = 'lib/c_body/bace/lib_header_bace.c'
    # create target path
    target_path = 'gen/' + glb.package_name + '/' + glb.package_name + '/'

    with open(bace_path) as c_hesd:
        c_hesd_string = c_hesd.read()
    
    # generate uros.h
    c_hesd_string = re.sub('@send_cyc@', str(conf.hub.hub_program_cycle), c_hesd_string)
    if conf.hub.opening == True:
        c_hesd_string = re.sub('@opening@', 'true', c_hesd_string)
    else:
        c_hesd_string = re.sub('@opening@', 'false', c_hesd_string)

    c_h = open(target_path + 'uros.h', 'w')
    c_h.write(c_hesd_string)
    c_h.close()
    
    # generate lib.h
    with open(lib_path) as lib_h:
        l_head_string = lib_h.read()

    if glb.color_sensor_exist == True:
        l_head_string = re.sub('@USE_COLOR@', 'true', l_head_string)
    else:
        l_head_string = re.sub('@USE_COLOR@', 'false', l_head_string)

    if glb.ultrasonic_sensor_exist == True:
        l_head_string = re.sub('@USE_ULTRASONIC@', 'true', l_head_string)
    else:
        l_head_string = re.sub('@USE_ULTRASONIC@', 'false', l_head_string)

    if glb.force_sensor_exist == True:
        l_head_string = re.sub('@USE_FORCE@', 'true', l_head_string)
    else:
        l_head_string = re.sub('@USE_FORCE@', 'false', l_head_string)

    if conf.hub.enable_button == True:
        l_head_string = re.sub('@USE_BUTTON@', 'true', l_head_string)
    else:
        l_head_string = re.sub('@USE_BUTTON@', 'false', l_head_string)

    l_h = open(target_path + 'lib.h', 'w')
    l_h.write(l_head_string)
    l_h.close()

    return