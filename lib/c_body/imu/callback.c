void imu_init_callback(const void * msgin)
{
    const std_msgs__msg__Bool * imu_init = (const std_msgs__msg__Bool *)msgin;
    
    if(imu_init->data)	hub_imu_init();
}
 