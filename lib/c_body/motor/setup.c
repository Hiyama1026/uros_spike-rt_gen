    for(int i = 0; i < 10; i++)
    {
        motor@@_err = pup_motor_setup(motor@@, @wise@, true);
        pup_motor_reset_count(motor@@);
        if(motor@@_err != PBIO_ERROR_AGAIN){
            break;
        }
        // Set up failed -> Wait 1s and ry one more
        dly_tsk(1000000);
    }