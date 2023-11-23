// SPDX-License-Identifier: MIT
/*
 * Copyright (c) 2022 Embedded and Real-Time Systems Laboratory,
 *                    Graduate School of Information Science, Nagoya Univ., JAPAN
 */
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "spike_rt_uros.h"

/*
 *  メインタスク
 */
void
main_task(intptr_t exinf)
{

    while (1) {
        slp_tsk();     
    }
}
