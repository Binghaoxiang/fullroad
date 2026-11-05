/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   串口命令解析层
  ******************************************************************************
  */
#ifndef __CMD_H
#define __CMD_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

void cmd_init(void);
void autoalign_run(void);
void autoalign_sweep(void);
void cmd_poll(void);            /* 主循环里反复调用，非阻塞 */
void cmd_exec(char *line);      /* 执行一条已去掉行尾的完整命令 */
void cmd_telem_poll(void);      /* 主循环里反复调用，按开关输出遥测 */

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H */
