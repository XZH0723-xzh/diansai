/*
 * slave_protocol.h - 从板接收主控指令
 *
 * 指令格式: $T,N#  启动任务N
 *           $X#    紧急停止
 */
#ifndef SLAVE_PROTOCOL_H
#define SLAVE_PROTOCOL_H

void slave_proto_init(void);       /* 初始化UART接收 */
void slave_proto_poll(void);       /* 主循环轮询, 非阻塞 */
int  slave_proto_task(void);       /* 当前任务号, 0=无 */
int  slave_proto_task_new(void);   /* 新任务触发, 读取后清零 */

#endif
