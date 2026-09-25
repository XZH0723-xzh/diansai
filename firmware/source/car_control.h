#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

int  ir_read_line(void);          /* 加权位置 -100~+100, 127=丢线 */
int  line_follow_pid(void);       /* PID巡线, 返回转向量 */
void motor_drive(int speed, int steer);

#endif
