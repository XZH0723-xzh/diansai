#ifndef Y42_STEPPER_H
#define Y42_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * y42_stepper.h：Y42闭环步进电机驱动的公开接口
 *
 * 本驱动采用STEP/DIR方式控制Y42：
 *   PA30 -> Y42 Stp：每个有效脉冲让电机前进一个细分步；
 *   PB11 -> Y42 Dir：高、低电平决定运动方向；
 *   MSPM0 GND、Y42 GND和独立电源负极必须共地。
 *
 * 电机整步角为1.8°，所以一圈包含：
 *   360° ÷ 1.8° = 200个整步。
 *
 * 当前假设Y42菜单设置为16细分：
 *   200整步/圈 × 16细分 = 3200个STEP脉冲/圈。
 *
 * 如果在Y42菜单中改变了细分值，必须同步修改Y42_MICROSTEPS，
 * 否则“转一圈”和“转指定角度”的实际结果会不正确。
 */
#define Y42_FULL_STEPS_PER_REV       (200U)
#define Y42_MICROSTEPS               (16U)
#define Y42_PULSES_PER_REV           (Y42_FULL_STEPS_PER_REV * Y42_MICROSTEPS)

/*
 * 软件允许的STEP脉冲频率范围。
 *
 * 下限20 Hz用于避免定时器周期过长；
 * 上限20000 Hz是本项目主动设置的安全上限，不是Y42硬件的绝对极限。
 * 当前16细分时，20000 Hz约等于375 RPM。
 *
 * 频率越高不一定越好：机械负载、电源能力以及是否使用加减速曲线，
 * 都会影响电机能否稳定启动和运行。
 */
#define Y42_MIN_PULSE_HZ             (20U)
#define Y42_MAX_PULSE_HZ             (20000U)

/* 程序使用的逻辑方向；若实物方向相反，可在y42_stepper.c中交换电平。 */
typedef enum {
    Y42_DIR_CW = 0,   /* clockwise：顺时针 */
    Y42_DIR_CCW = 1   /* counter-clockwise：逆时针 */
} Y42_Direction;

/*
 * 初始化Y42软件状态和TIMG8中断。
 * 必须先调用SYSCFG_DL_init()，再调用本函数。
 */
void Y42_Init(void);

/*
 * 以指定机械转速连续运行。
 *
 * rpm：目标转速，单位为转/分钟；
 * direction：顺时针或逆时针；
 * 返回true表示已经开始输出脉冲；
 * 返回false表示rpm换算出的脉冲频率超出允许范围。
 *
 * 本函数非阻塞，电机将持续运行，必须调用Y42_Stop()才会停止。
 */
bool Y42_RunRPM(float rpm, Y42_Direction direction);

/*
 * 输出指定数量的STEP脉冲。
 *
 * steps：需要输出的脉冲总数；
 * pulse_hz：STEP频率，单位为脉冲/秒；
 * direction：运动方向。
 *
 * 本函数非阻塞；启动成功后通过Y42_IsBusy()判断是否完成。
 */
bool Y42_MoveSteps(uint32_t steps, uint32_t pulse_hz,
                   Y42_Direction direction);

/*
 * 按角度和转速进行定量运动。
 *
 * angle_deg：需要旋转的机械角度，必须为正数；
 * rpm：目标机械转速，必须为正数；
 * direction：运动方向。
 *
 * 函数内部会把角度换算成脉冲数，把RPM换算成脉冲频率，
 * 然后调用Y42_MoveSteps()。
 */
bool Y42_MoveAngle(float angle_deg, float rpm,
                   Y42_Direction direction);

/* 立即停止定时器和STEP输出，并清除“正在运动”状态。 */
void Y42_Stop(void);

/* 返回true表示定量运动尚未完成或电机正在连续运行。 */
bool Y42_IsBusy(void);

/* 获取本次定量运动要求输出的总脉冲数。 */
uint32_t Y42_GetTargetSteps(void);

/* 获取本次定量运动已经完成的脉冲数。 */
uint32_t Y42_GetCompletedSteps(void);

#endif /* Y42_STEPPER_H */
