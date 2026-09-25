#ifndef Y42_STEPPER_H
#define Y42_STEPPER_H
#include <stdbool.h>
#include <stdint.h>

#define Y42_FULL_STEPS_PER_REV   (200U)
#define Y42_MICROSTEPS           (16U)
#define Y42_PULSES_PER_REV       (Y42_FULL_STEPS_PER_REV * Y42_MICROSTEPS)
#define Y42_MIN_PULSE_HZ          (20U)
#define Y42_MAX_PULSE_HZ          (20000U)

/* sysconfig: PWM_8=TIMG0 CCP0(PB10), stepper_DIR=PB1 */
#define Y42_STEP_INST      PWM_8_INST
#define Y42_STEP_CLK_FREQ  1000000
#define Y42_CC_IDX         GPIO_PWM_8_C0_IDX
#define Y42_DIR_PORT       (GPIOB)
#define Y42_DIR_PIN        stepper_DIR_PIN

typedef enum { Y42_DIR_CW=0, Y42_DIR_CCW=1 } Y42_Direction;

void Y42_Init(void);
bool Y42_RunRPM(float rpm, Y42_Direction direction);
bool Y42_MoveSteps(uint32_t steps, uint32_t pulse_hz, Y42_Direction direction);
bool Y42_MoveAngle(float angle_deg, float rpm, Y42_Direction direction);
void Y42_Stop(void);
bool Y42_IsBusy(void);
uint32_t Y42_GetTargetSteps(void);
uint32_t Y42_GetCompletedSteps(void);
#endif
