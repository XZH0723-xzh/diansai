/*
 * y42_stepper.c - Y42步进 STEP/DIR (原版)
 */
#include "ti_msp_dl_config.h"
#include "y42_stepper.h"

#define Y42_CW_DIR_LEVEL    (1U)
#define Y42_CCW_DIR_LEVEL   (0U)
#define Y42_DIR_SETUP_US    (50U)

static volatile uint32_t g_target_steps;
static volatile uint32_t g_completed_steps;
static volatile bool g_busy;
static volatile bool g_continuous;

static void Y42_DelayUs(uint32_t us) {
    uint32_t c = CPUCLK_FREQ / 1000000U;
    if (c == 0U) c = 1U;
    delay_cycles(c * us);
}

static void Y42_SetDirection(Y42_Direction direction) {
    uint32_t level = (direction == Y42_DIR_CCW) ? Y42_CCW_DIR_LEVEL : Y42_CW_DIR_LEVEL;
    if (level != 0U) DL_GPIO_setPins(Y42_DIR_PORT, Y42_DIR_PIN);
    else             DL_GPIO_clearPins(Y42_DIR_PORT, Y42_DIR_PIN);
    Y42_DelayUs(Y42_DIR_SETUP_US);
}

static bool Y42_ConfigurePulseFrequency(uint32_t pulse_hz) {
    if ((pulse_hz < Y42_MIN_PULSE_HZ) || (pulse_hz > Y42_MAX_PULSE_HZ)) return false;
    uint32_t period_count = (Y42_STEP_CLK_FREQ + (pulse_hz / 2U)) / pulse_hz;
    if ((period_count < 4U) || (period_count > 65535U)) return false;
    uint32_t compare_count = period_count / 2U;
    DL_TimerG_stopCounter(Y42_STEP_INST);
    DL_TimerG_setCCPOutputDisabled(Y42_STEP_INST, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
    DL_TimerG_setLoadValue(Y42_STEP_INST, period_count);
    DL_TimerG_setCaptureCompareValue(Y42_STEP_INST, compare_count, Y42_CC_IDX);
    DL_TimerG_setTimerCount(Y42_STEP_INST, 0U);
    DL_TimerG_clearInterruptStatus(Y42_STEP_INST, 0xFFFFFFFFU);
    return true;
}

void Y42_Init(void) {
    DL_TimerG_stopCounter(Y42_STEP_INST);
    DL_TimerG_disableInterrupt(Y42_STEP_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    DL_TimerG_clearInterruptStatus(Y42_STEP_INST, 0xFFFFFFFFU);
    DL_GPIO_clearPins(Y42_DIR_PORT, Y42_DIR_PIN);
    g_target_steps = 0U; g_completed_steps = 0U;
    g_busy = false; g_continuous = false;
    NVIC_ClearPendingIRQ(PWM_8_INST_INT_IRQN);
    NVIC_EnableIRQ(PWM_8_INST_INT_IRQN);
}

bool Y42_MoveAngle(float angle_deg, float rpm, Y42_Direction direction) {
    if ((angle_deg <= 0.0f) || (rpm <= 0.0f)) return false;
    float steps_f = angle_deg * (float)Y42_PULSES_PER_REV / 360.0f;
    float hz_f = rpm * (float)Y42_PULSES_PER_REV / 60.0f;
    uint32_t steps = (uint32_t)(steps_f + 0.5f);
    uint32_t hz = (uint32_t)(hz_f + 0.5f);
    if (steps == 0U) return false;
    Y42_Stop();
    Y42_SetDirection(direction);
    if (!Y42_ConfigurePulseFrequency(hz)) return false;
    g_target_steps = steps; g_completed_steps = 0U;
    g_continuous = false; g_busy = true;
    DL_TimerG_enableInterrupt(Y42_STEP_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(PWM_8_INST_INT_IRQN);
    DL_TimerG_startCounter(Y42_STEP_INST);
    return true;
}

void Y42_Stop(void) {
    DL_TimerG_disableInterrupt(Y42_STEP_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    DL_TimerG_stopCounter(Y42_STEP_INST);
    DL_TimerG_setCaptureCompareValue(Y42_STEP_INST, 0U, Y42_CC_IDX);
    DL_TimerG_clearInterruptStatus(Y42_STEP_INST, 0xFFFFFFFFU);
    g_busy = false; g_continuous = false;
}

bool Y42_IsBusy(void) { return g_busy; }

void PWM_8_INST_IRQHandler(void) {
    switch (DL_TimerG_getPendingInterrupt(Y42_STEP_INST)) {
    case DL_TIMER_IIDX_LOAD:
        if (g_busy && !g_continuous) {
            g_completed_steps++;
            if (g_completed_steps >= g_target_steps) {
                DL_TimerG_stopCounter(Y42_STEP_INST);
                DL_TimerG_disableInterrupt(Y42_STEP_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
                DL_TimerG_setCaptureCompareValue(Y42_STEP_INST, 0U, Y42_CC_IDX);
                g_busy = false;
            }
        }
        break;
    default: break;
    }
}
