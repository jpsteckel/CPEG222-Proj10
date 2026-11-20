/****************************************************************
 * CPEG222 Project 10 - Line Following Robot
 * Nucleo F446ZE, CMSIS (bare metal), system clock 180 MHz
 *
 * Behaviour
 *   - Press UP (PF7) to start, CENTER (PF8) to stop at any time.
 *   - Follows the black line on the white cardboard with a PD
 *     controller fed by the five analog IR sensor readings, plus a
 *     pivot behaviour for sharp (90 degree or tighter) corners.
 *   - Passes through the first crossing and stops on the second one.
 *   - Mirrors each IR sensor onto the front LEDs exactly as in the
 *     IR sensor demo (IR1->LED1, IR2->LED2, IR3->LED4+LED5,
 *     IR4->LED7, IR5->LED8).
 *
 * Pin map (see "Ports and Pins.pdf")
 *   IR1 PA3, IR2 PC0, IR3 PC3, IR4 PF3, IR5 PF5  (ADC3)
 *   LEDs PD0..PD7
 *   Left servo  PD12 (TIM4_CH1), Right servo PD14 (TIM4_CH3)
 *   UP PF7, CENTER PF8 (EXTI, active low with pull-up)
 ****************************************************************/
#include "stm32f4xx.h"
#include "SysClock180MHz.h"
#include "IRSensors.h"
#include "NeoPixel.h"
#include "OLED.h"
#include <stdint.h>
#include <stdio.h>

/* ---------------- hardware pins ---------------- */
#define LED_PORT        GPIOD
#define LED1_PIN        0U
#define LED2_PIN        1U
#define LED4_PIN        3U
#define LED5_PIN        4U
#define LED7_PIN        6U
#define LED8_PIN        7U

#define LEFT_PWM_PIN    12U   /* PD12, TIM4_CH1 */
#define RIGHT_PWM_PIN   14U   /* PD14, TIM4_CH3 */

#define BTN_UP_PIN      7U    /* PF7 */
#define BTN_CENTER_PIN  8U    /* PF8 */

/* ---------------- servo timing (Parallax Feedback 360) ---------------- */
#define PULSE_STOP      1500U  /* us, centre of the dead band */
#define PULSE_MIN_US    1280U  /* us, full speed clockwise */
#define PULSE_MAX_US    1720U  /* us, full speed counter-clockwise */

/* ---------------- line follower tuning ----------------
 * error_x100 is the line position in hundredths (-200 .. +200).
 * Both wheel speeds are kept >= 0 (forward only).  Because the IR
 * sensors sit on the wheel axis there is no look-ahead, so the gains
 * must be gentle: an aggressive turn rotates the sensors across the
 * line before the correction can settle, causing overshoot/weaving.
 * All gains are fixed point: value = (gain * error_x100) / 100.
 */
#define BASE_SPEED      45     /* us offset from stop when on line centre */
#define SPEED_DROP      25     /* us of speed removed at full error */
#define MIN_SPEED       25     /* us, keep enough speed to turn with */
#define KP              35     /* proportional gain */
#define KD              40     /* derivative gain (damping) */
#define KI              1      /* integral gain (small bias correction only) */
#define MAX_TURN        50     /* us, maximum differential between wheels */
#define TURN_SLEW       3      /* us, maximum change of turn per 5 ms tick */
#define MAX_INTEGRAL    800    /* anti-windup clamp on the integral */
#define INTEGRAL_LIMIT  30     /* only integrate when |error| < 0.30 spacing */
#define CONTROL_MS      10U     /* 5 ms control loop (200 Hz) */

/* ---------------- sharp corner handling ----------------
 * A 90 degree (or tighter) corner leaves the line at the edge of the
 * sensor array for a sustained time.  The robot then performs a
 * gentle powered turn: the outer wheel drives faster while the inner
 * wheel slows but keeps moving, so the robot arcs around the corner
 * without spinning or overshooting the new path.
 */
#define SHARP_ERROR     180    /* |error_x100| that counts as a hard corner */
#define SHARP_MS        150    /* how long it must persist */
#define RECOVER_ERROR   70     /* |error_x100| at which the turn ends */
#define CORNER_SPEED    22     /* us, forward speed during a corner turn */
#define CORNER_TURN     28     /* us, differential during a corner turn */
#define TURN_MIN_MS     120U   /* minimum turn time (ignore brief dropouts) */
#define TURN_TIMEOUT    900U   /* ms, give up a turn after this long */

/* ---------------- IR thresholds ----------------
 * Black reads higher than white.  Calibrate these for the room
 * lighting (see the IR sensor demo handout), then tune KP/KD.
 */
#define THRESH_IR1      1192
#define THRESH_IR2      1358
#define THRESH_IR3      1374
#define THRESH_IR4      1052
#define THRESH_IR5      1466

/* ---------------- crossing detector ----------------
 * A perpendicular (solid black) line is identified by BOTH outer
 * sensors, IR5 and IR1, seeing black at the same time.  On the
 * straight guide line only the inner sensors see black, so the
 * guide line itself is never mistaken for a crossing.
 * The first crossing is passed through; the second one stops the robot.
 */
#define CROSS_MIN_BLACK     4U     /* total black sensors at a crossing */
#define CROSS_REARM_MS      200U   /* sustained off-time before re-arming */
#define CROSS_DEBOUNCE_MS   800U   /* minimum gap between two crossings */
#define CROSS_START_MS      1000U  /* ignore crossings just after starting */
#define LINE_LOST_MS        2000U  /* stop if no sensor sees the line this long */

/* ---------------- run states ---------------- */
typedef enum {
    STATE_IDLE = 0,
    STATE_RUNNING,
    STATE_STOPPED
} run_state_t;

/* ---------------- globals ---------------- */
static volatile uint32_t ms_count = 0U;
static volatile uint8_t  control_tick = 0U;
static volatile uint8_t  display_tick = 0U;
static volatile uint8_t  btn_start = 0U;
static volatile uint8_t  btn_stop = 0U;

static volatile run_state_t run_state = STATE_IDLE;

static int32_t  error_x100 = 0;      /* line position error */
static int32_t  prev_error_x100 = 0;
static int32_t  integral = 0;        /* bounded integral (bias correction) */
static int32_t  last_turn = 0;       /* for the display */
static uint8_t  turn_active = 0U;
static uint8_t  sharp_timer_on = 0U;
static uint32_t sharp_since_ms = 0U;
static uint32_t turn_start_ms = 0U;
static uint16_t left_pulse = PULSE_STOP;
static uint16_t right_pulse = PULSE_STOP;

static uint8_t  crossing_seen = 0U;
static uint8_t  crossing_count = 0U;
static uint32_t last_cross_ms = 0U;
static uint32_t not_cross_ms = 0U;
static uint32_t run_start_ms = 0U;
static uint8_t  line_seen = 0U;
static uint32_t last_line_ms = 0U;

/* ---------------- prototypes ---------------- */
static void leds_init(void);
static void leds_update(void);
static void servo_pwm_init(void);
static void buttons_init(void);
static void motors_stop(void);
static void handle_buttons(void);
static int32_t compute_line_error(void);
static void update_corner_turn(uint32_t abs_error);
static void control_cycle(void);
static void display_update(void);

/* ---------------- interrupt handlers ---------------- */
void SysTick_Handler(void)
{
    static uint8_t control_div = 0U;
    static uint8_t display_div = 0U;

    ms_count++;

    if (++control_div >= CONTROL_MS) {
        control_div = 0U;
        control_tick = 1U;
    }
    if (++display_div >= 100U) {
        display_div = 0U;
        display_tick = 1U;
    }
}

void EXTI9_5_IRQHandler(void)
{
    const uint32_t mask = (1U << BTN_UP_PIN) | (1U << BTN_CENTER_PIN);
    uint32_t pr = EXTI->PR & mask;
    static uint32_t last_button_ms = 0U;

    EXTI->PR = pr;                       /* clear the pending bits */
    if (pr == 0U) {
        return;
    }

    /* simple 200 ms debounce */
    if ((last_button_ms != 0U) && ((uint32_t)(ms_count - last_button_ms) < 200U)) {
        return;
    }
    last_button_ms = ms_count;

    if (pr & (1U << BTN_UP_PIN)) {
        btn_start = 1U;
    }
    if (pr & (1U << BTN_CENTER_PIN)) {
        btn_stop = 1U;
    }
}

/* ---------------- initialisation ---------------- */
static void leds_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    LED_PORT->MODER &= ~(0xFFFFU << (LED1_PIN * 2U));
    LED_PORT->MODER |=  (0x5555U << (LED1_PIN * 2U)); /* PD0..PD7 = output */
    leds_update();
}

static void leds_update(void)
{
    /* Same mapping as the IR sensor demo. */
    if (IRSensor1 > THRESH_IR1) {
        LED_PORT->BSRR = (1U << LED1_PIN);
    } else {
        LED_PORT->BSRR = (1U << (LED1_PIN + 16U));
    }

    if (IRSensor2 > THRESH_IR2) {
        LED_PORT->BSRR = (1U << LED2_PIN);
    } else {
        LED_PORT->BSRR = (1U << (LED2_PIN + 16U));
    }

    if (IRSensor3 > THRESH_IR3) {
        LED_PORT->BSRR = (1U << LED4_PIN) | (1U << LED5_PIN);
    } else {
        LED_PORT->BSRR = (1U << (LED4_PIN + 16U)) | (1U << (LED5_PIN + 16U));
    }

    if (IRSensor4 > THRESH_IR4) {
        LED_PORT->BSRR = (1U << LED7_PIN);
    } else {
        LED_PORT->BSRR = (1U << (LED7_PIN + 16U));
    }

    if (IRSensor5 > THRESH_IR5) {
        LED_PORT->BSRR = (1U << LED8_PIN);
    } else {
        LED_PORT->BSRR = (1U << (LED8_PIN + 16U));
    }
}

static void servo_pwm_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* PD12 = TIM4_CH1 (AF2), PD14 = TIM4_CH3 (AF2) */
    GPIOD->MODER &= ~((3U << (LEFT_PWM_PIN * 2U)) |
                      (3U << (RIGHT_PWM_PIN * 2U)));
    GPIOD->MODER |=  (2U << (LEFT_PWM_PIN * 2U)) |
                     (2U << (RIGHT_PWM_PIN * 2U));
    GPIOD->AFR[1] &= ~((0xFU << ((LEFT_PWM_PIN - 8U) * 4U)) |
                       (0xFU << ((RIGHT_PWM_PIN - 8U) * 4U)));
    GPIOD->AFR[1] |=  (2U << ((LEFT_PWM_PIN - 8U) * 4U)) |
                      (2U << ((RIGHT_PWM_PIN - 8U) * 4U));

    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->PSC = 89U;                       /* 90 MHz / 90 = 1 MHz */
    TIM4->ARR = 19999U;                    /* 20 ms = 50 Hz */
    TIM4->CCR1 = PULSE_STOP;
    TIM4->CCR3 = PULSE_STOP;

    TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M);
    TIM4->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM4->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M);
    TIM4->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E;
    TIM4->CR1  |= TIM_CR1_ARPE;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->CR1  |= TIM_CR1_CEN;
}

static void buttons_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* PF7 and PF8 are inputs with internal pull-ups ( pressed = 0 ). */
    GPIOF->MODER &= ~((3U << (BTN_UP_PIN * 2U)) | (3U << (BTN_CENTER_PIN * 2U)));
    GPIOF->PUPDR &= ~((3U << (BTN_UP_PIN * 2U)) | (3U << (BTN_CENTER_PIN * 2U)));
    GPIOF->PUPDR |=  (1U << (BTN_UP_PIN * 2U)) | (1U << (BTN_CENTER_PIN * 2U));

    /* EXTI7 and EXTI8 both come from port F. */
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~(0xFU << 12U)) | (0x5U << 12U);
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~(0xFU << 0U))  | (0x5U << 0U);

    {
        const uint32_t mask = (1U << BTN_UP_PIN) | (1U << BTN_CENTER_PIN);
        EXTI->IMR  |= mask;
        EXTI->RTSR &= ~mask;
        EXTI->FTSR |= mask;
        EXTI->PR    = mask;
    }

    NVIC_SetPriority(EXTI9_5_IRQn, 2U);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* ---------------- control ---------------- */
static void motors_stop(void)
{
    TIM4->CCR1 = PULSE_STOP;
    TIM4->CCR3 = PULSE_STOP;
    left_pulse = PULSE_STOP;
    right_pulse = PULSE_STOP;
    last_turn = 0;
}

static void handle_buttons(void)
{
    if (btn_start) {
        btn_start = 0U;
        if (run_state != STATE_RUNNING) {
            error_x100 = 0;
            prev_error_x100 = 0;
            integral = 0;
            turn_active = 0U;
            sharp_timer_on = 0U;
            last_turn = 0;
            crossing_count = 0U;
            crossing_seen = 0U;
            last_cross_ms = 0U;
            not_cross_ms = ms_count;
            run_start_ms = ms_count;
            last_line_ms = ms_count;
            run_state = STATE_RUNNING;
        }
    }

    if (btn_stop) {
        btn_stop = 0U;
        run_state = STATE_STOPPED;
        motors_stop();
    }
}

/* Weighted average of the analog readings.  Only sensors that see
 * black contribute, so the white baseline does not bias the result.
 * Returns the line position in hundredths of a sensor spacing:
 * -200 = far left, 0 = centre, +200 = far right.
 */
static int32_t compute_line_error(void)
{
    static const int32_t weight[5] = { -2, -1, 0, 1, 2 }; /* IR5 .. IR1 */
    const int32_t reading[5] = {
        (int32_t)IRSensor5, (int32_t)IRSensor4, (int32_t)IRSensor3,
        (int32_t)IRSensor2, (int32_t)IRSensor1
    };
    const int32_t thresh[5] = {
        THRESH_IR5, THRESH_IR4, THRESH_IR3, THRESH_IR2, THRESH_IR1
    };
    int32_t numerator = 0;
    int32_t denominator = 0;

    for (uint32_t i = 0U; i < 5U; i++) {
        int32_t blackness = reading[i] - thresh[i];
        if (blackness > 0) {
            numerator += weight[i] * blackness;
            denominator += blackness;
        }
    }

    if (denominator == 0) {
        line_seen = 0U;
        return 0;                 /* line lost: drive straight (briefly) */
    }
    line_seen = 1U;
    return (numerator * 100) / denominator;
}

static uint32_t black_sensor_count(void)
{
    uint32_t n = 0U;
    n += (IRSensor1 > THRESH_IR1) ? 1U : 0U;
    n += (IRSensor2 > THRESH_IR2) ? 1U : 0U;
    n += (IRSensor3 > THRESH_IR3) ? 1U : 0U;
    n += (IRSensor4 > THRESH_IR4) ? 1U : 0U;
    n += (IRSensor5 > THRESH_IR5) ? 1U : 0U;
    return n;
}

/* A perpendicular line spans the whole sensor array, so the two
 * outside sensors must be on black; the normal guide line should not
 * reach them. */
static uint8_t crossing_detected(void)
{
    uint8_t outer = (IRSensor5 > THRESH_IR5) && (IRSensor1 > THRESH_IR1);
    return (outer && (black_sensor_count() >= CROSS_MIN_BLACK)) ? 1U : 0U;
}

static void check_crossing(void)
{
    if (crossing_detected()) {
        not_cross_ms = ms_count;

        if (!crossing_seen &&
            ((uint32_t)(ms_count - run_start_ms) >= CROSS_START_MS) &&
            ((uint32_t)(ms_count - last_cross_ms) >= CROSS_DEBOUNCE_MS)) {

            crossing_seen = 1U;
            last_cross_ms = ms_count;
            crossing_count++;

            if (crossing_count == 1U) {
                /* First solid black line: drive straight through it. */
                error_x100 = 0;
                prev_error_x100 = 0;
            } else if (crossing_count >= 2U) {
                /* Second solid black line: this is the end line. */
                run_state = STATE_STOPPED;
                motors_stop();
            }
        }
    } else if (crossing_seen &&
               ((uint32_t)(ms_count - not_cross_ms) >= CROSS_REARM_MS)) {
        crossing_seen = 0U;
    }
}

/* Detect a sustained large error (hard corner) and run a powered turn
 * until the line is centred again.  Both wheels keep driving forward
 * so the sensor array moves onto the next segment of the line. */
static void update_corner_turn(uint32_t abs_error)
{
    if (!turn_active) {
        if (abs_error >= SHARP_ERROR) {
            if (!sharp_timer_on) {
                sharp_since_ms = ms_count;
                sharp_timer_on = 1U;
            } else if ((uint32_t)(ms_count - sharp_since_ms) >= SHARP_MS) {
                turn_active = 1U;
                turn_start_ms = ms_count;
                sharp_timer_on = 0U;
                integral = 0;
            }
        } else {
            sharp_timer_on = 0U;
        }
    } else {
        uint32_t elapsed = (uint32_t)(ms_count - turn_start_ms);
        if ((elapsed >= TURN_MIN_MS) &&
            ((line_seen && (abs_error <= RECOVER_ERROR)) ||
             (elapsed >= TURN_TIMEOUT))) {
            turn_active = 0U;
            sharp_timer_on = 0U;
            prev_error_x100 = error_x100;
            integral = 0;
        }
    }
}

static void control_cycle(void)
{
    Read_ADC3_Channels();
    leds_update();

    if (run_state != STATE_RUNNING) {
        return;
    }

    /* ---- proportional + derivative on the analog line position ---- */
    error_x100 = compute_line_error();

    /* Fail-safe: stop if the robot has lost the line on a white surface. */
    if (line_seen) {
        last_line_ms = ms_count;
    } else if ((uint32_t)(ms_count - last_line_ms) >= LINE_LOST_MS) {
        run_state = STATE_STOPPED;
        motors_stop();
        return;
    }

    {
        uint32_t abs_error = (error_x100 < 0) ?
                             (uint32_t)-error_x100 : (uint32_t)error_x100;
        int32_t derivative;
        int32_t turn;
        int32_t speed;

        update_corner_turn(abs_error);

        /* The integral is only for small, persistent errors (e.g. one
         * motor is slower).  It is frozen and bled off during corners
         * so it cannot wind up and overshoot on corner exit. */
        if (!turn_active) {
            if (abs_error <= INTEGRAL_LIMIT) {
                integral += error_x100;
                if (integral > MAX_INTEGRAL) {
                    integral = MAX_INTEGRAL;
                } else if (integral < -MAX_INTEGRAL) {
                    integral = -MAX_INTEGRAL;
                }
            } else {
                integral /= 2;
            }
        }

        if (turn_active) {
            turn = (error_x100 >= 0) ? CORNER_TURN : -CORNER_TURN;
            speed = CORNER_SPEED;
        } else {
            derivative = error_x100 - prev_error_x100;
            turn = (KP * error_x100 + KD * derivative + KI * integral) / 100;
            if (turn > MAX_TURN) {
                turn = MAX_TURN;
            } else if (turn < -MAX_TURN) {
                turn = -MAX_TURN;
            }
            speed = BASE_SPEED - (SPEED_DROP * (int32_t)abs_error) / 100;
            if (speed < MIN_SPEED) {
                speed = MIN_SPEED;
            }
        }

        prev_error_x100 = error_x100;
        last_turn = turn;

        /* left forward = pulse above 1500, right forward = below 1500 */
        {
            int32_t l = (int32_t)PULSE_STOP + speed + turn;
            int32_t r = (int32_t)PULSE_STOP - speed + turn;

            if (l < (int32_t)PULSE_MIN_US) l = (int32_t)PULSE_MIN_US;
            if (l > (int32_t)PULSE_MAX_US) l = (int32_t)PULSE_MAX_US;
            if (r < (int32_t)PULSE_MIN_US) r = (int32_t)PULSE_MIN_US;
            if (r > (int32_t)PULSE_MAX_US) r = (int32_t)PULSE_MAX_US;

            left_pulse = (uint16_t)l;
            right_pulse = (uint16_t)r;
            TIM4->CCR1 = left_pulse;
            TIM4->CCR3 = right_pulse;
        }
    }

    check_crossing();
}

/* ---------------- display ---------------- */
static void display_update(void)
{
    char line[24];

    OLED_Clear();
    OLED_Print(0, 0, "CPEG222 Project 10");

    if (run_state == STATE_RUNNING) {
        OLED_Print(1, 0, "State: RUNNING");
    } else if (run_state == STATE_STOPPED) {
        OLED_Print(1, 0, "State: STOPPED");
    } else {
        OLED_Print(1, 0, "State: IDLE");
    }

    /* left (IR5), centre (IR3) and right (IR1) raw counts */
    snprintf(line, sizeof(line), "L%4u C%4u R%4u",
             (unsigned)IRSensor5, (unsigned)IRSensor3, (unsigned)IRSensor1);
    OLED_Print(2, 0, line);

    snprintf(line, sizeof(line), "Err %+4ld Trn %+4ld",
             (long)(error_x100 / 100), (long)last_turn);
    OLED_Print(3, 0, line);

    snprintf(line, sizeof(line), "PWM %4u / %4u",
             (unsigned)left_pulse, (unsigned)right_pulse);
    OLED_Print(4, 0, line);

    snprintf(line, sizeof(line), "Line %u/2 (2=Stop)", (unsigned)crossing_count);
    OLED_Print(5, 0, line);

    if (turn_active) {
        OLED_Print(6, 0, ">> CORNER PIVOT <<");
    }

    OLED_Print(7, 0, "UP=Go CENTER=Stop");
}

/* ---------------- main ---------------- */
int main(void)
{
    SystemClock_Config();
    SystemCoreClockUpdate();

    leds_init();
    ADC3_Init_Specific_Pins();

    /* Keep the NeoPixels dark (they are very bright if left on). */
    NeoPixel_Init_PB11();
    NeoPixel_SetColor(0, 0, 0, 0);
    NeoPixel_SetColor(1, 0, 0, 0);
    NeoPixel_SetColor(2, 0, 0, 0);
    NeoPixel_SetColor(3, 0, 0, 0);
    NeoPixel_Show_PB11();

    servo_pwm_init();
    buttons_init();

    OLED_Init();
    OLED_Print(0, 0, "CPEG222 Project 10");
    OLED_Print(7, 0, "UP=Go CENTER=Stop");

    if (SysTick_Config(180000U) != 0U) {   /* 1 ms tick at 180 MHz */
        while (1) {
        }
    }
    NVIC_SetPriority(SysTick_IRQn, 15U);

    while (1) {
        handle_buttons();

        if (control_tick) {
            control_tick = 0U;
            control_cycle();
        }
        if (display_tick) {
            display_tick = 0U;
            display_update();
        }
    }
}
