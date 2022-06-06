#include <math.h>
#include <vector>
#include <string.h>
// #include <cstdlib>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/divider.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"

#include <tusb.h>
#include "pico/unique_id.h"

#include "pid.h"
// #include "pio_quad_encoder.pio.h"
#include "pio_quad_encoder.h"

////////////////////////////////////////
// Define Code Switches
////////////////////////////////////////
#define WAIT_USB            1       // set to 0 disable waiting for USB
#define UART_DEBUG          1

////////////////////////////////////////
// Define Hardware Pins
////////////////////////////////////////
#define PWM_OUT_ONE_PIN         2
#define PWM_OUT_TWO_PIN         3
#define PWM_EN_PIN              4
#define BEAM_BREAK_PIN          5
#define GREEN_RGB_LED_PIN       6
#define BLUE_RGB_LED_PIN        7
#define RED_RGB_LED_PIN         8
#define ENC_ONE_PIN             9
#define ENC_TWO_PIN             10
#define BNC_INPUT_PIN           11
#define PELLET_DELIVERED_PIN    21
#define GREEN_LED_PIN           25

////////////////////////////////////////
// Define SET Software Values
////////////////////////////////////////
#define GPIO_ON             1
#define GPIO_OFF            0

#define RGB_ON              0
#define RGB_OFF             1

#define LEVEL_LOW           0x1
#define LEVEL_HIGH          0x2
#define EDGE_FALL           0x4
#define EDGE_RISE           0x8

// for 24 hole feeder disk
#define TIME_DIF_MAX        1000u
#define MAX_ROTATION_TOTAL  23104u
#define MAX_ROTATION_HOLE   963u    // 1925u basically MAX_ROTATION_TOTAL / 12 (number of holes) --- for 24 holes 963u
#define ROTATION_PRE_LOAD   800u   // distance to travel around before stopping from the last hole to pre load a pellet was 1500u for 12 hole and 500u for 24 (initially)
#define ROTATION_OVERSHOOT  250u    // How far to overshoot the front of the hole - gives the pellet more time to drop before a false negative is picked up.
#define SPEED_MAX           400u
#define SPEED_PRE_LOAD      300u
#define SPEED_DELIVER       400u
#define POSITION_MARGIN     15u


// #define SPEED_MAX           100u
// #define SPEED_PRE_LOAD      50u
// #define SPEED_DELIVER       100u

#define USB_WAIT_TIME           100u     // 10 seconds (100 * 0.1ms)
#define UART_STRING_BUF_SIZE    40

#define MULTICORE_FIFO_TIMEOUT  1000u

#define BNC_TRIGGER_TIME_LIMIT      20u     // upper limit of 20 ms (more likely 1ms)
#define BNC_INITIALISE_TIME_LIMIT   25u     // low limit of 25ms

#define MAX_INITIALISE_COUNT    6u

////////////////////////////////////////
// Define Feeder Application States
////////////////////////////////////////

#define FEEDER_INITIALISE           0x01
#define FEEDER_PRE_LOAD             0x02
#define FEEDER_DELIVERED            0x04
#define FEEDER_DELIVERY_ERROR       0x08

////////////////////////////////////////
// Define Display Update Types
////////////////////////////////////////

#define DISPLAY_MESSAGE             0x01
#define DISPLAY_STATUS              0x02
#define DISPLAY_COUNTS              0x04

////////////////////////////////////////
// Velocity Controller parameters
////////////////////////////////////////
#define PID_VEL_KP  10.0f
#define PID_VEL_KI  20.0f
#define PID_VEL_KD  0.002f

#define PID_VEL_TAU 0.200f

#define PID_VEL_LIM_MIN -4095.0f
#define PID_VEL_LIM_MAX  4095.0f

#define PID_VEL_LIM_MIN_INT -4095.0f
#define PID_VEL_LIM_MAX_INT  4095.0f

#define SAMPLE_TIME_VEL_S 0.002f

////////////////////////////////////////
// Position Controller parameters
////////////////////////////////////////
#define PID_POS_KP  0.3f
#define PID_POS_KI  0.08f
#define PID_POS_KD  0.005f

#define PID_POS_TAU 0.02f

#define PID_POS_LIM_MIN -400.0f
#define PID_POS_LIM_MAX  400.0f

#define PID_POS_LIM_MIN_INT -25.0f
#define PID_POS_LIM_MAX_INT  25.0f

#define SAMPLE_TIME_POS_S 0.002f

////////////////////////////////////////
// Define classes / structs / ints etc.
////////////////////////////////////////

QuadEncoder quad_encoder(ENC_ONE_PIN);

PIDController pid_vel = { PID_VEL_KP, PID_VEL_KI, PID_VEL_KD,
                        PID_VEL_TAU,
                        PID_VEL_LIM_MIN, PID_VEL_LIM_MAX,
                        PID_VEL_LIM_MIN_INT, PID_VEL_LIM_MAX_INT,
                        SAMPLE_TIME_VEL_S };

PIDController pid_pos = { PID_POS_KP, PID_POS_KI, PID_POS_KD,
                        PID_POS_TAU,
                        PID_POS_LIM_MIN, PID_POS_LIM_MAX,
                        PID_POS_LIM_MIN_INT, PID_POS_LIM_MAX_INT,
                        SAMPLE_TIME_POS_S };

struct repeating_timer control_loop;

// Debounce control
// unsigned long time = to_ms_since_boot(get_absolute_time());
// const int delayTime =  250;
// block extra pellet triggers per shot - to fix pellet count
// unsigned long pellet_time = to_ms_since_boot(get_absolute_time());

unsigned long bnc_trigger_rise_time;
unsigned long bnc_trigger_fall_time;
// const int pellet_delay_time =  50;         // was 250ms but was missing pellets when fine tuning speed of delivery

volatile int32_t motor_position = 0;
volatile int32_t motor_position_buf;
volatile int32_t motor_position_old = 0;
volatile int32_t position_setpoint = 0;   // Feeder Position setpoint
volatile int32_t motor_speed = 0;           // Actual measued speed from the motor
volatile int32_t motor_speed_buf; 
volatile int32_t speed_setpoint = 0;   // Motor Speed setpoint
volatile int32_t speed_pid_out = 0;   // Motor Speed setpoint
volatile int32_t speed_limit = 0;   // Motor Speed setpoint
uint16_t set_pwm_one = 0;               // PWM values for writing to Timer
uint16_t set_pwm_two = 0;               // PWM values for writing to Timer
static bool motor_moving;             // Flag to update screens etc.
static bool feeder_position_reached;  // flag to set when the feeder has reached the position setpoint (within a margin set by POSITION_MARGIN)

static bool pellet_delivered = false;         // Flag set when pellet gets delivered
static bool pellet_delivered_led = false;
volatile int32_t pellet_delivered_count_1 = 0;
volatile int32_t pellet_delivered_count_2 = 0;
volatile int32_t pellet_delivered_count_3 = 0;
volatile int32_t pellet_delivered_count_4 = 0;
volatile int32_t pellet_delivered_count_5 = 0;
volatile int32_t pellet_delivered_count_6 = 0;
volatile int32_t pellet_delivered_msg_count = 0;
static bool missed_pellet;
volatile bool uart_message_flag;
volatile bool motor_brake;
volatile bool bnc_triggered;
volatile bool led_toggle;
volatile bool h_bridge_start;

volatile bool A_flag;
volatile bool B_flag;
volatile bool X_flag;
volatile bool Y_flag;

volatile uint32_t application_status = 0;
volatile uint32_t application_flags = 0;

divmod_result_t delta_time_hw;
divmod_result_t motor_speed_hw;

// static char event_str[128];

static char uart_message_str[UART_STRING_BUF_SIZE];

// Define FreeRTOS Task
void vSerialDebugTask( void * pvParameters );
void vApplicationTask( void * pvParameters );
void vSystemMonitorTask( void * pvParameters );

bool feeder_initialise(int attempt);
bool feeder_pre_load_pellet(void);
bool feeder_deliver_pellet(void);
bool feeder_deliver_error(void);

// void gpio_event_string(char *buf, uint32_t events);

////////////////////////////////////////
// Interupt Callback Routines - START
////////////////////////////////////////
// CORE 1 - START
////////////////////////////////////////
// GPIO Interupt
void gpio_callback_core_1(uint gpio, uint32_t events) {
    // irq_clear(IO_IRQ_BANK0);
}

////////////////////////////////////////
// CORE 1 - END
////////////////////////////////////////

////////////////////////////////////////
// CORE 0 - START
////////////////////////////////////////
// Alarm callbacks

int64_t green_led_callback(alarm_id_t id, void *user_data) {
    gpio_put(GREEN_RGB_LED_PIN, RGB_OFF);
    gpio_put(RED_RGB_LED_PIN, RGB_ON);
    return 0;
}

int64_t amber_led_callback(alarm_id_t id, void *user_data) {
    gpio_put(GREEN_LED_PIN, GPIO_OFF);
    if (pellet_delivered_led) {
        add_alarm_in_ms(200, green_led_callback, NULL, false);
        gpio_put(RED_RGB_LED_PIN, RGB_OFF);
        pellet_delivered_led = false;
    } else {
        gpio_put(GREEN_RGB_LED_PIN, RGB_OFF);
    }
    return 0;
}
// GPIO Interupt
void gpio_callback_core_0(uint gpio, uint32_t events) {

    switch(gpio) {
        case BEAM_BREAK_PIN:
            switch (events) {   
                case EDGE_RISE:
                    gpio_put(PELLET_DELIVERED_PIN, GPIO_ON);
                    pellet_delivered = true;
                    pellet_delivered_led = true;
                    break;          
                case EDGE_FALL:
                    gpio_put(PELLET_DELIVERED_PIN, GPIO_OFF);
                    break;
                default:
                    break;
            }
            break;
        case BNC_INPUT_PIN:
            switch (events) {
                case EDGE_FALL:      // Lab Unit
                // case EDGE_RISE:         // Bench Prototype
                    bnc_trigger_fall_time = to_ms_since_boot(get_absolute_time());
                    // gpio_put(GREEN_LED_PIN, GPIO_ON);
                    break;
                case EDGE_RISE:      // Lab Unit
                // case EDGE_FALL:         // Bench Prototype
                    bnc_trigger_rise_time = to_ms_since_boot(get_absolute_time());
                    // gpio_put(GREEN_LED_PIN, GPIO_OFF);
                    if ((bnc_trigger_rise_time - bnc_trigger_fall_time) > BNC_INITIALISE_TIME_LIMIT) {
                        application_status = FEEDER_INITIALISE;
                        application_flags = 0;
                        A_flag = true;
                        // gpio_put(GREEN_LED_PIN, GPIO_ON);
                    } else if ((bnc_trigger_rise_time - bnc_trigger_fall_time) < BNC_TRIGGER_TIME_LIMIT) {
                        bnc_triggered = true;
                        add_alarm_in_ms(200, amber_led_callback, NULL, false);
                        gpio_put(GREEN_RGB_LED_PIN, RGB_ON);
                        gpio_put(GREEN_LED_PIN, GPIO_ON);
                    }
                    break;
                default:
                    break;
            }           
            break;
        default:
            break;
    }

}
// Timer Callback Interupt for 2ms Control Loop.
bool repeating_timer_callback(struct repeating_timer *t) {
    
    static uint32_t r_d;                        // Change in position
    static uint64_t t_d;                        // Change in time

    // motor_position = quad_encoder.get_rotation();
    motor_position = quad_encoder.get_rotation();
    r_d = motor_position - motor_position_old;
    // r_d = motor_position_old - motor_position;
    r_d = r_d<<16;                              // shifting the quad encoder step counts to facilitate the division without floating point calcs

    if (motor_position != motor_position_old) {
        t_d = quad_encoder.get_time_dif();
        quad_encoder.clear_time_dif();
        motor_speed_hw = hw_divider_divmod_s32(r_d,t_d);        // Hardware division
        motor_speed = to_quotient_s32(motor_speed_hw);          // Hardware division
        motor_moving = true;
    } else {
        motor_moving = false;
    }

    if (abs(position_setpoint - motor_position) > POSITION_MARGIN) {
        feeder_position_reached = false;
    } else {
        feeder_position_reached = true;
    }

    speed_setpoint = PIDController_Update(&pid_pos, position_setpoint, motor_position);
    motor_position_old = motor_position;
    
    if (speed_setpoint > speed_limit) speed_setpoint = speed_limit;
    if (speed_setpoint < (0 - speed_limit)) speed_setpoint = 0 - speed_limit;

    if (speed_setpoint != 0){      
        motor_brake = false;
        speed_pid_out = PIDController_Update(&pid_vel, speed_setpoint, motor_speed);
    } else {
        motor_brake = true;
        PIDController_Init(&pid_vel);
    }
    
    return true;
}
// PWM Wrap Interupt
void on_pwm_wrap() {

    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT_ONE_PIN));
    if (h_bridge_start) {      
        if (motor_brake) {
            set_pwm_two = 4095;
            set_pwm_one = 4095;
        } else {
            if (speed_pid_out == 0) {
                set_pwm_two = 0;
                set_pwm_one = 0;
            } else if (speed_pid_out > 0) {
                set_pwm_one = speed_pid_out;
                if (set_pwm_one > 4095) set_pwm_one = 4095;
                set_pwm_two = 0;
            } else {
                set_pwm_two = speed_pid_out*-1;
                if (set_pwm_two > 4095) set_pwm_two = 4095;
                set_pwm_one = 0;
            }
        }
    } else {
        set_pwm_two = 0;
        set_pwm_one = 0;
    }


    pwm_set_gpio_level(PWM_OUT_ONE_PIN, set_pwm_one);
    pwm_set_gpio_level(PWM_OUT_TWO_PIN, set_pwm_two);
    // pwm_set_gpio_level(PWM_OUT_ONE_PIN, set_pwm_two);
    // pwm_set_gpio_level(PWM_OUT_TWO_PIN, set_pwm_one);
}

////////////////////////////////////////
// CORE 0 - END
////////////////////////////////////////
// Interupt Callback Routines - END
////////////////////////////////////////

////////////////////////////////////////
// SWC Base code - will run on Core 1 - START
////////////////////////////////////////
// void swc_base() {

// }

////////////////////////////////////////
// SWC Base code - will run on Core 1 - END
////////////////////////////////////////

////////////////////////////////////////
// Main code - will run on Core 0 - START
////////////////////////////////////////
int main() 
{
    u_int16_t usb_timeout_count = 0;

    pico_unique_board_id_t id_out;
    // multicore_launch_core1(swc_base);
    stdio_init_all();

#if WAIT_USB
    while (!tud_cdc_connected() && usb_timeout_count < USB_WAIT_TIME)
    {
        usb_timeout_count++;
        sleep_ms(100); 
    }
#endif

    quad_encoder.set_rotation(0);

    pico_get_unique_board_id(&id_out);

    sprintf(uart_message_str, "SWC Underground Feeder\nBoard ID = %llx\n", *((uint64_t*)(id_out.id)));
    uart_message_flag = true;

    gpio_init(BEAM_BREAK_PIN);
    gpio_set_dir(BEAM_BREAK_PIN, GPIO_IN);
    gpio_init(BNC_INPUT_PIN);
    gpio_set_dir(BNC_INPUT_PIN, GPIO_IN);
    gpio_pull_up(BNC_INPUT_PIN); // original config
    // gpio_pull_down(BNC_INPUT_PIN);

// BEAM_BREAK_PIN 
    gpio_set_irq_enabled_with_callback(BEAM_BREAK_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_0);
    gpio_set_irq_enabled_with_callback(BNC_INPUT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_0);

    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);
    
    gpio_init(GREEN_RGB_LED_PIN);
    gpio_set_dir(GREEN_RGB_LED_PIN, GPIO_OUT);
    gpio_init(BLUE_RGB_LED_PIN);
    gpio_set_dir(BLUE_RGB_LED_PIN, GPIO_OUT);
    gpio_init(RED_RGB_LED_PIN);
    gpio_set_dir(RED_RGB_LED_PIN, GPIO_OUT);

    gpio_init(PELLET_DELIVERED_PIN);
    gpio_set_dir(PELLET_DELIVERED_PIN, GPIO_OUT);

    gpio_init(PWM_EN_PIN);
    gpio_set_dir(PWM_EN_PIN, GPIO_OUT);

    gpio_set_function(PWM_OUT_ONE_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT_TWO_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(PWM_OUT_ONE_PIN); 

    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 4096);
    pwm_init(slice_num, &config, true);

    gpio_put(PWM_EN_PIN, GPIO_OFF);
    gpio_put(PELLET_DELIVERED_PIN, GPIO_OFF);
    gpio_put(GREEN_LED_PIN, GPIO_OFF);
    gpio_put(RED_RGB_LED_PIN, RGB_ON);
    gpio_put(GREEN_RGB_LED_PIN, RGB_OFF);
    gpio_put(BLUE_RGB_LED_PIN, RGB_OFF);

    BaseType_t status;
#if UART_DEBUG
    TaskHandle_t xSerialDebugHandle = NULL;
#endif
    TaskHandle_t xApplicationHandle = NULL;
    TaskHandle_t xSystemMonitorTask = NULL;

#if UART_DEBUG
    status = xTaskCreate(
                  vSerialDebugTask,               // Task Function
                  "Serial Debug Task Task",       // Task Name
                  256,                            // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY,               // Task Priority
                  &xSerialDebugHandle );   
#endif

    status = xTaskCreate(
                  vApplicationTask,               // Task Function
                  "Application Task",             // Task Name
                  1024,                           // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY + 3,           // Task Priority
                  &xApplicationHandle );  

    status = xTaskCreate(
                  vSystemMonitorTask,              // Task Function
                  "System Monitor Task",           // Task Name
                  256,                            // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY + 1,           // Task Priority
                  &xSystemMonitorTask ); 

    PIDController_Init(&pid_vel);
    PIDController_Init(&pid_pos);

    add_repeating_timer_us(-2000, repeating_timer_callback, NULL, &control_loop);
    speed_limit = 0;

    application_status = application_status | FEEDER_INITIALISE;
    application_flags = 0;

    vTaskStartScheduler();

    while(true) {
        //
    }
}
////////////////////////////////////////
// Main code - will run on Core 0 - END
////////////////////////////////////////

////////////////////////////////////////
// FreeRTOS Tasks - will run on Core 0 - START
////////////////////////////////////////
#if UART_DEBUG
void vSerialDebugTask( void * pvParameters )
{
    for( ;; )
    {
        if (uart_message_flag){
            printf(uart_message_str);
            uart_message_flag = false;
        }
        vTaskDelay(1);
    }
}
#endif

void vApplicationTask( void * pvParameters )
{
    bool status;
    int i;
    uint8_t delay_count;

    while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
    sprintf(uart_message_str, "LONG PULSE TO INITIALISE\n");
    uart_message_flag = true;

    for( ;; )
    {    
        switch (application_status) {
            case FEEDER_INITIALISE:
                if (A_flag && ((application_flags & FEEDER_INITIALISE) != FEEDER_INITIALISE)) {
                    while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
                    sprintf(uart_message_str, "Starting to INITIALISE\n");
                    uart_message_flag = true;

                    gpio_put(PWM_EN_PIN, GPIO_ON);
                    vTaskDelay(10);

                    speed_limit = 0;
                    quad_encoder.set_rotation(0);
                    position_setpoint = 0;
                    PIDController_Init(&pid_pos);
                    PIDController_Init(&pid_vel);
                    vTaskDelay(5);
                    h_bridge_start = true;


                    i = 1;
                    status = feeder_initialise(i);
                    // i = 0;
                    while (!status) {
                        i++;
                        status = feeder_initialise(i);
                        // i++;
                        if (i > MAX_INITIALISE_COUNT) break;
                    }
                    if (status) {
                        application_flags = application_flags | FEEDER_INITIALISE;
                        application_status = application_status & ~FEEDER_INITIALISE;
                        application_status = application_status | FEEDER_PRE_LOAD;
                        pellet_delivered_count_1 = 0;
                        pellet_delivered_count_2 = 0;
                        pellet_delivered_count_3 = 0;
                        pellet_delivered_count_4 = 0;
                        pellet_delivered_count_5 = 0;
                        pellet_delivered_count_6 = 0;
                        while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
                        sprintf(uart_message_str, "FEEDER INITIALISED\n");
                        uart_message_flag = true;
                        i = 1;  // going to the delivery stage, i = 1 for 1st pellet delivery attempt
                    } else {
                        application_flags = application_flags & ~FEEDER_INITIALISE;
                        while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
                        sprintf(uart_message_str, "FEEDER INITIALISE ERROR\n");
                        uart_message_flag = true;
                    }
                    A_flag = false;
                    
                }
                break;
            case FEEDER_PRE_LOAD:
                if ((application_flags & FEEDER_PRE_LOAD) != FEEDER_PRE_LOAD) {
                    status = feeder_pre_load_pellet();
                    if (status) {
                        application_flags = application_flags | FEEDER_PRE_LOAD;
                        application_status = application_status & ~FEEDER_PRE_LOAD;
                        application_status = application_status | FEEDER_DELIVERED;
                        application_flags = application_flags & ~FEEDER_DELIVERED;
                        // gpio_put(PELLET_DELIVERED_PIN, GPIO_OFF);   // Pin goes Low here after being high from pellet delivered
                        // above handled by GPIO interupt for pulse width
                    } else {
                        // there is no case that it should get here
                        application_flags = application_flags & ~FEEDER_PRE_LOAD;
                    }
                    break;
                }
            case FEEDER_DELIVERED:
                // if ((A_flag || B_flag || bnc_triggered) && ((application_flags & FEEDER_DELIVERED) != FEEDER_DELIVERED)) {
                if ((bnc_triggered) && ((application_flags & FEEDER_DELIVERED) != FEEDER_DELIVERED)) {
                    status = feeder_deliver_pellet();
                    if (status) {
                        application_flags = application_flags | FEEDER_DELIVERED;
                        application_status = application_status & ~FEEDER_DELIVERED;
                        application_status = application_status | FEEDER_PRE_LOAD;
                        application_flags = application_flags & ~FEEDER_PRE_LOAD;
                        // gpio_put(PELLET_DELIVERED_PIN, GPIO_ON);    // Pin goes high here and low after feeder pre-load complete
                        // Above done by interrupt now

                        switch(i) {
                            case 1:
                                pellet_delivered_count_1++;
                                break;
                            case 2:
                                pellet_delivered_count_2++;
                                break;
                            case 3:
                                pellet_delivered_count_3++;
                                break;
                            case 4:
                                pellet_delivered_count_4++;
                                break;
                            case 5:
                                pellet_delivered_count_5++;
                                break;
                            case 6:
                                pellet_delivered_count_6++;
                                break;
                            default:
                                break;
                        }
                        while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
                        sprintf(uart_message_str, "1:%d  2:%d  3: %d  4:%d  5:%d  6:%d\n", pellet_delivered_count_1, pellet_delivered_count_2, pellet_delivered_count_3, pellet_delivered_count_4, pellet_delivered_count_5, pellet_delivered_count_6);
                        uart_message_flag = true;
                        i = 1;
                        // A_flag = false;
                        bnc_triggered = false;
                    } else {
                        if (i >= 6) {
                            application_flags = application_flags & ~FEEDER_PRE_LOAD;
                            application_flags = application_flags & ~FEEDER_DELIVERED;
                            application_status = application_status & ~FEEDER_DELIVERED;
                            application_status = application_status | FEEDER_DELIVERY_ERROR;
                            while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clearR
                            sprintf(uart_message_str, "Delivery %d Failed\nDelivery ERROR!!\n",i);
                            uart_message_flag = true;
                            vTaskDelay(10);
                            // A_flag = false;
                            bnc_triggered = false;
                        } else {
                            application_status = application_status & ~FEEDER_DELIVERED;
                            application_status = application_status | FEEDER_PRE_LOAD;
                            application_flags = application_flags & ~FEEDER_PRE_LOAD;
                            while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
                            sprintf(uart_message_str, "Delivery %d Failed\n",i);
                            uart_message_flag = true;
                            // A_flag = false;
                            bnc_triggered = false;
                        }
                        i++;
                    }    
                }
                break;
            case FEEDER_DELIVERY_ERROR:
                feeder_deliver_error();
                if (Y_flag) {
                    application_flags = application_flags & ~FEEDER_INITIALISE;
                    application_status = application_status & ~FEEDER_DELIVERY_ERROR;
                    application_status = application_status | FEEDER_INITIALISE;
                    Y_flag = false;
                }
                break;
            default:
                
                break;            
        }
        delay_count = 0;
        while (!bnc_triggered && !A_flag && (delay_count <= 20)) {
            delay_count++;
            vTaskDelay(1);
        }
        // vTaskDelay(20);    // was 200
    }
}
 
void vSystemMonitorTask( void * pvParameters )
{
    for( ;; )
    {
        // not being used yet
        vTaskDelay(2000);
    }
}

////////////////////////////////////////
// FreeRTOS Tasks - will run on Core 0 - END
////////////////////////////////////////


bool feeder_initialise(int attempt) {
    while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
    // sprintf(uart_message_str, "INITIALISING %d\n", attempt);
    sprintf(uart_message_str, "INITIALISING attempt %d\n", attempt);
    uart_message_flag = true;
    vTaskDelay(10);

    speed_limit = SPEED_DELIVER;
    position_setpoint = position_setpoint + MAX_ROTATION_HOLE;
    feeder_position_reached = false;
    pellet_delivered = false;

    while (!feeder_position_reached && !pellet_delivered) {
        vTaskDelay(1);
    }
    
    if (pellet_delivered) {
        speed_limit = 0;
        quad_encoder.set_rotation(0);
        position_setpoint = 0;
        PIDController_Init(&pid_pos);
        PIDController_Init(&pid_vel);
        vTaskDelay(5);
        pellet_delivered = false;
        speed_limit = SPEED_PRE_LOAD;
        return true;
    } else {
        return false;
    }
}

bool feeder_pre_load_pellet() {

    while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
    sprintf(uart_message_str, "Pre Load Pellet\n");
    uart_message_flag = true;

    speed_limit = SPEED_PRE_LOAD;
    position_setpoint = position_setpoint + ROTATION_PRE_LOAD;
    feeder_position_reached = false;
    pellet_delivered = false;

    while (!feeder_position_reached) {
        vTaskDelay(1);
    }

    speed_limit = 0;

    if (feeder_position_reached) {
        return true;
    } else {
        return false;
    }
}

bool feeder_deliver_pellet() {

    
    while (uart_message_flag) { vTaskDelay(1);}     //wait for previous uart message to clear
    sprintf(uart_message_str, "Delivering Pellet\n");
    uart_message_flag = true;

    speed_limit = SPEED_DELIVER;
    position_setpoint = position_setpoint + MAX_ROTATION_HOLE - ROTATION_PRE_LOAD + ROTATION_OVERSHOOT;
    feeder_position_reached = false;
    pellet_delivered = false;

    while (!feeder_position_reached && !pellet_delivered) {
        vTaskDelay(2);
    }

    speed_limit = SPEED_MAX;
    position_setpoint = position_setpoint - ROTATION_OVERSHOOT; // to remove the overshoot from making the position overrun carry over - also acts as a brake

    vTaskDelay(10);

    if (pellet_delivered) {
        speed_limit = 0;
        quad_encoder.set_rotation(0);
        position_setpoint = 0;
        PIDController_Init(&pid_pos);
        PIDController_Init(&pid_vel);
        // vTaskDelay(5);
        pellet_delivered = false;
        missed_pellet = false;
        return true;
    } else {
        // vTaskDelay(5);
        missed_pellet = true;
        return false;
    }
}

bool feeder_deliver_error() {

    speed_limit = 0;
    quad_encoder.set_rotation(0);
    position_setpoint = 0;
    PIDController_Init(&pid_pos);
    PIDController_Init(&pid_vel);

    if (led_toggle){
        gpio_put(GREEN_LED_PIN, GPIO_OFF);
        led_toggle = !led_toggle;
    } else {
        gpio_put(GREEN_LED_PIN, GPIO_ON);
        led_toggle = !led_toggle;
    }
    vTaskDelay(200);
    return true;
}

