#include <math.h>
#include <vector>
// #include <string.h>
// #include <cstdlib>
// #include <stdio.h>
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

#include "pico_display.hpp"
#include "gfxfont.h"
#include "FreeSans9pt7b.h"
#include "pid.h"
#include "pio_quad_encoder.pio.h"

////////////////////////////////////////
// Define Code Switches
////////////////////////////////////////
#define UART_DEBUG          1

////////////////////////////////////////
// Define Hardware Pins
////////////////////////////////////////
#define BNC_INPUT           11
#define PWM_OUT_ONE         2
#define PWM_OUT_TWO         3
#define PWM_EN              4
#define ENC_ONE             9
#define ENC_TWO             10
#define BEAM_BREAK_PIN      5
#define GREEN_LED_PIN       25

////////////////////////////////////////
// Define SET Software Values
////////////////////////////////////////
#define GPIO_ON             1
#define GPIO_OFF            0

// #define LEVEL_LOW           0x1
// #define LEVEL_HIGH          0x2
// #define EDGE_FALL           0x4
// #define EDGE_RISE           0x8

#define TIME_DIF_MAX        1000u
#define MAX_ROTATION_TOTAL  23104u
#define MAX_ROTATION_HOLE   1925u   // basically MAX_ROTATION_TOTAL / 12 (number of holes)
#define ROTATION_PRE_LOAD   1500u   // distance to travel around before stopping from the last hole to pre load a pellet
#define SPEED_MAX           400u
#define SPEED_PRE_LOAD      200u
#define SPEED_DELIVER       100u
#define POSITION_MARGIN     15u

#define LCD_STRING_BUF_SIZE     30
#define UART_STRING_BUF_SIZE    40

#define MULTICORE_FIFO_TIMEOUT  1000u

////////////////////////////////////////
// Define Feeder Application States
////////////////////////////////////////

#define FEEDER_INITIALISE         0x01
#define FEEDER_PRE_LOAD        0x02
#define FEEDER_DELIVERED           0x04

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
using namespace pimoroni;

uint16_t buffer[PicoDisplay::WIDTH * PicoDisplay::HEIGHT];
PicoDisplay pico_display(buffer);

// class to read the rotation of the rotary encoder
class RotaryEncoder
{
public:
    // constructor
    // rotary_encoder_A is the pin for the A of the rotary encoder.
    // The B of the rotary encoder has to be connected to the next GPIO.
    RotaryEncoder(uint rotary_encoder_A)
    {
        uint8_t rotary_encoder_B = rotary_encoder_A + 1;
        // pio 0 is used
        PIO pio = pio0;
        // state machine 0
        uint8_t sm = 0;
        // configure the used pins as input with pull up
        pio_gpio_init(pio, rotary_encoder_A);
        gpio_set_pulls(rotary_encoder_A, true, false);
        pio_gpio_init(pio, rotary_encoder_B);
        gpio_set_pulls(rotary_encoder_B, true, false);
        // load the pio program into the pio memory
        uint offset = pio_add_program(pio, &pio_rotary_encoder_program);
        // make a sm config
        pio_sm_config c = pio_rotary_encoder_program_get_default_config(offset);
        // set the 'in' pins
        sm_config_set_in_pins(&c, rotary_encoder_A);
        // set shift to left: bits shifted by 'in' enter at the least
        // significant bit (LSB), no autopush
        sm_config_set_in_shift(&c, false, false, 0);
        // set the IRQ handler
        irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
        // enable the IRQ
        irq_set_enabled(PIO0_IRQ_0, true);
        pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS;
        // init the sm.
        // Note: the program starts after the jump table -> initial_pc = 16
        pio_sm_init(pio, sm, 16, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
    }

    // set the current rotation to a specific value
    void set_rotation(int _rotation)
    {
        rotation = _rotation;
    }

    // get the current rotation
    int get_rotation(void)
    {
        return rotation;
    }

    // set the current rotation to a specific value
    void clear_time_dif(void)
    {
        time_dif = 0;
    }

    // get the current time difference
    int get_time_dif(void)
    {
        return time_dif;
    }

private:
    static void pio_irq_handler()
    {
        uint32_t lo = timer_hw->timelr;
        uint32_t hi = timer_hw->timehr;
        time = ((uint64_t) hi << 32u) | lo;
        time_dif += (time - time_old);
        // test if irq 0 was raised
        if (pio0_hw->irq & 1)
        {
            rotation = rotation - 1;
        }
        // test if irq 1 was raised
        if (pio0_hw->irq & 2)
        {
            rotation = rotation + 1;
        }
        // clear both interrupts
        time_old = time;
        pio0_hw->irq = 3;
    }

    // the pio instance
    PIO pio;
    // the state machine
    uint sm;
    // the current location of rotation
    static int rotation;
    static uint64_t time;                          // To get current time from low level hardware clock
    static uint64_t time_old;                      // to get old time from hardware clock to calc dif
    static uint64_t time_dif;                      // difference in time between encoder pulses

};

// Initialize static member of class Rotary_encoder
int RotaryEncoder::rotation = 0;
uint64_t RotaryEncoder::time = 0;
uint64_t RotaryEncoder::time_old = 0;
uint64_t RotaryEncoder::time_dif = 0;

RotaryEncoder quad_encoder(ENC_ONE);

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
unsigned long time = to_ms_since_boot(get_absolute_time());
const int delayTime =  250;
// block extra pellet triggers per shot - to fix pellet count
unsigned long pellet_time = to_ms_since_boot(get_absolute_time());
const int pellet_delay_time =  250;

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
volatile int32_t pellet_delivered_count = 0;
volatile int32_t pellet_delivered_msg_count = 0;
volatile bool lcd_message_flag;
volatile bool uart_message_flag;
volatile bool motor_brake;
volatile bool bnc_triggered;
volatile bool led_toggle;

volatile bool A_flag;
volatile bool B_flag;
volatile bool X_flag;
volatile bool Y_flag;

volatile uint32_t application_status = 0;
volatile uint32_t application_flags = 0;

divmod_result_t delta_time_hw;
divmod_result_t motor_speed_hw;

// static char event_str[128];

static char lcd_message_str[LCD_STRING_BUF_SIZE];
static char uart_message_str[UART_STRING_BUF_SIZE];

// Define FreeRTOS Task
void vSerialDebugTask( void * pvParameters );
void vApplicationTask( void * pvParameters );
void vUpdateScreenTask( void * pvParameters );
void vUserInterfaceTask( void * pvParameters );
void vSystemMonitorTask( void * pvParameters );

bool feeder_initialise(void);
bool feeder_pre_load_pellet(void);
bool feeder_deliver_pellet(void);

// void gpio_event_string(char *buf, uint32_t events);

////////////////////////////////////////
// Interupt Callback Routines - START
////////////////////////////////////////
// CORE 1 - START
////////////////////////////////////////
// GPIO Interupt
void gpio_callback_core_1(uint gpio, uint32_t events) {
    // irq_clear(IO_IRQ_BANK0);
    if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) {
        time = to_ms_since_boot(get_absolute_time());
        switch(gpio) {
            case pico_display.A:
                pico_display.set_led(255,0,0);
                multicore_fifo_push_timeout_us(gpio, MULTICORE_FIFO_TIMEOUT);
                break;
            case pico_display.B:
                pico_display.set_led(0,255,0);
                multicore_fifo_push_timeout_us(gpio, MULTICORE_FIFO_TIMEOUT);
                break;
            case pico_display.X:
                pico_display.set_led(0,0,255);
                multicore_fifo_push_timeout_us(gpio, MULTICORE_FIFO_TIMEOUT);
                break;
            case pico_display.Y:
                pico_display.set_led(0,0,0);
                multicore_fifo_push_timeout_us(gpio, MULTICORE_FIFO_TIMEOUT);
                break;
            default:

            break;
        }
    }
}
////////////////////////////////////////
// CORE 1 - END
////////////////////////////////////////

////////////////////////////////////////
// CORE 0 - START
////////////////////////////////////////
// GPIO Interupt
void gpio_callback_core_0(uint gpio, uint32_t events) {
    
    if ((to_ms_since_boot(get_absolute_time()) - pellet_time) > pellet_delay_time) {
        pellet_time = to_ms_since_boot(get_absolute_time());
        switch(gpio) {
            case BEAM_BREAK_PIN:
                pellet_delivered_count++;
                pellet_delivered = true;
                break;
            default:

                break;
        }
    }

}
// Timer Callback Interupt for 2ms Control Loop.
bool repeating_timer_callback(struct repeating_timer *t) {
    
    static uint32_t r_d;                        // Change in position
    static uint64_t t_d;                        // Change in time

    motor_position = quad_encoder.get_rotation();
    r_d = motor_position - motor_position_old;
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

    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT_ONE));

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

    pwm_set_gpio_level(PWM_OUT_ONE, set_pwm_one);
    pwm_set_gpio_level(PWM_OUT_TWO, set_pwm_two);
}
////////////////////////////////////////
// CORE 0 - END
////////////////////////////////////////
// Interupt Callback Routines - END
////////////////////////////////////////

////////////////////////////////////////
// SWC Base code - will run on Core 1 - START
////////////////////////////////////////
void swc_base() {

    
    gpio_set_irq_enabled_with_callback(pico_display.A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_1);
    gpio_set_irq_enabled_with_callback(pico_display.B, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_1);
    gpio_set_irq_enabled_with_callback(pico_display.X, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_1);
    gpio_set_irq_enabled_with_callback(pico_display.Y, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_1);


    int x = 0;
    uint16_t white = pico_display.create_pen(255, 255, 255);
    // uint16_t black = pico_display.create_pen(0, 0, 0);
    // uint16_t red = pico_display.create_pen(255, 0, 0);
    // uint16_t green = pico_display.create_pen(0, 255, 0);
    // uint16_t dark_grey = pico_display.create_pen(20, 40, 60);
    // uint16_t dark_green = pico_display.create_pen(10, 100, 10);
    // uint16_t blue = pico_display.create_pen(0, 0, 255);

    sprintf(lcd_message_str, "Hello from Core 1");
    // sprintf(lcd_message_str, "PRESS A TO INITIALISE");

    pico_display.init();
    pico_display.set_backlight(100);

    while(true) {
        pico_display.set_pen(0, 0, 0);
        pico_display.clear();

        while(multicore_fifo_rvalid()) {
            lcd_message_str[x] = multicore_fifo_pop_blocking();
            x++;
        }
        x = 0;

        pico_display.set_pen(white);
        pico_display.customFontSetFont((const pimoroni::GFXfont&)FreeSans9pt7b);
        pico_display.text(lcd_message_str, Point(0, 30), 240, 1);

        pico_display.update();
    }

}
////////////////////////////////////////
// SWC Base code - will run on Core 1 - END
////////////////////////////////////////

////////////////////////////////////////
// Main code - will run on Core 0 - START
////////////////////////////////////////
int main() 
{
    multicore_launch_core1(swc_base);
    stdio_init_all();

    quad_encoder.set_rotation(0);

    sprintf(uart_message_str, "Hello from Core 0\n");
    uart_message_flag = true;

    gpio_init(BEAM_BREAK_PIN);
    gpio_set_dir(BEAM_BREAK_PIN, GPIO_IN);

// BEAM_BREAK_PIN 
    gpio_set_irq_enabled_with_callback(BEAM_BREAK_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_0);

    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);

    gpio_init(PWM_EN);
    gpio_set_dir(PWM_EN, GPIO_OUT);

    gpio_set_function(PWM_OUT_ONE, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT_TWO, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(PWM_OUT_ONE); 

    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 4096);
    pwm_init(slice_num, &config, true);


    gpio_put(PWM_EN, GPIO_ON);

    BaseType_t status;
#if UART_DEBUG
    TaskHandle_t xSerialDebugHandle = NULL;
#endif
    TaskHandle_t xApplicationHandle = NULL;
    TaskHandle_t xUpdateScreenHandle = NULL;
    TaskHandle_t xUserInterfaceHandle = NULL;
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
                  vUpdateScreenTask,              // Task Function
                  "Update Screen Task",           // Task Name
                  256,                            // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY,           // Task Priority
                  &xUpdateScreenHandle );  

    status = xTaskCreate(
                  vUserInterfaceTask,              // Task Function
                  "User Interface Task",           // Task Name
                  256,                            // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY + 2,           // Task Priority
                  &xUserInterfaceHandle ); 

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
    speed_limit = SPEED_MAX;

    application_status = application_status | FEEDER_INITIALISE;

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

    sprintf(uart_message_str, "PRESS A TO INITIALISE\n");
    uart_message_flag = true;

    for( ;; )
    {    
        switch (application_status) {
            case FEEDER_INITIALISE:
                if (A_flag && ((application_flags & FEEDER_INITIALISE) != FEEDER_INITIALISE)) {
                    status = feeder_initialise();
                    while (!status) {
                        status = feeder_initialise();
                        i++;
                        if (i > (6 - 1)) break;
                    }
                    if (status) {
                        application_flags = application_flags | FEEDER_INITIALISE;
                        application_status = application_status & ~FEEDER_INITIALISE;
                        application_status = application_status | FEEDER_PRE_LOAD;
                        sprintf(uart_message_str, "FEEDER INITIALISED\n");
                        uart_message_flag = true;
                        vTaskDelay(10);
                    } else {
                        application_flags = application_flags & ~FEEDER_INITIALISE;
                        sprintf(uart_message_str, "FEEDER ERROR\n");
                        uart_message_flag = true;
                        vTaskDelay(10);
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
                        sprintf(uart_message_str, "FEEDER PRE LOADED\n");
                        uart_message_flag = true;
                        vTaskDelay(10);
                    } else {
                        application_flags = application_flags & ~FEEDER_PRE_LOAD;
                        sprintf(uart_message_str, "FEEDER ERROR\n");
                        uart_message_flag = true;
                        vTaskDelay(10);
                    }
                    break;
                }
            case FEEDER_DELIVERED:
                status = feeder_deliver_pellet();
                break;
            default:
                
                break;            
        }

        vTaskDelay(200);
    }
}

void vUpdateScreenTask( void * pvParameters )
{
    int y;
    for( ;; )
    {
        if (lcd_message_flag) {

            for (y = 0; y < LCD_STRING_BUF_SIZE; y++) {
                multicore_fifo_push_blocking((uintptr_t) lcd_message_str[y]);
            }
            lcd_message_flag = false;
        }
        vTaskDelay(20);
    }
}

// change in vUserInterfaceTask below for debuging - must remove --- DO NOT FORGET GRAEME
void vUserInterfaceTask( void * pvParameters )
{
    static uint32_t x = 0;
    static bool status;

    for( ;; )
    {
        status = multicore_fifo_pop_timeout_us(MULTICORE_FIFO_TIMEOUT, &x);

        if (status){           
            switch(x) {
                case pico_display.A:
                    A_flag = true;
                    // position_setpoint += MAX_ROTATION_HOLE;
                    break;
                case pico_display.B:
                    B_flag = true;
                    // position_setpoint -= MAX_ROTATION_HOLE;
                    break;
                case pico_display.X:
                    X_flag = true;
                    position_setpoint += MAX_ROTATION_HOLE>>2;  // will have to remove this --- DO NOT FORGET GRAEME
                    // position_setpoint += MAX_ROTATION_TOTAL;
                    break;
                case pico_display.Y:
                    Y_flag = true;
                    // position_setpoint = 0;
                    break;
                default:

                break;
            }
        }
        vTaskDelay(50);
    }
}
 
void vSystemMonitorTask( void * pvParameters )
{
    for( ;; )
    {
#if UART_DEBUG
        // if (!uart_message_flag && pellet_delivered) {
        //     if (pellet_delivered_count != pellet_delivered_msg_count) {
        //         sprintf(uart_message_str, "Pellet %d Delivered\n", pellet_delivered_count);
        //         uart_message_flag = true;
        //         pellet_delivered_msg_count = pellet_delivered_count;
        //     }
        // }
        motor_position_buf = motor_position;       
        if (!uart_message_flag && !feeder_position_reached) {
            sprintf(uart_message_str, "Position=%d PosSet=%d Pellets=%d\n", motor_position_buf, position_setpoint, pellet_delivered_count);
            // sprintf(uart_message_str, "Position=%d PosSet=%d SpeedSet=%d\n", motor_position_buf, position_setpoint, speed_setpoint);
            uart_message_flag = true;
        }
#endif
        vTaskDelay(20);
    }
}

////////////////////////////////////////
// FreeRTOS Tasks - will run on Core 0 - END
////////////////////////////////////////


bool feeder_initialise() {

    sprintf(uart_message_str, "INITIALISING\n");
    uart_message_flag = true;
    vTaskDelay(10);

    speed_limit = SPEED_DELIVER;
    position_setpoint = position_setpoint + MAX_ROTATION_HOLE;
    feeder_position_reached = false;

    while (!feeder_position_reached && !pellet_delivered) {
        // vTaskDelay(1);
    }
    
    if (pellet_delivered) {
        speed_limit = 0;
        quad_encoder.set_rotation(0);
        position_setpoint = 0;
        PIDController_Init(&pid_pos);
        PIDController_Init(&pid_vel);
        vTaskDelay(5);
        speed_limit = SPEED_PRE_LOAD;
        return true;
    } else {
        return false;
    }
}


bool feeder_pre_load_pellet() {

    sprintf(uart_message_str, "Pre Loading Pellet\n");
    uart_message_flag = true;
    vTaskDelay(10);

    speed_limit = SPEED_PRE_LOAD;
    position_setpoint = position_setpoint + ROTATION_PRE_LOAD;
    feeder_position_reached = false;

    while (!feeder_position_reached) {
        vTaskDelay(10);
    }
    
    sprintf(uart_message_str, "Pre Loaded Pellet\n");
    uart_message_flag = true;
    vTaskDelay(10);

    if (feeder_position_reached) {
        return true;
    } else {
        return false;
    }

}

bool feeder_deliver_pellet() {

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
////////////////////////////////////////
// Debug code for GPIO Interupts - START
////////////////////////////////////////

// static const char *gpio_irq_str[] = {
//         "LEVEL_LOW",  // 0x1
//         "LEVEL_HIGH", // 0x2
//         "EDGE_FALL",  // 0x4
//         "EDGE_RISE"   // 0x8
// };

// void gpio_event_string(char *buf, uint32_t events) {
//     for (uint i = 0; i < 4; i++) {
//         uint mask = (1 << i);
//         if (events & mask) {
//             // Copy this event string into the user string
//             const char *event_str = gpio_irq_str[i];
//             while (*event_str != '\0') {
//                 *buf++ = *event_str++;
//             }
//             events &= ~mask;
//             // If more events add ", "
//             if (events) {
//                 *buf++ = ',';
//                 *buf++ = ' ';
//             }
//         }
//     }
//     *buf++ = '\0';
// }

////////////////////////////////////////
// Debug code for GPIO Interupts - END
////////////////////////////////////////