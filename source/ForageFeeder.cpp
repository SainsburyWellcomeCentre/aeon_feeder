#include <math.h>
#include <vector>
// #include <string.h>
// #include <cstdlib>

// Demo switches
#define SCREEN_TEXT         1
#define PWM_ENABLE          1
#define QUAD_ENCODER        1

// #include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "hardware/irq.h"
#if PWM_ENABLE
#include "hardware/pwm.h"
#endif
#include "FreeRTOS.h"
#include "task.h"

#include "breakout_colourlcd240x240.hpp"
#include "gfxfont.h"
// #include "FreeMono12pt7b.h"
#include "FreeSansBoldOblique12pt7b.h"
// #include "FreeSerif24pt7b.h"
#include "pid.h"

#define ROTARY_SW           7
#define ROTARY_A            8
#define ROTARY_B            9
#define ROTARY_RIGHT        1
#define ROTARY_LEFT         0
#if PWM_ENABLE
#define PWM_IN_ONE          10
#define PWM_IN_TWO          11
#define PWM_EN              14
#endif
#if QUAD_ENCODER
#define ENC_ONE             12
#define ENC_TWO             13
#endif
#define BEAM_BREAK_PIN      15
#define GREEN_LED_PIN       25

#define GPIO_ON             1
#define GPIO_OFF            0

#define LEVEL_LOW           0x1
#define LEVEL_HIGH          0x2
#define EDGE_FALL           0x4
#define EDGE_RISE           0x8

/* Velocity Controller parameters */
#define PID_VEL_KP  2.0f
#define PID_VEL_KI  0.5f
#define PID_VEL_KD  0.25f

#define PID_VEL_TAU 0.02f

#define PID_VEL_LIM_MIN -10.0f
#define PID_VEL_LIM_MAX  10.0f

#define PID_VEL_LIM_MIN_INT -5.0f
#define PID_VEL_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_VEL_S 0.001f

/* Position Controller parameters */
#define PID_POS_KP  2.0f
#define PID_POS_KI  0.5f
#define PID_POS_KD  0.25f

#define PID_POS_TAU 0.02f

#define PID_POS_LIM_MIN -10.0f
#define PID_POS_LIM_MAX  10.0f

#define PID_POS_LIM_MIN_INT -5.0f
#define PID_POS_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_POS_S 0.001f

using namespace pimoroni;

uint16_t buffer[BreakoutColourLCD240x240::WIDTH * BreakoutColourLCD240x240::HEIGHT];
BreakoutColourLCD240x240 lcd(buffer);

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

volatile bool rotary_direction;
<<<<<<< HEAD
volatile bool rotary_int;
=======
volatile bool rotary_int_flag;
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
volatile bool rotary_rotation_flag;
volatile bool rotary_switch_In_flag;
volatile bool rotary_switch_Out_flag;
volatile bool motor_direction;
int motor_postion = 0;
int motor_postion_old = 0;
int motor_speed = 0;
int motor_speed_old = 0;
<<<<<<< HEAD
int speed_setpoint = 0;
=======
int velocity_setpoint = 0;
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
uint16_t set_pwm_one = 0;
uint16_t set_pwm_two = 0;
volatile bool pellet_delivered;
volatile bool LED_TOGGLE;

static char event_str[128];

char lcd_message_str[20];

// Define FreeRTOS Task
void vGreenLEDTask( void * pvParameters );
void vApplicationTask( void * pvParameters );
void vUpdateScreenTask( void * pvParameters );

void gpio_event_string(char *buf, uint32_t events);
void initialise_feeder(void);

// Interupt Callback Routines - START
// GPIO Callback for Core 1
void gpio_callback_core_1(uint gpio, uint32_t events) {
<<<<<<< HEAD
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    // gpio_event_string(event_str, events);
    // sprintf(lcd_message_str, "GPIO %d %s\n", gpio, event_str);
    // printf("GPIO %d %s\n", gpio, event_str);
=======
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
    switch(gpio) {
        case ROTARY_A:
            switch(events) {
                case EDGE_FALL:
<<<<<<< HEAD
                    if (rotary_int){
=======
                    if (rotary_int_flag){
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
                        rotary_direction = ROTARY_LEFT;
                        rotary_rotation_flag = true;
                        printf("Rotary Left\n");
                    } else {
<<<<<<< HEAD
                        rotary_int = true;
                    }
                    break;
                case EDGE_RISE:
                    rotary_int = false;
=======
                        rotary_int_flag = true;
                    }
                    break;
                case EDGE_RISE:
                    rotary_int_flag = false;
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
                    break;
                default:
                    break;
            }
            break;
        case ROTARY_B:
            switch(events) {
                case EDGE_FALL:
<<<<<<< HEAD
                    if (rotary_int){
=======
                    if (rotary_int_flag){
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
                        rotary_direction = ROTARY_RIGHT;
                        rotary_rotation_flag = true;
                        printf("Rotary Right\n");
                    } else {
<<<<<<< HEAD
                        rotary_int = true;
                    }
                    break;
                case EDGE_RISE:
                    rotary_int = false;
=======
                        rotary_int_flag = true;
                    }
                    break;
                case EDGE_RISE:
                    rotary_int_flag = false;
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
                    break;
                default:
                    break;
            }
            break;
        case ROTARY_SW:
            switch(events) {
                case EDGE_FALL:
                    rotary_switch_In_flag = true;
                    printf("Rotary Switch IN\n");
                    break;
                case EDGE_RISE:
                    rotary_switch_Out_flag = true;
                    printf("Rotary Switch OUT\n");
                    break;
                default:

                    break;
            }
            
            // gpio_event_string(event_str, events);
            // printf("GPIO %d %s\n", gpio, event_str);
            break;
        default:
            break;
    }
}

// GPIO Callback for Core 0
void gpio_callback_core_0(uint gpio, uint32_t events) {

    switch(gpio) {
#if QUAD_ENCODER
        case ENC_ONE:
            motor_direction = ((gpio_get(ENC_ONE))^(gpio_get(ENC_TWO)));    // For some unknown reason, the compiler does not like adding the ! here
            motor_direction = !motor_direction;                             // If you put ! here then it works 100% of the time
            if (motor_direction){
                motor_postion++;
            } else{
                motor_postion--;
            }
            break;
        case ENC_TWO:
            motor_direction = ((gpio_get(ENC_ONE))^(gpio_get(ENC_TWO)));
            if (motor_direction){
                motor_postion++;
            } else{
                motor_postion--;
            }
            break;
#endif
        case BEAM_BREAK_PIN:
            pellet_delivered = true;
            set_pwm_one = 0;    // NB must remove this later after testing.....
            gpio_event_string(event_str, events);
            printf("GPIO %d %s\n", gpio, event_str);
            break;
        default:

            break;
    }
}

// Timer Callback for 1ms Control Loop.
bool repeating_timer_callback(struct repeating_timer *t) {
    // if (LED_TOGGLE){
    //     gpio_put(GREEN_LED_PIN, GPIO_OFF);
    //     LED_TOGGLE = !LED_TOGGLE;
    // } else {
    //     gpio_put(GREEN_LED_PIN, GPIO_ON);
    //     LED_TOGGLE = !LED_TOGGLE;
    // }
    motor_speed = motor_postion - motor_postion_old;
<<<<<<< HEAD
=======
    motor_postion_old = motor_postion;
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
    return true;
}

#if PWM_ENABLE
void on_pwm_wrap() {

    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_IN_ONE));

<<<<<<< HEAD
    if (speed_setpoint == 0) {
        set_pwm_two = 0;
        set_pwm_one = 0;
    } else if (speed_setpoint > 0) {
        set_pwm_one = speed_setpoint;
        if (set_pwm_one > 4095) set_pwm_one = 4095;
        set_pwm_two = 0;
    } else {
        set_pwm_two = speed_setpoint*-1;
        if (set_pwm_two > 4095) set_pwm_two = 4095;
        set_pwm_one = 0;
    }

=======
    if (velocity_setpoint == 0) {
        set_pwm_two = 0;
        set_pwm_one = 0;
    } else if (velocity_setpoint > 0) {
        set_pwm_one = velocity_setpoint;
        if (set_pwm_one > 4095) set_pwm_one = 4095;
        set_pwm_two = 0;
    } else {
        set_pwm_two = velocity_setpoint*-1;
        if (set_pwm_two > 4095) set_pwm_two = 4095;
        set_pwm_one = 0;
    }
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
    pwm_set_gpio_level(PWM_IN_ONE, set_pwm_one);
    pwm_set_gpio_level(PWM_IN_TWO, set_pwm_two);
}

#endif

// Interupt Callback Routines - END

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

// SWC Base code - will run on Core 1 - START
void swc_base() {
  
    printf("Hello GPIO IRQ\n");
    sprintf(lcd_message_str, "Hello GPIO IRQ");
    gpio_set_irq_enabled_with_callback(ROTARY_SW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_1);
    gpio_set_irq_enabled_with_callback(ROTARY_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_1);
    gpio_set_irq_enabled_with_callback(ROTARY_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_1);

    lcd.init();
    lcd.set_backlight(255);

  while(true) {
    lcd.set_pen(120, 40, 60);
    lcd.clear();

#if SCREEN_TEXT
    lcd.set_pen(255,255,255);
    lcd.customFontSetFont((const pimoroni::GFXfont&)FreeSansBoldOblique12pt7b);
    // lcd.customFontSetFont((const pimoroni::GFXfont&)FreeMono12pt7b);
    // lcd.customFontSetFont((const pimoroni::GFXfont&)FreeSerif24pt7b); 
    // lcd.customFontSetFont();
    // sprintf(lcd_message_str, "Hello");
    lcd.text(lcd_message_str, Point(0, 30), 240, 1);  

#endif
    // update screen
    lcd.update();
    }
}
// SWC Base code - will run on Core 1 - END

int main() 
{
    multicore_launch_core1(swc_base);

    stdio_init_all();

#if QUAD_ENCODER
    gpio_set_irq_enabled_with_callback(ENC_ONE, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_0);
    gpio_set_irq_enabled_with_callback(ENC_TWO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_0);
#endif

// BEAM_BREAK_PIN 
    gpio_set_irq_enabled_with_callback(BEAM_BREAK_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_core_0);

    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);

#if PWM_ENABLE  //PWM_EN
    gpio_init(PWM_EN);
    gpio_set_dir(PWM_EN, GPIO_OUT);

    gpio_set_function(PWM_IN_ONE, GPIO_FUNC_PWM);
    gpio_set_function(PWM_IN_TWO, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(PWM_IN_ONE); // slice_num for both PWM_IN_ONE and PWM_IN_TWO = 5
    
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_config config = pwm_get_default_config();
    // pwm_config_set_clkdiv(&config_one, 1);
    pwm_config_set_wrap(&config, 4096);
    pwm_init(slice_num, &config, true);

    gpio_put(PWM_EN, GPIO_ON);
#endif

    BaseType_t status;

    TaskHandle_t xGreenLEDHandle = NULL;
    TaskHandle_t xApplicationHandle = NULL;
    TaskHandle_t xUpdateScreenHandle = NULL;

    status = xTaskCreate(
                  vGreenLEDTask,                  // Task Function
                  "Green LED Task",               // Task Name
                  512,                            // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY + 1,           // Task Priority
                  &xGreenLEDHandle );   

    status = xTaskCreate(
                  vApplicationTask,               // Task Function
                  "Application Task",             // Task Name
                  1024,                           // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY + 2,           // Task Priority
                  &xApplicationHandle );  

    status = xTaskCreate(
                  vUpdateScreenTask,              // Task Function
                  "Update Screen Task",           // Task Name
                  512,                            // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY + 1,           // Task Priority
                  &xUpdateScreenHandle );  

    // Add control loop timer 
    add_repeating_timer_us(-1000, repeating_timer_callback, NULL, &control_loop);   // 1ms
    add_repeating_timer_ms(-10, repeating_timer_callback, NULL, &control_loop);   // 10ms

    PIDController_Init(&pid_vel);
    PIDController_Init(&pid_pos);

    vTaskStartScheduler();

    while(true) {
        /* Nothing here should run */ 
    }
}

void vGreenLEDTask( void * pvParameters )
{
    for( ;; )
    {
        // gpio_put(GREEN_LED_PIN, GPIO_ON);
        // vTaskDelay(200);
        // gpio_put(GREEN_LED_PIN, GPIO_OFF);
        // vTaskDelay(200);
    }
}

void vApplicationTask( void * pvParameters )
{
    // initialise_feeder();     // commented out while doing the control loops

    for( ;; )
    {
<<<<<<< HEAD
        if (rotary_rotation_flag) {
            if (rotary_direction) {
                speed_setpoint += 250;
            } else {
                speed_setpoint -= 250;
=======
        // gpio_put(GREEN_LED_PIN, GPIO_ON);
        if (rotary_rotation_flag) {
            if (rotary_direction) {
                velocity_setpoint += 250;
            } else {
                velocity_setpoint -= 250;
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
            }
            rotary_rotation_flag = false;
        }
        if (rotary_switch_In_flag) {
<<<<<<< HEAD
            speed_setpoint = 4095;
            rotary_switch_In_flag = false;
        }
        if (rotary_switch_Out_flag) {
            speed_setpoint = 0;
            rotary_switch_Out_flag = false;
        }
        vTaskDelay(10);
        
=======
            velocity_setpoint = 4095;
            rotary_switch_In_flag = false;
        }
        if (rotary_switch_Out_flag) {
            velocity_setpoint = 0;
            rotary_switch_Out_flag = false;
        }
        vTaskDelay(10);
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
        // gpio_put(GREEN_LED_PIN, GPIO_OFF);
        // vTaskDelay(1);
    }
}

void vUpdateScreenTask( void * pvParameters )
{
    for( ;; )
    {
<<<<<<< HEAD
        if (motor_postion != motor_postion_old){
            printf("Speed %d\n", motor_speed);
        }
        motor_postion_old = motor_postion;
=======
        if (motor_speed != motor_speed_old){
            printf("Speed %d\n", motor_speed);
        }
        motor_speed_old = motor_speed;
>>>>>>> 90d0b632553ffc72e81ba176dacc03b4b786a446
        vTaskDelay(10);
    }
}

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

void initialise_feeder(void) {
    set_pwm_one = 150;
    while(!pellet_delivered){
        vTaskDelay(1);
    }
    set_pwm_one = 0;
    pellet_delivered = false;

}