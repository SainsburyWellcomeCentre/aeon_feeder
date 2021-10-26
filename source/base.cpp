#include <math.h>
#include <vector>
// #include <string.h>
// #include <cstdlib>

// #include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"

#include "breakout_colourlcd240x240.hpp"

#include "gfxfont.h"
#include "FreeMono12pt7b.h"
#include "FreeSansBoldOblique12pt7b.h"
#include "FreeSerif24pt7b.h"


// Screen demo switches
#define SCREEN_TEXT         1

#define ROTARY_SW           7
#define ROTARY_A            8
#define ROTARY_B            9
#define PWM_IN_ONE          10
#define PWN_IN_TWO          11
#define ENC_ONE             12
#define ENC_TWO             13
#define PWM_EN              14
#define GREEN_LED_PIN       25

#define GPIO_ON             1
#define GPIO_OFF            0

using namespace pimoroni;

uint16_t buffer[BreakoutColourLCD240x240::WIDTH * BreakoutColourLCD240x240::HEIGHT];
BreakoutColourLCD240x240 lcd(buffer);


static char event_str[128];

char lcd_message_str[20];

void gpio_event_string(char *buf, uint32_t events);


// Interupt Callback Routines - START
void gpio_callback_rotary_sw(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    sprintf(lcd_message_str, "GPIO %d %s\n", gpio, event_str);
    printf("GPIO %d %s\n", gpio, event_str);
}

void gpio_callback_rotary(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    sprintf(lcd_message_str, "GPIO %d %s\n", gpio, event_str);
    printf("GPIO %d %s\n", gpio, event_str);
}

void gpio_callback_motor(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    // sprintf(lcd_message_str, "GPIO %d %s\n", gpio, event_str);
    printf("GPIO %d %s\n", gpio, event_str);
}

void on_pwm_wrap_one() {
    // static int fade = 0;
    // static bool going_up = true;
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_IN_ONE));

    // if (going_up) {
    //     ++fade;
    //     if (fade > 255) {
    //         fade = 255;
    //         going_up = false;
    //     }
    // } else {
    //     --fade;
    //     if (fade < 0) {
    //         fade = 0;
    //         going_up = true;
    //     }
    // }
    // Square the fade value to make the LED's brightness appear more linear
    // Note this range matches with the wrap value
    // pwm_set_gpio_level(PWM_IN_ONE, fade * fade);
    pwm_set_gpio_level(PWM_IN_ONE, 2058);
}

// Interupt Callback Routines - END

// Define FreeRTOS Task
void vGreenLEDTask( void * pvParameters );

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
    gpio_set_irq_enabled_with_callback(ROTARY_SW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_rotary_sw);
    gpio_set_irq_enabled_with_callback(ROTARY_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_rotary);
    gpio_set_irq_enabled_with_callback(ROTARY_B, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_rotary);

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
    // lcd.text("Wellcome", Point(0, 30), 240, 1); 

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

    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(ENC_ONE, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_motor);
    gpio_set_irq_enabled_with_callback(ENC_TWO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback_motor);
    
    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);

    //PWM_EN
    gpio_init(PWM_EN);
    gpio_set_dir(PWM_EN, GPIO_OUT);

    gpio_set_function(PWM_IN_ONE, GPIO_FUNC_PWM);
    uint slice_num_one = pwm_gpio_to_slice_num(PWM_IN_ONE);
    pwm_clear_irq(slice_num_one);
    pwm_set_irq_enabled(slice_num_one, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap_one);
    irq_set_enabled(PWM_IRQ_WRAP, true);


    pwm_config config_one = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    // pwm_config_set_clkdiv(&config_one, 4.f);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num_one, &config_one, false);
    
    pwm_set_wrap(slice_num_one, 4096);  
    pwm_set_enabled(slice_num_one, true);
    gpio_put(PWM_EN, GPIO_ON);

    BaseType_t status;

    TaskHandle_t xGreenLEDHandle = NULL;

    status = xTaskCreate(
                  vGreenLEDTask,                  // Task Function
                  "Green LED Task",               // Task Name
                  512,                            // Stack size in words
                  NULL,                           // Parameter passed to task
                  tskIDLE_PRIORITY + 1,           // Task Priority
                  &xGreenLEDHandle );   

    
    vTaskStartScheduler();

    while(true) {
        /* Nothing here should run */ 
    }

}

void vGreenLEDTask( void * pvParameters )
{
    for( ;; )
    {
        gpio_put(GREEN_LED_PIN, GPIO_ON);
        vTaskDelay(200);
        gpio_put(GREEN_LED_PIN, GPIO_OFF);
        vTaskDelay(200);
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
