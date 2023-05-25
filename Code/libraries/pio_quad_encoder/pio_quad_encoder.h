#ifndef PIO_QUAD_ENC_H
#define PIO_QUAD_ENC_H

class QuadEncoder
{
public:
    // constructor
    // rotary_encoder_A is the pin for the A of the rotary encoder.
    // The B of the rotary encoder has to be connected to the next GPIO.
    QuadEncoder(uint rotary_encoder_A);
    // set the current rotation to a specific value
    void set_rotation(int _rotation);
    // get the current rotation
    int get_rotation(void);
    // clear the sum of all the time differences
    void clear_time_dif(void);
    // get the sum of all the current time differences
    int get_time_dif(void);

private:
    static void pio_irq_handler();
    // the pio instance
    PIO pio;
    // the state machine
    uint sm;
    // the current location of rotation
    inline static int rotation = 0;
    inline static uint64_t time = 0;                          // To get current time from low level hardware clock
    inline static uint64_t time_old = 0;                      // to get old time from hardware clock to calc dif
    inline static uint64_t time_dif = 0;                      // difference in time between encoder pulses

};

#endif