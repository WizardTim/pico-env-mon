#ifndef LS027B4DH01_HPP
#define LS027B4DH01_HPP

#include "stdint.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "msb1stimage.hpp"

static constexpr int SCREEN_WIDTH = 400;
static constexpr int SCREEN_HEIGHT = 240;
static constexpr int SCREEN_STRIDE = (SCREEN_WIDTH + 7) / 8;

class LcdScreen : public Msb1stImage {
private:
    uint8_t buff_data[SCREEN_STRIDE * SCREEN_HEIGHT];
public:
    LcdScreen() : Msb1stImage(SCREEN_WIDTH, SCREEN_HEIGHT, (uint8_t*)buff_data) {}
};

class LcdDriver {
public:
    spi_inst_t * const spi;
    const int pin_scs;
    const int pin_disp;
    const int pin_extcomin;

    LcdDriver(spi_inst_t *spi, int pin_scs, int pin_disp, int pin_extcomin) :
        spi(spi), pin_scs(pin_scs), pin_disp(pin_disp), pin_extcomin(pin_extcomin)
    { }

    void init() {
        gpio_init(pin_scs);
        gpio_init(pin_disp);
        gpio_init(pin_extcomin);
        gpio_set_dir(pin_scs, GPIO_OUT);
        gpio_set_dir(pin_disp, GPIO_OUT);
        gpio_put(pin_scs, 0);
        gpio_put(pin_disp, 0);
		
		// Experimental PWM EXTCOMIN
		// Control LCD liquid crystal cell polarity inversion with hardware PWM instead of in software (processor can sleep more)
		// HOWEVER, LCD EXTCOMIN only arms the internal COMZ signal, inversion occurs on next SCS falling edge! (EXTMODE = HIGH)
		// So PWM does 'work', but only during an gpio_put(pin_scs, 0)
		pwm_config config = pwm_get_default_config();
		gpio_set_function(pin_extcomin, GPIO_FUNC_PWM);
		pwm_init(pwm_gpio_to_slice_num(pin_extcomin), &config, true);
		pwm_set_clkdiv(pwm_gpio_to_slice_num(pin_extcomin), 256); // Minimum PWM speed is ~7.5 Hz using 1 PWM slice and default clock sources
		pwm_set_gpio_level(pin_extcomin, 65535/2); // 50% duty cycle (only rising edge and period matters, minimum 1 us high)
    }

    void disp_on() { gpio_put(pin_disp, 1); }
    void disp_off() { gpio_put(pin_disp, 0); }

    static uint8_t revert_byte(uint8_t b) {
        return 
            ((b << 7) & 0x80) |
            ((b << 5) & 0x40) |
            ((b << 3) & 0x20) |
            ((b << 1) & 0x10) |
            ((b >> 1) & 0x08) |
            ((b >> 3) & 0x04) |
            ((b >> 5) & 0x02) |
            ((b >> 7) & 0x01);
    }

    void write(const uint8_t *data, int height = SCREEN_HEIGHT, int dest_y = 0) {
        uint8_t spi_byte;

        uint8_t gate_line = dest_y + 1;

        // chip select
        gpio_put(pin_scs, 1);
        sleep_us(1);
        
        // mode select
        spi_byte = revert_byte(0x01);
        spi_write_blocking(spi, &spi_byte, 1);
        for (int src_y = 0; src_y < height; src_y++) {
            // line select
            spi_byte = revert_byte(gate_line);
            spi_write_blocking(spi, &spi_byte, 1);

            // write line
            spi_write_blocking(spi, data, SCREEN_STRIDE);

            // dummy write
            spi_byte = 0x00;
            spi_write_blocking(spi, &spi_byte, 1);

            data += SCREEN_STRIDE;
            gate_line += 1;
        }

        // dummy write
        spi_byte = 0x00;
        spi_write_blocking(spi, &spi_byte, 1);

        // chip deselect
        sleep_us(1);
        gpio_put(pin_scs, 0);
        sleep_us(1);

    }

};

#endif
