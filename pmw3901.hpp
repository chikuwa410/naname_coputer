#ifndef PMW3901_HPP
#define PMW3901_HPP

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <cstdint>

#define PMW3901_SPI_CLOCK 2000000
#define PMW3901_DELAY_US 50

class PMW3901 {
public:
    PMW3901(spi_inst_t* spi = spi0,
            uint pin_miso = 16,
            uint pin_mosi = 19,
            uint pin_sck  = 18,
            uint pin_cs   = 17);

    bool pmw_init();
    bool pmw_deinit();
    bool selectBank(uint8_t bank);
    bool reset();
    bool initRegisters();

    bool readRegister(uint8_t reg ,uint8_t *value);
    bool writeRegister(uint8_t reg ,uint8_t value);
    bool readMotion(int16_t *delta_x ,int16_t *delta_y );


private:
    spi_inst_t* _spi;
    uint _pin_miso, _pin_mosi, _pin_sck, _pin_cs;

    void cs_low();
    void cs_high();
};

#endif