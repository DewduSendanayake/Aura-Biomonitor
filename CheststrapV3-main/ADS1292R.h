#ifndef ADS1292R_H
#define ADS1292R_H

#include <Arduino.h>
#include <SPI.h>

#include "Config.h"
#include "DataTypes.h"

class ADS1292R
{
public:

    ADS1292R();

    bool begin();

    void update();

    bool available();

    bool read(ECGData &data);

    void start();

    void stop();

    bool selfTest();

    uint8_t getID();

private:

    SPIClass spi;

    volatile bool dataReady;

    static ADS1292R* instance;

    static void IRAM_ATTR drdyISR();

    void hardwareReset();

    void sendCommand(uint8_t cmd);

    void writeRegister(uint8_t reg, uint8_t value);

    uint8_t readRegister(uint8_t reg);

    int32_t read24();

    void configure();

    bool readFrame(ECGData &data);
};

#endif