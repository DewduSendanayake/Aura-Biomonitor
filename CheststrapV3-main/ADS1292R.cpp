#include "ADS1292R.h"

/************************************************
 * ADS1292R Commands
 ************************************************/

#define WAKEUP     0x02
#define STANDBY    0x04
#define RESET_CMD  0x06
#define START_CMD  0x08
#define STOP_CMD   0x0A
#define RDATAC     0x10
#define SDATAC     0x11
#define RDATA      0x12
#define RREG       0x20
#define WREG       0x40

ADS1292R* ADS1292R::instance = nullptr;

/************************************************/

ADS1292R::ADS1292R() :
    spi(FSPI)
{
    dataReady = false;
}

/************************************************/

void IRAM_ATTR ADS1292R::drdyISR()
{
    if(instance)
        instance->dataReady = true;
}

/************************************************/

bool ADS1292R::begin()
{
    instance = this;

    pinMode(ECG_CS_PIN, OUTPUT);
    pinMode(ECG_START_PIN, OUTPUT);
    pinMode(ECG_RESET_PIN, OUTPUT);
    pinMode(ECG_DRDY_PIN, INPUT_PULLUP);

    digitalWrite(ECG_CS_PIN, HIGH);
    digitalWrite(ECG_START_PIN, HIGH);

    spi.begin(
        ECG_SCLK_PIN,
        ECG_MISO_PIN,
        ECG_MOSI_PIN,
        ECG_CS_PIN);

    hardwareReset();

    sendCommand(SDATAC);

    if(getID()!=0x73)
        return false;

    configure();

    attachInterrupt(
        digitalPinToInterrupt(ECG_DRDY_PIN),
        drdyISR,
        FALLING);

    start();

    return true;
}

/************************************************/

void ADS1292R::hardwareReset()
{
    digitalWrite(ECG_RESET_PIN, LOW);
    delay(20);

    digitalWrite(ECG_RESET_PIN, HIGH);
    delay(100);
}

/************************************************/

void ADS1292R::configure()
{
    writeRegister(0x01,0x02);
    writeRegister(0x02,0x80);

    writeRegister(0x03,0x10);

    writeRegister(0x04,0x00);
    writeRegister(0x05,0x00);

    writeRegister(0x06,0x00);
    writeRegister(0x07,0x00);
    writeRegister(0x08,0x00);

    writeRegister(0x09,0x02);
    writeRegister(0x0A,0x02);
    writeRegister(0x0B,0x0F);
}

/************************************************/

void ADS1292R::start()
{
    sendCommand(START_CMD);
    delay(10);

    sendCommand(RDATAC);
}

/************************************************/

void ADS1292R::stop()
{
    sendCommand(SDATAC);
    sendCommand(STOP_CMD);
}

/************************************************/

void ADS1292R::update()
{
}

/************************************************/

bool ADS1292R::available()
{
    return dataReady;
}

/************************************************/

bool ADS1292R::read(ECGData &data)
{
    if(!dataReady)
        return false;

    dataReady=false;

    return readFrame(data);
}

/************************************************/

bool ADS1292R::readFrame(ECGData &data)
{
    spi.beginTransaction(
        SPISettings(
            ECG_SPI_SPEED,
            MSBFIRST,
            SPI_MODE1));

    digitalWrite(ECG_CS_PIN,LOW);

    data.status[0]=spi.transfer(0);
    data.status[1]=spi.transfer(0);
    data.status[2]=spi.transfer(0);

    data.ch1=read24();
    data.ch2=read24();

    digitalWrite(ECG_CS_PIN,HIGH);

    spi.endTransaction();

    data.newData=true;

    return true;
}

/************************************************/

int32_t ADS1292R::read24()
{
    int32_t value=0;

    value|=((int32_t)spi.transfer(0))<<16;
    value|=((int32_t)spi.transfer(0))<<8;
    value|=((int32_t)spi.transfer(0));

    if(value&0x800000)
        value|=0xFF000000;

    return value;
}

/************************************************/

void ADS1292R::sendCommand(uint8_t cmd)
{
    spi.beginTransaction(
        SPISettings(
            ECG_SPI_SPEED,
            MSBFIRST,
            SPI_MODE1));

    digitalWrite(ECG_CS_PIN,LOW);

    spi.transfer(cmd);

    digitalWrite(ECG_CS_PIN,HIGH);

    spi.endTransaction();

    delayMicroseconds(10);
}

/************************************************/

void ADS1292R::writeRegister(uint8_t reg,uint8_t value)
{
    spi.beginTransaction(
        SPISettings(
            ECG_SPI_SPEED,
            MSBFIRST,
            SPI_MODE1));

    digitalWrite(ECG_CS_PIN,LOW);

    spi.transfer(WREG|reg);
    spi.transfer(0x00);
    spi.transfer(value);

    digitalWrite(ECG_CS_PIN,HIGH);

    spi.endTransaction();

    delayMicroseconds(20);
}

/************************************************/

uint8_t ADS1292R::readRegister(uint8_t reg)
{
    spi.beginTransaction(
        SPISettings(
            ECG_SPI_SPEED,
            MSBFIRST,
            SPI_MODE1));

    digitalWrite(ECG_CS_PIN,LOW);

    spi.transfer(RREG|reg);
    spi.transfer(0x00);

    delayMicroseconds(20);

    uint8_t value=spi.transfer(0);

    digitalWrite(ECG_CS_PIN,HIGH);

    spi.endTransaction();

    return value;
}

/************************************************/

uint8_t ADS1292R::getID()
{
    return readRegister(0);
}

/************************************************/

bool ADS1292R::selfTest()
{
    return (getID()==0x73);
}