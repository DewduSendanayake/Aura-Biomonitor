#include "ADS1292R.h"
#include "Config.h"

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
    leadOffDetected = true;  // Assume off-body until first valid read
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

    // Verify critical registers were written correctly
    DBG_PRINTLN("ADS1292R Register Readback:");
    DBG_PRINTF("  CONFIG2 (0x02): 0x%02X (expect 0xC0)\r\n", readRegister(0x02));
    DBG_PRINTF("  LOFF    (0x03): 0x%02X (expect 0x00)\r\n", readRegister(0x03));
    DBG_PRINTF("  LOFF_SENS(0x07): 0x%02X (expect 0x14)\r\n", readRegister(0x07));
    DBG_PRINTF("  LOFF_STAT(0x08): 0x%02X\r\n", readRegister(0x08));

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
    writeRegister(0x01,0x02);   // CONFIG1: continuous, 500 SPS
    writeRegister(0x02,0xC0);   // CONFIG2: lead-off comparator ON, internal ref

    writeRegister(0x03,0x00);   // LOFF: DC lead-off, 6nA, 95%/5% threshold

    writeRegister(0x04,0x00);   // CH1SET: normal input, gain 6
    writeRegister(0x05,0x00);   // CH2SET: normal input, gain 6

    writeRegister(0x06,0x00);   // RLD_SENS: disabled
    writeRegister(0x07,0x14);   // LOFF_SENS: bit4=SENSP1, bit2=SENSN1 → CH1 lead-off enabled
    writeRegister(0x08,0x00);   // LOFF_STAT: read-only

    writeRegister(0x09,0x02);   // RESP1
    writeRegister(0x0A,0x02);   // RESP2
    writeRegister(0x0B,0x0F);   // GPIO
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

    // Parse lead-off status from the 24-bit status word:
    // status[0] bits 23:16 -> Bit 18 (0x04) = IN2N_OFF, Bit 17 (0x02) = IN2P_OFF, Bit 16 (0x01) = IN1N_OFF
    // status[1] bits 15:8  -> Bit 15 (0x80) = IN1P_OFF
    // Only check Channel 1 (Chest strap electrodes)
    // Channel 2 is unconnected and will always trigger lead-off if checked
    bool in1n_off = (data.status[0] & 0x01) != 0;
    bool in1p_off = (data.status[1] & 0x80) != 0;

    leadOffDetected = in1n_off || in1p_off;
    data.leadOff = leadOffDetected;

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

/************************************************/

bool ADS1292R::isLeadOff() const
{
    return leadOffDetected;
}