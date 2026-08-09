#include "ADS1292R.h"
#include "Config.h"

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

ADS1292R::ADS1292R() : spi(FSPI)
{
    dataReady = false;
    leadOffDetected = true;
}

void IRAM_ATTR ADS1292R::drdyISR()
{
    if(instance)
        instance->dataReady = true;
}

bool ADS1292R::begin()
{
    instance = this;
    dataReady = false;

    pinMode(ECG_CS_PIN, OUTPUT);
    pinMode(ECG_START_PIN, OUTPUT);
    pinMode(ECG_RESET_PIN, OUTPUT);
    pinMode(ECG_DRDY_PIN, INPUT_PULLUP);

    digitalWrite(ECG_CS_PIN, HIGH);
    digitalWrite(ECG_START_PIN, HIGH);

    spi.begin(ECG_SCLK_PIN, ECG_MISO_PIN, ECG_MOSI_PIN, ECG_CS_PIN);

    hardwareReset();

    sendCommand(SDATAC);

    if(getID() != 0x73)
        return false;

    configure();

    DBG_PRINTLN("ADS1292R Register Readback:");
    DBG_PRINTF("  CONFIG1   (0x01): 0x%02X (expect 0x02)\r\n", readRegister(0x01));
    DBG_PRINTF("  CONFIG2   (0x02): 0x%02X (expect 0xE0)\r\n", readRegister(0x02));
    DBG_PRINTF("  LOFF      (0x03): 0x%02X (expect 0x10)\r\n", readRegister(0x03));
    DBG_PRINTF("  CH1SET    (0x04): 0x%02X (expect 0x00)\r\n", readRegister(0x04));
    DBG_PRINTF("  RLD_SENS  (0x06): 0x%02X (expect 0x2C)\r\n", readRegister(0x06));
    DBG_PRINTF("  LOFF_SENS (0x07): 0x%02X (expect 0x03)\r\n", readRegister(0x07));
    DBG_PRINTF("  LOFF_STAT (0x08): 0x%02X\r\n", readRegister(0x08));

    attachInterrupt(digitalPinToInterrupt(ECG_DRDY_PIN), drdyISR, FALLING);

    start();
    return true;
}

void ADS1292R::hardwareReset()
{
    digitalWrite(ECG_RESET_PIN, LOW);
    delay(20);
    digitalWrite(ECG_RESET_PIN, HIGH);
    delay(100);
}

void ADS1292R::configure()
{
    // Continuous conversion at 500 SPS.
    writeRegister(0x01, 0x02);

    // Lead-off comparators and internal 2.42 V reference buffer enabled.
    writeRegister(0x02, 0xE0);

    // DC lead-off detection, 6 nA excitation current. Bit 4 is reserved and
    // must remain 1 for this register configuration.
    writeRegister(0x03, 0x10);

    // Normal electrode input, PGA gain 6.
    writeRegister(0x04, 0x00);
    writeRegister(0x05, 0x00);

    // Enable RLD buffer and derive RLD from CH2P/CH2N.
    writeRegister(0x06, 0x2C);

    // Lead-off detection on Channel 1 positive and negative electrodes.
    writeRegister(0x07, 0x03);

    // Respiration is derived from the MPU6500 in this firmware.
    writeRegister(0x09, 0x02);
    writeRegister(0x0A, 0x03);
    writeRegister(0x0B, 0x0F);
}

void ADS1292R::start()
{
    // Clear any stale DRDY event before entering continuous data mode.
    noInterrupts();
    dataReady = false;
    interrupts();

    sendCommand(START_CMD);
    delay(10);
    sendCommand(RDATAC);
}

void ADS1292R::stop()
{
    sendCommand(SDATAC);
    sendCommand(STOP_CMD);

    noInterrupts();
    dataReady = false;
    interrupts();
}

void ADS1292R::update() {}

bool ADS1292R::available()
{
    return dataReady;
}

bool ADS1292R::read(ECGData &data)
{
    bool ready;

    noInterrupts();
    ready = dataReady;
    dataReady = false;
    interrupts();

    if(!ready)
        return false;

    return readFrame(data);
}

bool ADS1292R::readFrame(ECGData &data)
{
    // One DRDY event corresponds to one current conversion frame. Read exactly
    // the 3 status bytes + 2 channels (6 bytes), then release CS.
    spi.beginTransaction(SPISettings(ECG_SPI_SPEED, MSBFIRST, SPI_MODE1));
    digitalWrite(ECG_CS_PIN, LOW);

    data.status[0] = spi.transfer(0x00);
    data.status[1] = spi.transfer(0x00);
    data.status[2] = spi.transfer(0x00);
    data.ch1 = read24();
    data.ch2 = read24();

    digitalWrite(ECG_CS_PIN, HIGH);
    spi.endTransaction();

    // The ADS1292R status word has a fixed 1100 prefix. If that prefix is not
    // present, the frame is not trustworthy and must not reach QRS/HRV code.
    const bool validStatus = (data.status[0] & 0xF0) == 0xC0;

    if(!validStatus)
    {
        data.newData = false;
        data.leadOff = true;
        leadOffDetected = true;

        DBG_PRINTF("ADS1292R INVALID FRAME: %02X %02X %02X\r\n",
            data.status[0], data.status[1], data.status[2]);

        return false;
    }

    // LOFF_STAT bits 1:0 are IN1N_OFF and IN1P_OFF.
    const bool in1p_off = (data.status[2] & 0x01) != 0;
    const bool in1n_off = (data.status[2] & 0x02) != 0;

    leadOffDetected = in1p_off || in1n_off;
    data.leadOff = leadOffDetected;
    data.newData = true;

    return true;
}

int32_t ADS1292R::read24()
{
    int32_t value = 0;

    value |= ((int32_t)spi.transfer(0x00)) << 16;
    value |= ((int32_t)spi.transfer(0x00)) << 8;
    value |= ((int32_t)spi.transfer(0x00));

    if(value & 0x800000)
        value |= 0xFF000000;

    return value;
}

void ADS1292R::sendCommand(uint8_t cmd)
{
    spi.beginTransaction(SPISettings(ECG_SPI_SPEED, MSBFIRST, SPI_MODE1));
    digitalWrite(ECG_CS_PIN, LOW);
    spi.transfer(cmd);
    digitalWrite(ECG_CS_PIN, HIGH);
    spi.endTransaction();
    delayMicroseconds(10);
}

void ADS1292R::writeRegister(uint8_t reg, uint8_t value)
{
    spi.beginTransaction(SPISettings(ECG_SPI_SPEED, MSBFIRST, SPI_MODE1));
    digitalWrite(ECG_CS_PIN, LOW);
    spi.transfer(WREG | reg);
    spi.transfer(0x00);
    spi.transfer(value);
    digitalWrite(ECG_CS_PIN, HIGH);
    spi.endTransaction();
    delayMicroseconds(20);
}

uint8_t ADS1292R::readRegister(uint8_t reg)
{
    spi.beginTransaction(SPISettings(ECG_SPI_SPEED, MSBFIRST, SPI_MODE1));
    digitalWrite(ECG_CS_PIN, LOW);
    spi.transfer(RREG | reg);
    spi.transfer(0x00);
    delayMicroseconds(20);
    uint8_t value = spi.transfer(0x00);
    digitalWrite(ECG_CS_PIN, HIGH);
    spi.endTransaction();
    return value;
}

uint8_t ADS1292R::getID()
{
    return readRegister(0);
}

bool ADS1292R::selfTest()
{
    return getID() == 0x73;
}

bool ADS1292R::isLeadOff() const
{
    return leadOffDetected;
}
