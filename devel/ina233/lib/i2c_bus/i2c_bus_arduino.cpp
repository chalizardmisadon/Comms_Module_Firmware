/**
 * The source file of the i2c wrapper is different
 * for each platform. This source file wraps the
 * Arduino platform's TwoWire library.
 */

#include <stdint.h>

/* ----- define i2c implementation ----- */
#include "Wire.h"
typedef TwoWire I2cBusImple;

#define I2cBusImple_DEFINE
#include "i2c_bus.h"
#define Wire (*I2cBusImpleArg)

#define default_SCL PB10
#define default_SDA PB11

/* ----- initialize i2c bus ----- */
I2cBus::I2cBus(void)
{
	I2cBusImpleArg = new I2cBusImple;
	Is_MSByte = true;

	setSCL(default_SCL);
	setSDA(default_SDA);
}

I2cBus::I2cBus(uint8_t scl_pin, uint8_t sda_pin)
{
	I2cBusImpleArg = new I2cBusImple;
	Is_MSByte = true;

	setSCL(scl_pin);
	setSDA(sda_pin);
}

I2cBus::~I2cBus()
{
	// delete I2cBusImpleArg;
}

void I2cBus::begin(void)
{
	Wire.begin();
}

void I2cBus::setSCL(uint8_t scl_pin)
{
	Wire.setSCL((uint32_t)scl_pin);
}

void I2cBus::setSDA(uint8_t sda_pin)
{
	Wire.setSDA((uint32_t)sda_pin);
}

void I2cBus::setClock(uint32_t frequency)
{
	Wire.setClock(frequency);
}
void I2cBus::beginTransmission(uint8_t addr)
{
	Wire.beginTransmission(addr);
}
uint8_t I2cBus::endTransmission(void)
{
	return Wire.endTransmission();
}


/* ---------- read/write i2c functions ---------- */
uint8_t I2cBus::readBytes(uint8_t addr, uint8_t len, uint32_t reg, uint8_t regsize, uint8_t *data)
{
    uint8_t read = 0;
	read = Wire.requestFrom(addr, len, reg, regsize, (uint8_t)true);
	for (uint8_t i = 0; i < read; ++i) data[i] = Wire.read();
	return read;
}

uint8_t I2cBus::writeBytes(uint8_t addr, uint8_t *data, uint8_t len)
{
	Wire.beginTransmission(addr);
	Wire.write(data, len);
	return Wire.endTransmission();
}


/* ---------- read data functions ---------- */
uint8_t I2cBus::setMSByte(void) { return Is_MSByte = true; }
uint8_t I2cBus::setLSByte(void) { return Is_MSByte = false; }

#define read_uintX(dataType, dataLen) \
do { \
	uint8_t buffer[dataLen] = {0}; \
	dataType data = 0; \
	readBytes(addr, dataLen, reg, buffer); \
	if (Is_MSByte) \
		for (uint8_t i = 0; i < dataLen; ++i) data = buffer[i] | data << 8; \
	else \
		for (uint8_t i = 0; i < dataLen; ++i) data = buffer[dataLen-1-i] | data << 8; \
	return data; \
} while (0)

uint8_t I2cBus::read_uint8(uint8_t addr, uint8_t reg)
{
	read_uintX(uint8_t, 1);
}

uint16_t I2cBus::read_uint16(uint8_t addr, uint8_t reg)
{
	read_uintX(uint16_t, 2);
}

uint32_t I2cBus::read_uint32(uint8_t addr, uint8_t reg)
{
	read_uintX(uint32_t, 4);
}

uint64_t I2cBus::read_uint64(uint8_t addr, uint8_t reg)
{
	read_uintX(uint64_t, 8);
}


/* ---------- write data functions ---------- */
#define write_uintX(dataLen) \
do { \
	uint8_t buffer[dataLen+1] = {0}; \
	buffer[0] = reg; \
	if (Is_MSByte) \
		for (uint8_t i = 0; i < dataLen; ++i) buffer[dataLen-i] = (data >> (i*8)) & 0xff; \
	else \
		for (uint8_t i = 0; i < dataLen; ++i) buffer[i+1] = (data >> (i*8)) & 0xff; \
	return writeBytes(addr, buffer, dataLen); \
} while (0)

uint8_t I2cBus::write_uint8(uint8_t addr, uint8_t reg, uint8_t data)
{
	write_uintX(1);
}

uint8_t I2cBus::write_uint16(uint8_t addr, uint8_t reg, uint16_t data)
{
	write_uintX(2);
}

uint8_t I2cBus::write_uint32(uint8_t addr, uint8_t reg, uint32_t data)
{
	write_uintX(4);
}
uint8_t I2cBus::write_uint64(uint8_t addr, uint8_t reg, uint64_t data)
{
	write_uintX(8);
}


