/**
 * This library is a wrapper for a single i2c bus:
 * a single (SCL, SDA) two-wire pair
 * 
 * To use multiple i2c bus, initialize multiple
 * I2cBus objects with different two-wire pair
 * 
 * The concept design is this:
 * 1. create an i2c bus on a two-wire pair
 * 2. each i2c device selects the i2c bus
 */


#ifndef I2C_BUS_H
#define I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef I2cBusImple_DEFINE
class   I2cBusImple;
#endif

class I2cBus {

public:
	/* ----- initialize i2c bus ----- */
	I2cBus(void);
	I2cBus(uint8_t scl_pin, uint8_t sda_pin);
	~I2cBus(void);
	void begin(void);
	void setSCL(uint8_t scl_pin);
	void setSDA(uint8_t sda_pin);
	void setClock(uint32_t frequency);

	/* ----- Arduino transmission functions ----- */
	void beginTransmission(uint8_t addr);
	inline void beginTransmission(int addr) {
		beginTransmission((uint8_t)addr);
	}
	uint8_t endTransmission(void);

	/* ---------- read/write i2c functions ---------- */
	/**
	 * @brief      read bytes from register
	 *
	 * @param[in]  addr     The address of the device
	 * @param[in]  len      The length of the data being read
	 * @param[in]  reg      The register address (most significant byte first)
	 * @param[in]  regsize  The register address size
	 * @param      data     The bytes returned are stored in order of returned
	 *
	 * @return     { number of bytes read (length if read successful, 0 if fail) }
	 */
	uint8_t readBytes(uint8_t addr, uint8_t len, uint32_t reg, uint8_t regsize, uint8_t *data);

	inline
	uint8_t readBytes(uint8_t addr, uint8_t len, uint8_t reg, uint8_t *data) {
		return readBytes(addr, len, reg, (uint8_t)1, data);
	}

	/**
	 * @brief      Writes bytes to device
	 *
	 * @param[in]  addr     The address of the device
	 * @param      data     The data to send
	 * @param[in]  len      The length of the data
	 *
	 * @return     { return error message, 0 for no error }
	 */
	uint8_t writeBytes(uint8_t addr, uint8_t *data, uint8_t len);


	/* ---------- read data functions ---------- */

	/**
	 * @brief      Set the byte endian. Call to change how
	 *             read/write data functions pack i2c byte array.
	 *             If MSByte: first byte read/write is most significant
	 *             if LSByte: first byte read/write is least significant
	 * @return     { return the value of Is_MSByte after setting it }
	 */
	uint8_t setMSByte(void);
	uint8_t setLSByte(void);

	uint8_t read_uint8(uint8_t addr, uint8_t reg);
	uint16_t read_uint16(uint8_t addr, uint8_t reg);
	uint32_t read_uint32(uint8_t addr, uint8_t reg);
	uint64_t read_uint64(uint8_t addr, uint8_t reg);

	inline uint8_t readByte(uint8_t addr, uint8_t reg) {
		return read_uint8(addr, reg);
	}
	inline uint16_t readWord(uint8_t addr, uint8_t reg) {
		return read_uint16(addr, reg);
	}
	inline uint32_t readDWord(uint8_t addr, uint8_t reg) {
		return read_uint32(addr, reg);
	}
	inline uint64_t readQWord(uint8_t addr, uint8_t reg) {
		return read_uint64(addr, reg);
	}


	/* ---------- write data functions ---------- */
	/**
	 * @brief      Write data to the specified register of the device
	 *             Note: the first byte being send is the register
	 *             address, then send subsequent data bytes
	 *
	 * @param[in]  addr  The address of the device
	 * @param[in]  reg   The register to write data
	 * @param[in]  data  The data
	 *
	 * @return     { return error message, 0 for no error }
	 */
	uint8_t write_uint8(uint8_t addr, uint8_t reg, uint8_t data);
	uint8_t write_uint16(uint8_t addr, uint8_t reg, uint16_t data);
	uint8_t write_uint32(uint8_t addr, uint8_t reg, uint32_t data);
	uint8_t write_uint64(uint8_t addr, uint8_t reg, uint64_t data);

	inline uint8_t writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
		return write_uint8(addr, reg, data);
	}
	inline uint8_t writeWord(uint8_t addr, uint8_t reg, uint16_t data) {
		return write_uint16(addr, reg, data);
	}
	inline uint8_t writeDWord(uint8_t addr, uint8_t reg, uint32_t data) {
		return write_uint32(addr, reg, data);
	}
	inline uint8_t writeQWord(uint8_t addr, uint8_t reg, uint64_t data) {
		return write_uint64(addr, reg, data);
	}


private:
	bool Is_MSByte;
	I2cBusImple *I2cBusImpleArg;

};

#ifdef __cplusplus
}
#endif

#endif