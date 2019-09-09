#ifndef INA233_H
#define INA233_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "i2c_bus.h"

/* I2C addresses/bits */
#define INA233_ADDRESS_40			(0x40)	// 1000000 (A1+A0=GND)
#define INA233_ADDRESS_41			(0x41)	// 1000001 (A1=GND, A0=VDD)
#define INA233_ADDRESS_42			(0x42)	// 1000010 (A1=GND, A0=SDA)
#define INA233_ADDRESS_43			(0x43)	// 1000011 (A1=GND, A0=SCL)
#define INA233_ADDRESS_44			(0x44)	// 1000100 (A1=VDD, A0=GND)
#define INA233_ADDRESS_45			(0x45)	// 1000101 (A1+A0=VDD)
#define INA233_ADDRESS_46			(0x46)	// 1000110 (A1=VDD, A0=SDA)
#define INA233_ADDRESS_47			(0x47)	// 1000111 (A1=VDD, A0=SCL)
#define INA233_ADDRESS_48			(0x48)	// 1001000 (A1=SDA, A0=GND)
#define INA233_ADDRESS_49			(0x49)	// 1001001 (A1=SDA, A0=VDD)
#define INA233_ADDRESS_4A			(0x4A)	// 1001010 (A1+A0=SDA)
#define INA233_ADDRESS_4B			(0x4B)	// 1001011 (A1=SDA, A0=SCL)
#define INA233_ADDRESS_4C			(0x4C)	// 1001100 (A1=SCL, A0=GND)
#define INA233_ADDRESS_4D			(0x4D)	// 1001101 (A1=SCL, A0=VDD)
#define INA233_ADDRESS_4E			(0x4E)	// 1001110 (A1=SCL, A0=SDA)
#define INA233_ADDRESS_4F			(0x4F)	// 1001111 (A1+A0=SCL)

/*PMBus Commands based on ina233 Datasheet
NAME 							CODE 		R/W 		#Bytes 	Default value
---------------------------------------------------------------------------------------------------- */
#define CLEAR_FAULTS				(0x03)	// Send byte 	0 		N/A
#define RESTORE_DEFAULT_ALL			(0x12)	// Send byte 	0 		N/A
#define CAPABILITY					(0x19)	// R 			1 		xB0
#define IOUT_OC_WARN_LIMIT			(0x4A)	// R/W 			2 		x7FF8
#define VIN_OV_WARN_LIMIT			(0x57)  // R/W 			2 		x7FF8
#define VIN_UV_WARN_LIMIT			(0x58)  // R/W 			2 		x0000
#define PIN_OP_WARN_LIMIT			(0x6B)  // R/W 			2 		x7FF8
#define STATUS_BYTE					(0x78)  // R 			1 		x00
#define STATUS_WORD					(0x79)  // R 			2 		x1000
#define STATUS_IOUT					(0x7B)  // R/W,CLR 		1 		x00
#define STATUS_INPUT				(0x7C)  // R/W,CLR 		1 		x00
#define STATUS_CML					(0x7E)  // R/W,CLR 		1 		x00
#define STATUS_MFR_SPECIFIC			(0x80)  // R/W,CLR 		1 		x20
#define READ_EIN					(0x86)	// Block_R 		6 		x00, x00, x00, x00, x00, x00
#define READ_VIN					(0x88)	// R 			2 		x0000
#define READ_IIN					(0x89)	// R 			2 		x0000
#define READ_VOUT					(0x8B)	// R 			2 		x0000
#define READ_IOUT					(0x8C)	// R 			2 		x0000
#define READ_POUT					(0x96)	// R 			2 		x0000
#define READ_PIN					(0x97)	// R 			2 		x0000
#define MFR_ID						(0x99)	// Block_R 		2 		x54, x49
#define MFR_MODEL					(0x9A)	// Block_R 		6 		x49, x4E, x41, x32, x33, x33
#define MFR_REVISION				(0x9B)	// R 			2 		x41, x30
#define MFR_ADC_CONFIG				(0xD0)	// R/W 			2 		x4127
#define MFR_READ_VSHUNT				(0xD1)	// R 			2 		x0000
#define MFR_ALERT_MASK				(0xD2)	// R/W 			1 		xF0
#define MFR_CALIBRATION				(0xD4)	// R/W 			2 		x0001
#define MFR_DEVICE_CONFIG			(0xD5)	// R/W 			1 		x02
#define CLEAR_EIN					(0xD6)	// Send byte 	0 		N/A
#define TI_MFR_ID					(0xE0)	// R 			2 		x5449 (ASCII TI)
#define TI_MFR_MODEL				(0xE1)	// R 			2 		ASCII 33
#define TI_MFR_REVISION				(0xE2)	// R 			2 		ASCII A0

/*=========================================================================
	SHUNT VOLTAGE TELEMETRY & WARNING COEFFICIENTS
	-----------------------------------------------------------------------*/
	#define m_vs					(4)
	#define R_vs					(5)
	#define b_vs					(0)
/*=========================================================================*/

/*=========================================================================
	BUS VOLTAGE TELEMETRY & WARNING COEFFICIENTS
	-----------------------------------------------------------------------*/
	#define m_vb					(8)
	#define R_vb					(2)
	#define b_vb					(0)
/*=========================================================================*/

/*=========================================================================
	CURRENT & POWER CONSTANT TELEMETRY & WARNING COEFFICIENTS
	-----------------------------------------------------------------------*/
	#define b_c						(0)
	#define b_p						(0)
/*=========================================================================*/


class Ina233 {

public:
	/* ----- initialize ina233 ----- */
	/**
	 * @brief      Constructs an ina233 object.
	 *
	 * @param      bus   The pointer to an i2c bus
	 * @param[in]  addr  The address of the ina233
	 */
	Ina233(I2cBus *bus, uint8_t addr = INA233_ADDRESS_40);
	void begin(void);

	/**
	 * @brief      Gets the model.
	 *
	 * @param      model  To detect if an ina233 chip is present.
	 *                    The byte array value should be "INA233"
	 * 
	 * @return     { block size read, in this case: 6 }
	 */
	uint8_t getModel(uint8_t model[6]);

	/* ---------- get data functions ---------- */
	float getBusVoltage_V(void);
	float getBusVoltage_mV(void);
	float getShuntVoltage_V(void);
	float getShuntVoltage_mV(void);
	float getCurrent_A(void);
	float getCurrent_mA(void);
	float getPower_W(void);
	float getPower_mW(void);

	/* ---------- get raw data ---------- */
	float calcRawToValue(int16_t raw, int16_t m, int16_t b, int8_t R);
	int16_t getBusVoltage_raw(void);
	int16_t getShuntVoltage_raw(void);
	int16_t getCurrent_raw(void);
	int16_t getPower_raw(void);

	/* ---------- energy and average power data ---------- */
	float getAveragePower_W(void);
	float getAveragePower_mW(void);
	uint8_t getEnergy_raw(uint16_t* accumulator, uint8_t* roll_over, uint32_t* sample_count);

	/* ----- ina233 set data functions ----- */
	uint16_t setCalibration(float r_shunt, float max_i_expect);
	uint8_t  setMFR_CALIBRATION(float r_shunt, float max_i_expect);

	/* ---------- calculate coefficients ---------- */
	float calcCurrent_LSB(float max_i_expect);
	float calcPower_LSB(float Current_LSB);
	uint16_t calcCAL(float r_shunt, float Current_LSB);

	/* ----- ina233 read/write byte functions ----- */
	/**
	 * These functions are more lower level, and
	 * are mostly used by the getters/setters above
	 */

	// write byte command to ina233
	uint8_t writeCmd(uint8_t cmd);

	// write data to specified register
	uint8_t writeByte(uint8_t reg, uint8_t data);
	uint8_t writeWord(uint8_t reg, uint16_t data);

	//read data from the specified register
	uint8_t readBlock(uint8_t reg, uint8_t value[6]);
	uint8_t readByte(uint8_t reg);
	uint16_t readWord(uint8_t reg);



private:
	I2cBus *I2C;

	uint8_t INA233_I2C_ADDR;
	uint32_t ina233_calValue;

	/**
	 * The following coefficients are used to convert raw current and power
	 * values to mA and mW, taking into account the current config settings
	 */
	int16_t m_c;
	int8_t R_c;
	int16_t m_p;
	int8_t R_p;
};

#ifdef __cplusplus
}
#endif

#endif