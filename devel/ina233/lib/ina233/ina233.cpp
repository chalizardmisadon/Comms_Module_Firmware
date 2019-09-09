#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ina233.h"


/* --------------- abstracting i2c read/write --------------- */
// write byte command to ina233
uint8_t Ina233::writeCmd(uint8_t reg)
{
	return I2C->writeBytes(INA233_I2C_ADDR, &reg, 1);
}

// write data to specified register
uint8_t Ina233::writeByte(uint8_t reg, uint8_t data)
{
	return I2C->writeByte(INA233_I2C_ADDR, reg, data);
}
uint8_t Ina233::writeWord(uint8_t reg, uint16_t data)
{
	I2C->setLSByte();
	return I2C->writeByte(INA233_I2C_ADDR, reg, data);
}

/**
 * When reading a block in PMBUS the first byte from the slave is the
 * block size (6 in this case), so the request must be for block_size + 1
 * 
 * Note: INA233 block size is always 6 byte, so we can hardcode it
 */
uint8_t Ina233::readBlock(uint8_t reg, uint8_t data[6])
{
	uint8_t buffer[7];
	if (7 == I2C->readBytes(INA233_I2C_ADDR, sizeof(buffer), reg, buffer)) {
		memcpy(data, &buffer[1], 6);
		return buffer[0];
	}

	memset(data, 0, sizeof(*data));
	return 0;
}

// read data from specified register
uint8_t Ina233::readByte(uint8_t reg)
{
	return I2C->readByte(INA233_I2C_ADDR, reg);
}
uint16_t Ina233::readWord(uint8_t reg)
{
	I2C->setLSByte();
	return I2C->readWord(INA233_I2C_ADDR, reg);
}

/* --------------- initialize ina233 object --------------- */
Ina233::Ina233(I2cBus *bus, uint8_t addr) {
	INA233_I2C_ADDR = addr;
	I2C = bus;

	m_c = 0;
	R_c = 0;
	m_p = 0;
	R_p = 0;
}

void Ina233::begin() {
}

// get model register, return byte array should be "INA233"
uint8_t Ina233::getModel(uint8_t model[6]) {
	return readBlock(MFR_MODEL, model);
}

/* -----get raw value (2-bytes, two's complement integer) ----- */
int16_t Ina233::getBusVoltage_raw() {
	return (int16_t)readWord(READ_VIN);
}

int16_t Ina233::getShuntVoltage_raw() {
	return (int16_t)readWord(MFR_READ_VSHUNT);
}

int16_t Ina233::getCurrent_raw() {
	return (int16_t)readWord(READ_IIN);
}

int16_t Ina233::getPower_raw() {
	return (int16_t)readWord(READ_PIN);
}

// conversion from direct format to real-world dimensions
float Ina233::calcRawToValue(int16_t raw, int16_t m, int16_t b, int8_t R)
{
	return (((float)raw * pow(10, -R)) - b) / (float)m;
}

/* --------------- get reading value --------------- */
float Ina233::getBusVoltage_V()
{
	return calcRawToValue(getBusVoltage_raw(), m_vb, b_vb, R_vb);
}
float Ina233::getShuntVoltage_V()
{
	return calcRawToValue(getShuntVoltage_raw(), m_vs, b_vs, R_vs);
}
float Ina233::getCurrent_A()
{
	return calcRawToValue(getCurrent_raw(), m_c, b_c, R_c);
}
float Ina233::getPower_W()
{
	return calcRawToValue(getPower_raw(), m_p, b_p, R_p);
}

/* --------------- get millis value --------------- */
float Ina233::getPower_mW()
{
	return 1000 * getPower_W();
}
float Ina233::getCurrent_mA()
{
	return 1000 * getCurrent_A();
}
float Ina233::getBusVoltage_mV()
{
	return 1000 * getBusVoltage_V();
}
float Ina233::getShuntVoltage_mV()
{
	return 1000 * getShuntVoltage_V();
}

/* --------------- set ina233 calibration --------------- */
uint16_t Ina233::setCalibration(float r_shunt, float max_i_expect)
{
	float Current_LSB = max_i_expect / ((uint16_t)1<<15);
	// float   Power_LSB = 25 * Current_LSB;

	// calculate floating point, then try to convert
	// to nearest 16-bit unsigned integer
	float  f_CAL = 0.00512 / (Current_LSB * r_shunt);
	uint16_t CAL = 0;
	if (f_CAL < 0xFFFF)
		CAL = (uint16_t)(CAL + 0.5);

	// write calibration value
	writeWord(MFR_CALIBRATION, CAL);
	return CAL;
}

uint8_t Ina233::setMFR_CALIBRATION(float r_shunt, float max_i_expect)
{
	return writeWord(
		MFR_CALIBRATION,
		calcCAL(
			r_shunt,
			calcCurrent_LSB(max_i_expect)
		)
	);
}

/* --------------- calculate CAL value --------------- */
float Ina233::calcCurrent_LSB(float max_i_expect)
{
	return max_i_expect / ((uint16_t)1<<15);
}

float Ina233::calcPower_LSB(float Current_LSB)
{
	return 25 * Current_LSB;
}

uint16_t Ina233::calcCAL(float r_shunt, float Current_LSB)
{
	// calculate floating point, then try to convert
	// to nearest 16-bit unsigned integer
	float  f_CAL = 0.00512 / (Current_LSB * r_shunt);
	uint16_t CAL = 0;
	if (f_CAL < 0xFFFF) CAL = (uint16_t)(CAL + 0.5);
	return CAL;
}


/* --------------- get Energy and Average Power value --------------- */
/**
 * @brief      Gets the raw energy info
 *
 * @param      accumulator   The accumulator  (2-byte)
 * @param      roll_over     The roll over    (1-byte)
 * @param      sample_count  The sample count (3-byte)
 *
 * @return     The number of bytes read, which should be 6.
 */
uint8_t Ina233::getEnergy_raw(uint16_t *accumulator, uint8_t *roll_over, uint32_t *sample_count)
{
	uint8_t data[6];
	uint8_t bytesRead = readBlock(READ_EIN, data);

	*sample_count = (uint32_t)data[5] << 16 |
	                (uint32_t)data[4] << 8 |
	                (uint32_t)data[3];
	*roll_over    = (uint8_t) data[2];
	*accumulator  = (uint16_t)data[1] << 8 |
	                (uint16_t)data[0];

	return bytesRead;
}

/**
 * @brief      Gets the average power based on data sheet equation
 * @return     The average power in Watts
 */
float Ina233::getAveragePower_W()
{
	uint16_t accumulator = 0;
	uint8_t  roll_over = 0;
	uint32_t sample_count = 0;

	// if bytes read was not 6 bytes, then read failed
	if (6 != getEnergy_raw(&accumulator, &roll_over, &sample_count))
		return 0;
	
	// calculate total accumulated unscaled power
	uint32_t accumulator_24 = (uint32_t)roll_over * (uint32_t)(1<<16) + (uint32_t)accumulator;
	uint32_t raw_avg_power = accumulator_24 / sample_count;

	return calcRawToValue(raw_avg_power, m_p, b_p, R_p);
}

float Ina233::getAveragePower_mW()
{
	return 1000 * getAveragePower_W();
}









#if 0
/**************************************************************************/
/*!
		@brief  Set INA233 Calibration register for measuring based on the user's
		inputs r_shunt and i_max.
		-inputs: value of the shunt resistor and maximum current (in ohms and A)
		-inputs as outputs: measuring accuracy for current (uA) and power (mW) and
		ERROR state for possible errors during Calibration.
		-outputs: the CAL value to be written in MFR_CALIBRATION

		*/
/**************************************************************************/
uint16_t Ina233::setCalibrationOld(
	float r_shunt, float max_i_expect,
	float *Current_LSB, float *Power_LSB,
	int16_t *mc, int8_t *Rc, int16_t *mp, int8_t *Rp,
	uint8_t *ERROR)
{

	float m_c_F = 1 / C_LSB;
	float m_p_F = 1 / P_LSB;

	int32_t aux = 0;
	bool round_done = false;
	int8_t local_R_c = 0;
	int8_t local_R_p = 0;
	uint8_t local_ERROR = 0;

	
	
	*Current_LSB=C_LSB*1000000;
	*Power_LSB=P_LSB*1000;
	


	//Calculate m and R for maximum accuracy in current measurement
	aux=(int32_t)m_c_F;
	while ((aux>32768)||(aux<-32768))
		{
			m_c_F=m_c_F/10;
			local_R_c++;
			aux=(int32_t)m_c_F;
		}
	while (round_done==false)
		{
			aux=(int32_t)m_c_F;
			if (aux==m_c_F)
			{
				round_done=true;
			}
			else
			{
				 aux=(int32_t)(m_c_F*10);             //shift decimal to the right
				 if ((aux>32768)||(aux<-32768))       //m_c is out of int16 (-32768 to 32768)
				 {
					round_done=true;
				 }
				 else
				 {
					m_c_F=m_c_F*10;
					local_R_c--;
				 }
			}
		}
	round_done=false;
	//Calculate m and R for maximum accuracy in power measurement
	aux=(int32_t)m_p_F;
	while ((aux>32768)||(aux<-32768))
		{
			m_p_F=m_p_F/10;
			local_R_p++;
			aux=(int32_t)m_p_F;
		}
	while (round_done==false)
		{
			aux=(int32_t)m_p_F;
			if (aux==m_p_F)
			{
				round_done=true;
			}
			else
			{
				 aux=(int32_t)(m_p_F*10);          //shift decimal to the right
				 if ((aux>32768)||(aux<-32768))       //m_p is out of int16 (-32768 to 32768)
				 {
					round_done=true;
				 }
				 else
				 {
					m_p_F=m_p_F*10;
					local_R_p--;
				 }
			}
		}
	*mp=m_p_F;
	*mc=m_c_F;
	*Rc=local_R_c;
	*Rp=local_R_p;
	*ERROR=local_ERROR;

	m_c=int16_t(m_c_F);
	m_p=int16_t(m_p_F);
	R_c=local_R_c;
	R_p=local_R_p;

	return(uint16_t)CAL;
}
/**************************************************************************/
/*!
		@brief  Instantiates a new INA233 class
*/
/**************************************************************************/
#endif



