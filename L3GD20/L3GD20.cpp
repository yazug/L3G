#include <L3GD20.h>
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////
#define _MULTI_REGISTER_GYRO_READ

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
#define L3GD20_ADDR_SEL_LOW	0x6a
#define L3GD20_ADDR_SEL_HIGH	0x6b

// Public Methods //////////////////////////////////////////////////////////////

L3GD20::L3GD20()
{
	I2CAddr = L3GD20_ADDR_SEL_HIGH;
}


uint8_t L3GD20::setSELState(uint8_t SELState)
{
	uint8_t ReturnValue = 0;

	if(0 == SELState)
	{
		I2CAddr = L3GD20_ADDR_SEL_LOW;
	}
	else if(1 == SELState)
	{
		I2CAddr = L3GD20_ADDR_SEL_HIGH;
	}

	return ReturnValue;
}


// Turns on the L3GD20's gyro and places it in normal mode.
void L3GD20::enableDefault(void)
{
	// 0x0F = 0b00001111
	// Normal power mode, all axes enabled
	writeReg(L3GD20_CTRL_REG1, 0x0F);
}


// Writes a gyro register
void L3GD20::writeReg(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(I2CAddr);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}


// Reads a gyro register
uint8_t L3GD20::readReg(uint8_t reg)
{
	uint8_t value;
	
	Wire.beginTransmission(I2CAddr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(I2CAddr, 1);

	while(!Wire.available());

	value = Wire.read();
	Wire.endTransmission();
	
	return value;
}

// Reads the 3 gyro channels 
void L3GD20::readGyro(int16_t *pX, int16_t *pY, int16_t *pZ)
{
	Wire.beginTransmission(I2CAddr);

#ifdef _MULTI_REGISTER_GYRO_READ
	// assert the MSB of the address to get the gyro 
	// to do slave-transmit subaddress updating.
	Wire.write(L3GD20_OUT_X_L | (1 << 7)); 
	Wire.endTransmission();
	Wire.requestFrom(I2CAddr, 6);

	while (Wire.available() < 6);
	
	uint8_t xla = Wire.read();
	uint8_t xha = Wire.read();
	uint8_t yla = Wire.read();
	uint8_t yha = Wire.read();
	uint8_t zla = Wire.read();
	uint8_t zha = Wire.read();

#else

	uint8_t xla = readReg(L3GD20_OUT_X_L);
	uint8_t xha = readReg(L3GD20_OUT_X_H);
	uint8_t yla = readReg(L3GD20_OUT_Y_L);
	uint8_t yha = readReg(L3GD20_OUT_Y_H);
	uint8_t zla = readReg(L3GD20_OUT_Z_L);
	uint8_t zha = readReg(L3GD20_OUT_Z_H);

#endif	//_AUTO_ADDRESS_UPDATE

	*pX = (int16_t)(xha << 8 | xla);
	*pY = (int16_t)(yha << 8 | yla);
	*pZ = (int16_t)(zha << 8 | zla);
}


void L3GD20::readTemperature(int8_t *pTemperature)
{
	uint8_t RegisterValue;

	RegisterValue = readReg(L3GD20_OUT_TEMP);

	*pTemperature = (int8_t)RegisterValue;
}


void L3GD20::setFullScaleRange(uint8_t FullScaleRange)
{
	uint8_t RegisterValue;

	RegisterValue = readReg(L3GD20_CTRL_REG4);

	RegisterValue &= ~(0x30);
	RegisterValue |= ((FullScaleRange & 0x03) << 4);

	writeReg(L3GD20_CTRL_REG4, RegisterValue);
}


void L3GD20::setBandwidth(uint8_t Bandwidth)
{
	uint8_t RegisterValue;

	RegisterValue = readReg(L3GD20_CTRL_REG1);

	RegisterValue &= ~(0x30);
	RegisterValue |= ((Bandwidth & 0x03) << 4);

	writeReg(L3GD20_CTRL_REG1, RegisterValue);
}


void L3GD20::setOutputDataRate(uint8_t DataRate)
{
	uint8_t RegisterValue;

	RegisterValue = readReg(L3GD20_CTRL_REG1);

	RegisterValue &= ~(0xC0);
	RegisterValue |= ((DataRate & 0x03) << 6);

	writeReg(L3GD20_CTRL_REG1, RegisterValue);
}


