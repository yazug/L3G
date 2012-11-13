#ifndef L3GD20_h
#define L3GD20_h

#include <Arduino.h> // for byte data type

// device types

#define L3G_DEVICE_AUTO 0
#define L3G4200D_DEVICE 1
#define L3GD20_DEVICE   2


// SA0 states

#define L3G_SA0_LOW  0
#define L3G_SA0_HIGH 1
#define L3G_SA0_AUTO 2

// register addresses

#define L3G_WHO_AM_I      0x0F

#define L3GD20_CTRL_REG1     0x20
#define L3GD20_CTRL_REG2     0x21
#define L3GD20_CTRL_REG3     0x22
#define L3GD20_CTRL_REG4     0x23
#define L3GD20_CTRL_REG5     0x24
#define L3GD20_REFERENCE     0x25
#define L3GD20_OUT_TEMP      0x26
#define L3GD20_STATUS_REG    0x27

#define L3GD20_OUT_X_L       0x28
#define L3GD20_OUT_X_H       0x29
#define L3GD20_OUT_Y_L       0x2A
#define L3GD20_OUT_Y_H       0x2B
#define L3GD20_OUT_Z_L       0x2C
#define L3GD20_OUT_Z_H       0x2D

#define L3GD20_FIFO_CTRL_REG 0x2E
#define L3GD20_FIFO_SRC_REG  0x2F

#define L3GD20_INT1_CFG      0x30
#define L3GD20_INT1_SRC      0x31
#define L3GD20_INT1_THS_XH   0x32
#define L3GD20_INT1_THS_XL   0x33
#define L3GD20_INT1_THS_YH   0x34
#define L3GD20_INT1_THS_YL   0x35
#define L3GD20_INT1_THS_ZH   0x36
#define L3GD20_INT1_THS_ZL   0x37
#define L3GD20_INT1_DURATION 0x38

class L3GD20
{
	public:
		L3GD20();

		bool init(byte device = L3G_DEVICE_AUTO, byte sa0 = L3G_SA0_AUTO);
		uint8_t setSELState(uint8_t SELState);
	public:
		void enableDefault(void);
		void readGyro(int16_t *pX, int16_t *pY, int16_t *pZ);
		void readTemperature(int8_t *pTemperature);
		void setFullScaleRange(uint8_t FullScaleRange);
		void setBandwidth(uint8_t Bandwidth);
		void setOutputDataRate(uint8_t DataRate);

	protected:
		void writeReg(uint8_t reg, uint8_t value);
		uint8_t readReg(uint8_t reg);

	private:
		byte _device; // chip type (4200D or D20)
		int address;
};

#endif



