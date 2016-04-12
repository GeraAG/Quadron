#define MPU6050_ADDRESS 0xD0	//	AD0 is low, 0xD0 = write, 0xD1 = read

//	Registers
#define SAMPLE_RATE		0x19
#define CONFIG			0x1A
#define GYRO_CONFIG		0x1B
#define ACCEL_CONFIG	0x1C

#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40

#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48

//	Masks
//	Mask for CONFIG
#define DLPF_CFG_OFF	0x00	//Digital Low Pass Filter
#define DLPF_CFG_184	0x01
#define DLPF_CFG_94		0x02
#define DLPF_CFG_44		0x03
#define DLPF_CFG_21		0x04
#define DLPF_CFG_10		0x05
#define DLPF_CFG_5		0x06

//Mask for GYRO_CONFIG
#define FS_SEL_250		0x00
#define FS_SEL_500		0x08
#define FS_SEL_1000		0x10
#define FS_SEL_2000		0x18

//Mask for ACCEL_CONFIG
#define AFS_SEL_2		0x00
#define AFS_SEL_4		0x08
#define AFS_SEL_8		0x10
#define AFS_SEL_16		0x18

namespace MPU6050 {
	uint8_t setup(uint8_t DLPF, uint8_t GyroRange, uint8_t AccelRange);
	//uint8_t readAccelXYZ(void);		//uint8_t  ==  unsigned char
	//uint8_t readGyroXYZ(void);
	short readAccelX(void);			//uint16_t  ==  unsigned short
	short readAccelY(void);
	short readAccelZ(void);
	short readGyroX(void);
	short readGyroY(void);
	short readGyroZ(void);
}