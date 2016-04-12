#include <TWI.h>
#include <MPU-6050.h>

// in main program before all this functions run TWI::TWIInit();
namespace MPU6050{
	uint8_t setup(uint8_t DLPF, uint8_t GyroRange, uint8_t AccelRange){	//MPU6050::setup(DLPF_CFG_OFF, FS_SEL_250, AFS_SEL_2)
		TWI::TWIInit();
		if(TWI::WriteByte(MPU6050_ADDRESS, CONFIG, DLPF)) return 1;		//WriteByte returns 0 on success, so if statement is false it skips "return 1"
		if(TWI::WriteByte(MPU6050_ADDRESS, GYRO_CONFIG, GyroRange)) return 1;
		if(TWI::WriteByte(MPU6050_ADDRESS, ACCEL_CONFIG, AccelRange)) return 1;
		return 0;		
	}
	
	//uint8_t readAccelXYZ(){}
	
	short readAccelX(void){
		uint8_t XH;
		uint8_t XL;
		if(TWI::ReadByte(MPU6050_ADDRESS, ACCEL_XOUT_H, &XH)) return 1;
		if(TWI::ReadByte(MPU6050_ADDRESS, ACCEL_XOUT_L, &XL)) return 1;
		short X = ((short)XH << 8) | XL;
		return X;
	}
	/*
	short readSensor(uint8_t highAddress, uint8_t lowAddress){
		uint8_t H;
		uint8_t L;
		if(TWI::ReadByte(MPU6050_ADDRESS, highAddress, &H)) return 1;
		if(TWI::ReadByte(MPU6050_ADDRESS, lowAddress, &L)) return 1;
		short A = ((short)H << 8) | L;
		return A;
	}
	*/
	short readAccelY(void){
		uint8_t YH;
		uint8_t YL;
		if(TWI::ReadByte(MPU6050_ADDRESS, ACCEL_YOUT_H, &YH)) return 1;
		if(TWI::ReadByte(MPU6050_ADDRESS, ACCEL_YOUT_L, &YL)) return 1;
		short Y = ((short)YH << 8) | YL;
		return Y;
	}
	short readAccelZ(void){
		uint8_t ZH;
		uint8_t ZL;
		if(TWI::ReadByte(MPU6050_ADDRESS, ACCEL_ZOUT_H, &ZH)) return 1;
		if(TWI::ReadByte(MPU6050_ADDRESS, ACCEL_ZOUT_L, &ZL)) return 1;
		short Z = ((short)ZH << 8) | ZL;
		return Z;
	}
	short readGyroX(void){
		uint8_t XH;
		uint8_t XL;
		if(TWI::ReadByte(MPU6050_ADDRESS, GYRO_XOUT_H, &XH)) return 1;
		if(TWI::ReadByte(MPU6050_ADDRESS, GYRO_XOUT_L, &XL)) return 1;
		short X = ((short)XH << 8) | XL;
		return X;
	}
	short readGyroY(void){
		uint8_t YH;
		uint8_t YL;
		if(TWI::ReadByte(MPU6050_ADDRESS, GYRO_YOUT_H, &YH)) return 1;
		if(TWI::ReadByte(MPU6050_ADDRESS, GYRO_YOUT_L, &YL)) return 1;
		short Y = ((short)YH << 8) | YL;
		return Y;
	}
	short readGyroZ(void){
		uint8_t ZH;
		uint8_t ZL;
		if(TWI::ReadByte(MPU6050_ADDRESS, GYRO_ZOUT_H, &ZH)) return 1;
		if(TWI::ReadByte(MPU6050_ADDRESS, GYRO_ZOUT_L, &ZL)) return 1;
		short Z = ((short)ZH << 8) | ZL;
		return Z;
	}
}

