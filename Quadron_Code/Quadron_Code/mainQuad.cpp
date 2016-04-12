#include <avr/io.h>
#include <math.h>
#include <iom8.h>
#include <avr/interrupt.h>
#include "MPU-6050.h"
#include "nrf24.h"

#define F_CPU 16000000
#define GYROSCOPE_SENSITIVITY 65.536
#define M_PI 3.14159265359
#define dt 0.001							

void ComplementaryFilter(short XA, short YA, short ZA, short XG, short YG, float *pitch, float *roll, float *yaw);
void PID_Regulator(short XA, short YA, short ZA, short XG, short YG, float *pitch,float *roll, float *yaw,float *rPitch,float *rRoll,float *rYaw);
void PWM(int motor, float power);
	
int flagError;
float pitch = 0;
float roll = 0;
float yaw = 0;
short XA = 0, YA = 0, ZA = 0, XG = 0, YG = 0, ZG = 0;
short XA_1 = 0, YA_1 = 0, ZA_1 = 0, XG_1 = 0, YG_1 = 0, ZG_1 = 0;
float rPitch = 0;
float rRoll = 0;
float rYaw = 0;

uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t recieveArray[12];
	
ISR(TIMER1_COMPA_vect)//обработчик прерывания по совпадению А
{
	TCNT1H=0;//обнуляем регистр TCNT1
	TCNT1L=0;

	XA = MPU6050::readAccelX();
	YA = MPU6050::readAccelY();
	ZA = MPU6050::readAccelZ();
	XG = MPU6050::readGyroX();
	YG = MPU6050::readGyroY();
	ZG = MPU6050::readGyroZ();		
	// filter the data
	ComplementaryFilter(XA, YA, ZA, XG, YG, ZG, &pitch, &roll, &yaw);
		
		
	PID_Regulator(&pitch, &roll, &yaw, &rPitch, &rRoll, &rYaw);
}

int main()
{
	nrf24_init();
	/* Channel #2 , payload length: 12 */
	nrf24_config(2,12);
	/* Set the device addresses */
	nrf24_tx_address(tx_address);
	nrf24_rx_address(rx_address);
	
	TCCR1A=0x40;//
	TCCR1B=0x05;//CLK/1024
	OCR1AH=0x39;//write to OCR1A 14625 dt 0.001
	OCR1AL=0x21;
	TIMSK = 0x10;//разрешаем прерывание по совпадению
	sei();//разрешаем прерывания глобально

	DDRA=0x0f;//настраиваем OC1A как выход
	PORTB=0x00;
	
	
	if(MPU6050::setup()) flagError = 1;
    while (1) 
    {
		if(nrf24_dataReady())
		{
			nrf24_getData(recieveArray);
			float rPitch = (recieveArray[3] << 24) | (recieveArray[2] << 16) | (recieveArray[1] << 8) | recieveArray[0]; // maybe from 0 to 3
			float rRoll = (recieveArray[7] << 24) | (recieveArray[6] << 16) | (recieveArray[5] << 8) | recieveArray[4];
			float rYaw = (recieveArray[11] << 24) | (recieveArray[10] << 16) | (recieveArray[9] << 8) | recieveArray[8];
		}
		
		
    }
}

void ComplementaryFilter(short XA, short YA, short ZA, short XG, short YG, short ZG, float *pitch, float *roll, float *yaw)			// short == signed short int == 16 bit
{
	float pitchAcc, rollAcc, yawAcc;
	
	// Integrate the gyroscope data -> int(angularSpeed) = angle
	*pitch += ((float)XG / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
	*roll += ((float)YG / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
	*yaw+= ((float)ZG / GYROSCOPE_SENSITIVITY) * dt; // Angle around the Z-axis

	
	// Compensate for drift with accelerometer data if not "noise"
	// Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
	int forceMagnitudeApprox = abs(XA) + abs(YA) + abs(ZA);
	if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
	{
		// Turning around the X axis results in a vector on the Y-axis
		pitchAcc = atan2f((float)YA, (float)ZA) * 180 / M_PI;
		*pitch = *pitch * 0.98 + pitchAcc * 0.02;
		
		// Turning around the Y axis results in a vector on the X-axis
		rollAcc = atan2f((float)XA, (float)ZA) * 180 / M_PI;
		*roll = *roll * 0.98 + rollAcc * 0.02;

		yawAcc = atan2f((float)ZA, (float)XA) * 180 / M_PI;
		*yaw = *yaw * 0.98 + yawAcc * 0.02;

	}
}

void PID_Regulator(float *pitch,float *roll, float *yaw,float *rPitch,float *rRoll,float *rYaw)
{
	XA_1 = MPU6050::readAccelX();
	YA_1 = MPU6050::readAccelY();
	ZA_1 = MPU6050::readAccelZ();
	XG_1 = MPU6050::readGyroX();
	YG_1 = MPU6050::readGyroY();
	ZG_1 = MPU6050::readGyroZ();
	
	*pitch = *pitch + *rPitch + ((float)XA_1-(float)XA)/dt; 
	*pitch += ((float)XG / GYROSCOPE_SENSITIVITY) * dt;
	
	*roll = *roll + *rRoll + ((float)YA_1-(float)YA)/dt; 
	*roll += ((float)YG / GYROSCOPE_SENSITIVITY) * dt;
	
	*yaw = *yaw + *rYaw + ((float)ZA_1-(float)ZA)/dt; 
	*yaw += ((float)YG / GYROSCOPE_SENSITIVITY) * dt;
	
}

void PWM(int motor, float power)
{
	motor = power(2,motor);
	PORTA = motor;
	delay_us(100-power/10);
	PORTA = 0;
	
}

void YAW(float yaw)
{
	if(yaw>0){
		PWM(0,yaw);
		PWM(1,100);
		PWM(2,yaw);
		PWM(3,100);
	}
	if(yaw<0){
		PWM(1,-yaw);
		PWM(0,100);
		PWM(3,-yaw);
		PWM(2,100);
	}
}

void PITCH(float pitch)
{
	if(pitch>0){
		PWM(0,pitch);
		PWM(2,100);
		PWM(1,pitch);
		PWM(3,100);
	}
	if(pitch<0){
		PWM(2,-pitch);
		PWM(0,100);
		PWM(3,-pitch);
		PWM(1,100);
	}
}

void ROLL(float roll)
{
	if(roll>0){
		PWM(0,roll);
		PWM(1,100);
		PWM(3,roll);
		PWM(2,100);
	}
	if(roll<0){
		PWM(1,-roll);
		PWM(0,100);
		PWM(2,-roll);
		PWM(3,100);
	}
}

