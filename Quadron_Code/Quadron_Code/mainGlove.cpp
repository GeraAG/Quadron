#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#include "MPU-6050.h"
#include "nrf24.h"

#define F_CPU 16000000
#define GYROSCOPE_SENSITIVITY 65.536
#define M_PI 3.14159265359
#define dt 0.001							// 10 ms sample rate!

void ComplementaryFilter(short XA, short YA, short ZA, short XG, short YG, short ZG, float *pitch, float *roll, float *yaw);

float pitch = 0;
float roll = 0;
float yaw = 0;
short XA = 0, YA = 0, ZA = 0, XG = 0, YG = 0, ZG = 0;
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t sendArray[12];

int main(void)
{
	int flagError;
	
	if(MPU6050::setup()) flagError = 1;
	
	nrf24_init();
	/* Channel #2 , payload length: 12 */
	nrf24_config(2,12);
	/* Set the device addresses */
	nrf24_tx_address(tx_address);
	nrf24_rx_address(rx_address);
	
	TCCR1A=0x40;
	TCCR1B=0x05;		//CLK/1024
	OCR1AH=0x39;		//write to OCR1A 14625 dt 0.001
	OCR1AL=0x21;
	TIMSK = 0x10;		//разрешаем прерывание по совпадению
	sei();				//разрешаем прерывания глобально
    while (1) 
    {
		
    }
}

ISR(TIMER1_COMPA_vect)	//обработчик прерывания по совпадению А
{
	TCNT1H=0;			//обнуляем регистр TCNT1
	TCNT1L=0;

	XA = MPU6050::readAccelX();
	YA = MPU6050::readAccelY();
	ZA = MPU6050::readAccelZ();
	XG = MPU6050::readGyroX();
	YG = MPU6050::readGyroY();
	ZG = MPU6050::readGyroZ();
	// filter the data
	ComplementaryFilter(XA, YA, ZA, XG, YG, ZG, &pitch, &roll, &yaw);
	
	//Can send only bytes so need to transform float(32 bit) to uint8_t(8 bit)
	sendArray = reinterpret_cast<uint8_t*>(&pitch);
	sendArray[4] = reinterpret_cast<uint8_t*>(&roll);
	sendArray[8] = reinterpret_cast<uint8_t*>(&yaw);
	
	nrf24_send(sendArray); 
	
}

void ComplementaryFilter(short XA, short YA, short ZA, short XG, short YG, short ZG, float *pitch, float *roll, float *yaw)			// short == signed short int == 16 bit
{
	float pitchAcc, rollAcc;
	
	// Integrate the gyroscope data -> int(angularSpeed) = angle
	*pitch += ((float)XG / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
	*roll += ((float)YG / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
	*yaw += ((float)ZG / GYROSCOPE_SENSITIVITY) * dt; // Angle around the Z-axis
	
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

