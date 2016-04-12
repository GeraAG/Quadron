#include <TWI.h>
namespace TWI	{
	void TWIInit(void)
	{
		//set SCL to 400kHz if cpu freq = 16 MHz
		TWSR = 0x00;
		TWBR = 0x0C;
		//enable TWI
		TWCR = (1<<TWEN);
	}

	void TWIStart(void)
	{
		TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		while ((TWCR & (1<<TWINT)) == 0);
	}

	//send stop signal
	void TWIStop(void)
	{
		TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	}

	void TWIWrite(uint8_t u8data)
	{
		TWDR = u8data;
		TWCR = (1<<TWINT)|(1<<TWEN);
		while ((TWCR & (1<<TWINT)) == 0);
	}

	uint8_t TWIReadACK(void)
	{
		TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
		while ((TWCR & (1<<TWINT)) == 0);
		return TWDR;
	}
	
	uint8_t TWIReadNACK(void)
	{
		TWCR = (1<<TWINT)|(1<<TWEN);
		while ((TWCR & (1<<TWINT)) == 0);
		return TWDR;
	}

	uint8_t TWIGetStatus(void)
	{
		uint8_t status;
		//mask status
		status = TWSR & 0xF8;
		return status;
	}
	
	uint8_t WriteByte(uint8_t u8device_addr, uint8_t u8register_addr, uint8_t u8data)
	{
		TWIStart();
		if (TWIGetStatus() != 0x08)
			return ERROR;
		//select device
		TWIWrite(u8device_addr);
		if (TWIGetStatus() != 0x18)
			return ERROR;
		//send register address
		TWIWrite(u8register_addr);
		if (TWIGetStatus() != 0x28)
			return ERROR;
		//write byte to device
		TWIWrite(u8data);
		if (TWIGetStatus() != 0x28)
			return ERROR;
		TWIStop();
		return SUCCESS;
	}
	
	uint8_t ReadByte(uint8_t u8device_addr, uint8_t u8register_addr, uint8_t *u8data)
	{
		//uint8_t databyte;
		TWIStart();
		if (TWIGetStatus() != 0x08)
			return ERROR;
		//select device
		TWIWrite(u8device_addr);
		if (TWIGetStatus() != 0x18)
			return ERROR;
		//send register address
		TWIWrite(u8register_addr);
		if (TWIGetStatus() != 0x28)
			return ERROR;
		//send start
		TWIStart();
		if (TWIGetStatus() != 0x10)
			return ERROR;
		//select devise and send read bit
		TWIWrite(u8device_addr|1);
		if (TWIGetStatus() != 0x40)
			return ERROR;
		*u8data = TWIReadNACK();
		if (TWIGetStatus() != 0x58)
			return ERROR;
		TWIStop();
		return SUCCESS;
	}
	
	uint8_t ReadByte(uint8_t u8device_addr, uint8_t u8register_addr, uint8_t *u8data, int length){
		//uint8_t databyte;
		TWIStart();
		if (TWIGetStatus() != 0x08) return ERROR;
		//select device
		TWIWrite();
		if (TWIGetStatus() != 0x18) return ERROR;
		//send register address
		TWIWrite(u8register_addr);
		if (TWIGetStatus() != 0x28) return ERROR;
		//send start
		TWIStart();
		if (TWIGetStatus() != 0x10) return ERROR;
		//select devise and send read bit
		TWIWrite(u8device_addr|1);
		if (TWIGetStatus() != 0x40) return ERROR;
		for(int n=0; n<length; n++){
			*(u8data+n) = TWIReadNACK();
			if (TWIGetStatus() != 0x58) return ERROR;
		}
		TWIStop();
		return SUCCESS;
	}
}
