#define ERROR	0x01
#define SUCCESS	0x00

namespace TWI	{
	void TWIInit(void);
	void TWIStart(void);
	void TWIStop(void);
	void TWIWrite(uint8_t u8data);
	uint8_t TWIReadACK(void);
	uint8_t TWIReadNACK(void);
	uint8_t WriteByte(uint8_t u8device_addr, uint8_t u8register_addr, uint8_t u8data);
	uint8_t ReadByte(uint8_t u8device_addr, uint8_t u8register_addr, uint8_t *u8data);
	uint8_t ReadByte(uint8_t u8device_addr, uint8_t u8register_addr, uint8_t *u8data, int length);
};
