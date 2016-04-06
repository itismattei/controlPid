

#ifndef I2C_TIVA_H_
#define I2C_TIVA_H_

#include <stdint.h>

class I2C{
public:
	I2C(){;}
	I2C(uint32_t);
	uint32_t BASE_ADDR;
};
#endif
