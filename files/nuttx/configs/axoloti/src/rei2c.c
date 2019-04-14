//-----------------------------------------------------------------------------
/*


*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/input/rei2c.h>

#include "stm32_i2c.h"
#include "axoloti.h"

//-----------------------------------------------------------------------------

#define REI2C_I2C_PORTNO 1	// On I2C1

int rei2c_initialize(char *devname) {
	struct i2c_master_s *i2c;

	iinfo("%s\n", devname);

	// Initialize I2C
	i2c = stm32_i2cbus_initialize(REI2C_I2C_PORTNO);
	if (i2c == NULL) {
		return -ENODEV;
	}
	// Register the rei2c device
	return rei2c_register(devname, i2c);
}

//-----------------------------------------------------------------------------
