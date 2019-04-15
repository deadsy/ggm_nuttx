//-----------------------------------------------------------------------------
/*


*/
//-----------------------------------------------------------------------------

#ifndef __INCLUDE_NUTTX_INPUT_REI2C_H
#define __INCLUDE_NUTTX_INPUT_REI2C_H 1

//-----------------------------------------------------------------------------

#include <nuttx/config.h>
#include <nuttx/input/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

//-----------------------------------------------------------------------------
// registers

#define REI2C_GCONF    0x00	// General Configuration (1 byte)
#define REI2C_GP1CONF  0x01	// GP 1 Configuration (1 byte)
#define REI2C_GP2CONF  0x02	// GP 2 Configuration (1 byte)
#define REI2C_GP3CONF  0x03	// GP 3 Configuration (1 byte)
#define REI2C_INTCONF  0x04	// INT pin Configuration (1 byte)
#define REI2C_ESTATUS  0x05	// Encoder Status (1 byte)
#define REI2C_I2STATUS 0x06	// Secondary interrupt status (1 byte)
#define REI2C_FSTATUS  0x07	// Fade process status (1 byte)
#define REI2C_CVAL     0x08	// Counter Value (4 bytes)
#define REI2C_CMAX     0x0C	// Counter Max value (4 bytes)
#define REI2C_CMIN     0x10	// Counter Min value (4 bytes)
#define REI2C_ISTEP    0x14	// Increment step value (4 bytes)
#define REI2C_RLED     0x18	// LED red color intensity (1 byte)
#define REI2C_GLED     0x19	// LED green color intensity (1 byte)
#define REI2C_BLED     0x1A	// LED blue color intensity (1 byte)
#define REI2C_GP1REG   0x1B	// I/O GP1 Register (1 byte)
#define REI2C_GP2REG   0x1C	// I/O GP2 Register (1 byte)
#define REI2C_GP3REG   0x1D	// I/O GP3 Register (1 byte)
#define REI2C_ANTBOUNC 0x1E	// Anti-bouncing period (1 Byte)
#define REI2C_DPPERIOD 0x1F	// Double push period (1 Byte)
#define REI2C_FADERGB  0x20	// Fade timer RGB Encoder (1 Byte)
#define REI2C_FADEGP   0x21	// Fade timer GP ports (1 Byte)
#define REI2C_EEPROM   0x80	// EEPROM memory (128 bytes)

// REI2C_GCONF bits
#define REI2C_GCONF_DTYPE (1 << 0)	// Data type of the register: CVAL, CMAX, CMIN and ISTEP.
#define REI2C_GCONF_WRAPE (1 << 1)	// Enable counter wrap.
#define REI2C_GCONF_DIRE  (1 << 2)	// Direction of the encoder when increment.
#define REI2C_GCONF_IPUD  (1 << 3)	// Interrupt Pull-UP disable.
#define REI2C_GCONF_RMOD  (1 << 4)	// Reading Mode.
#define REI2C_GCONF_ETYPE (1 << 5)	// Set the encoder type (normal/illuminated)
#define REI2C_GCONF_MBANK (1 << 6)	// Select the EEPROM memory bank. Each bank is 128 bytes.
#define REI2C_GCONF_RESET (1 << 7)	// Reset the I2C Encoder V2

// REI2C_ESTATUS bits
#define REI2C_ESTATUS_PUSHR (1 << 0)	// push button has been released
#define REI2C_ESTATUS_PUSHP (1 << 1)	// push button has been pressed
#define REI2C_ESTATUS_PUSHD (1 << 2)	// push button has been double pushed
#define REI2C_ESTATUS_RINC  (1 << 3)	// rotated in the increase direction
#define REI2C_ESTATUS_RDEC  (1 << 4)	// rotated in the decrease direction
#define REI2C_ESTATUS_RMAX  (1 << 5)	// maximum counter value has been reached
#define REI2C_ESTATUS_RMIN  (1 << 6)	// minimum counter value has been reached
#define REI2C_ESTATUS_INT2  (1 << 7)	// Secondary interrupt status

//-----------------------------------------------------------------------------

// rei2c configuration
struct rei2c_cfg {
	uint8_t reg;
	uint32_t val;
};

int rei2c_register(const char *devname, struct i2c_master_s *i2c, const struct rei2c_cfg *cfg);

//-----------------------------------------------------------------------------

#endif				// __INCLUDE_NUTTX_INPUT_REI2C_H

//-----------------------------------------------------------------------------
