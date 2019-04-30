//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

#ifndef __INCLUDE_NUTTX_AUDIO_ADAU1361_H
#define __INCLUDE_NUTTX_AUDIO_ADAU1361_H

//-----------------------------------------------------------------------------

struct adau1361_lower_s;	/* Forward reference.  Defined below */

typedef CODE int (*adau1361_handler_t) (FAR const struct adau1361_lower_s * lower, FAR void *arg);

struct adau1361_lower_s {
	// I2C characterization

	uint32_t frequency;	// Initial I2C frequency
	uint8_t address;	// 7-bit I2C address (only bits 0-6 used)

	/* IRQ/GPIO access callbacks.  These operations all hidden behind
	 * callbacks to isolate the ADAU1361 driver from differences in GPIO
	 * interrupt handling by varying boards and MCUs.  If possible,
	 * interrupts should be configured on both rising and falling edges
	 * so that contact and loss-of-contact events can be detected.
	 *
	 * attach  - Attach or detach the ADAU1361 interrupt handler to the GPIO
	 *           interrupt
	 * enable  - Enable or disable the GPIO interrupt.  Returns the
	 *           previous interrupt state.
	 * reset   - HW reset of the ADAU1361 chip
	 */

	CODE int (*attach) (FAR const struct adau1361_lower_s * lower, adau1361_handler_t isr, FAR void *arg);
	CODE bool(*enable) (FAR const struct adau1361_lower_s * lower, bool enable);
	CODE void (*reset) (FAR const struct adau1361_lower_s * lower);
};

//-----------------------------------------------------------------------------

#endif				// __INCLUDE_NUTTX_AUDIO_ADAU1361_H

//-----------------------------------------------------------------------------
