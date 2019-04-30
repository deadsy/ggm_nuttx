//-----------------------------------------------------------------------------
/*

Analog Devices ADAU1361 Stereo CODEC

*/
//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <stdint.h>
#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/audio/audio.h>

//-----------------------------------------------------------------------------

#define ADAU1361_REG_BASE 0x4000

#define ADAU1361_REG_Clock_Ctl              0x00
#define ADAU1361_REG_PLL_Ctl                0x02
#define ADAU1361_REG_Mic_Jack_Detect        0x08
#define ADAU1361_REG_Rec_Power_Mgmt         0x09
#define ADAU1361_REG_Rec_Mixer_Left0        0x0A
#define ADAU1361_REG_Rec_Mixer_Left1        0x0B
#define ADAU1361_REG_Rec_Mixer_Right0       0x0C
#define ADAU1361_REG_Rec_Mixer_Right1       0x0D
#define ADAU1361_REG_Left_Diff_Input_Vol    0x0E
#define ADAU1361_REG_Right_Diff_Input_Vol   0x0F
#define ADAU1361_REG_Record_Mic_Bias        0x10
#define ADAU1361_REG_ALC0                   0x11
#define ADAU1361_REG_ALC1                   0x12
#define ADAU1361_REG_ALC2                   0x13
#define ADAU1361_REG_ALC3                   0x14
#define ADAU1361_REG_Serial_Port0           0x15
#define ADAU1361_REG_Serial_Port1           0x16
#define ADAU1361_REG_Converter0             0x17
#define ADAU1361_REG_Converter1             0x18
#define ADAU1361_REG_ADC_Ctl                0x19
#define ADAU1361_REG_Left_Digital_Vol       0x1A
#define ADAU1361_REG_Right_Digital_Vol      0x1B
#define ADAU1361_REG_Play_Mixer_Left0       0x1C
#define ADAU1361_REG_Play_Mixer_Left1       0x1D
#define ADAU1361_REG_Play_Mixer_Right0      0x1E
#define ADAU1361_REG_Play_Mixer_Right1      0x1F
#define ADAU1361_REG_Play_LR_Mixer_Left     0x20
#define ADAU1361_REG_Play_LR_Mixer_Right    0x21
#define ADAU1361_REG_Play_LR_Mixer_Mono     0x22
#define ADAU1361_REG_Play_HP_Left_Vol       0x23
#define ADAU1361_REG_Play_HP_Right_Vol      0x24
#define ADAU1361_REG_Line_Output_Left_Vol   0x25
#define ADAU1361_REG_Line_Output_Right_Vol  0x26
#define ADAU1361_REG_Play_Mono_Output       0x27
#define ADAU1361_REG_Pop_Click_Suppress     0x28
#define ADAU1361_REG_Play_Power_Mgmt        0x29
#define ADAU1361_REG_DAC_Ctl0               0x2A
#define ADAU1361_REG_DAC_Ctl1               0x2B
#define ADAU1361_REG_DAC_Ctl2               0x2C
#define ADAU1361_REG_Serial_Port_Pad        0x2D
#define ADAU1361_REG_Ctl_Port_Pad0          0x2F
#define ADAU1361_REG_Ctl_Port_Pad1          0x30
#define ADAU1361_REG_Jack_Detect_Pin        0x31
#define ADAU1361_REG_Dejitter_Ctl           0x36

//-----------------------------------------------------------------------------

struct adau1361_dev_s {
	struct audio_lowerhalf_s dev;	// ADAU1361 audio lower half (this device)

	// Our specific driver data goes here
	const struct adau1361_lower_s *lower;	// Pointer to the board lower functions
	struct i2c_master_s *i2c;	// I2C driver to use
	struct i2s_dev_s *i2s;	// I2S driver to use
	struct dq_queue_s pendq;	// Queue of pending buffers to be sent
	struct dq_queue_s doneq;	// Queue of sent buffers to be returned
	sem_t pendsem;		// Protect pendq

};

//-----------------------------------------------------------------------------

static uint8_t adau1361_readreg(struct adau1361_dev_s *priv, uint8_t addr) {
	return 0;
}

//-----------------------------------------------------------------------------

struct adau1361_regdump_s {
	const char *name;
	uint8_t addr;
};

static const struct adau1361_regdump_s g_adau1361_debug[] = {
	{"Clock_Ctl", ADAU1361_REG_Clock_Ctl},
	{"PLL_Ctl", ADAU1361_REG_PLL_Ctl},
	{"Mic_Jack_Detect", ADAU1361_REG_Mic_Jack_Detect},
	{"Rec_Power_Mgmt", ADAU1361_REG_Rec_Power_Mgmt},
	{"Rec_Mixer_Left0", ADAU1361_REG_Rec_Mixer_Left0},
	{"Rec_Mixer_Left1", ADAU1361_REG_Rec_Mixer_Left1},
	{"Rec_Mixer_Right0", ADAU1361_REG_Rec_Mixer_Right0},
	{"Rec_Mixer_Right1", ADAU1361_REG_Rec_Mixer_Right1},
	{"Left_Diff_Input_Vol", ADAU1361_REG_Left_Diff_Input_Vol},
	{"Right_Diff_Input_Vol", ADAU1361_REG_Right_Diff_Input_Vol},
	{"Record_Mic_Bias", ADAU1361_REG_Record_Mic_Bias},
	{"ALC0", ADAU1361_REG_ALC0},
	{"ALC1", ADAU1361_REG_ALC1},
	{"ALC2", ADAU1361_REG_ALC2},
	{"ALC3", ADAU1361_REG_ALC3},
	{"Serial_Port0", ADAU1361_REG_Serial_Port0},
	{"Serial_Port1", ADAU1361_REG_Serial_Port1},
	{"Converter0", ADAU1361_REG_Converter0},
	{"Converter1", ADAU1361_REG_Converter1},
	{"ADC_Ctl", ADAU1361_REG_ADC_Ctl},
	{"Left_Digital_Vol", ADAU1361_REG_Left_Digital_Vol},
	{"Right_Digital_Vol", ADAU1361_REG_Right_Digital_Vol},
	{"Play_Mixer_Left0", ADAU1361_REG_Play_Mixer_Left0},
	{"Play_Mixer_Left1", ADAU1361_REG_Play_Mixer_Left1},
	{"Play_Mixer_Right0", ADAU1361_REG_Play_Mixer_Right0},
	{"Play_Mixer_Right1", ADAU1361_REG_Play_Mixer_Right1},
	{"Play_LR_Mixer_Left", ADAU1361_REG_Play_LR_Mixer_Left},
	{"Play_LR_Mixer_Right", ADAU1361_REG_Play_LR_Mixer_Right},
	{"Play_LR_Mixer_Mono", ADAU1361_REG_Play_LR_Mixer_Mono},
	{"Play_HP_Left_Vol", ADAU1361_REG_Play_HP_Left_Vol},
	{"Play_HP_Right_Vol", ADAU1361_REG_Play_HP_Right_Vol},
	{"Line_Output_Left_Vol", ADAU1361_REG_Line_Output_Left_Vol},
	{"Line_Output_Right_Vol", ADAU1361_REG_Line_Output_Right_Vol},
	{"Play_Mono_Output", ADAU1361_REG_Play_Mono_Output},
	{"Pop_Click_Suppress", ADAU1361_REG_Pop_Click_Suppress},
	{"Play_Power_Mgmt", ADAU1361_REG_Play_Power_Mgmt},
	{"DAC_Ctl0", ADAU1361_REG_DAC_Ctl0},
	{"DAC_Ctl1", ADAU1361_REG_DAC_Ctl1},
	{"DAC_Ctl2", ADAU1361_REG_DAC_Ctl2},
	{"Serial_Port_Pad", ADAU1361_REG_Serial_Port_Pad},
	{"Ctl_Port_Pad0", ADAU1361_REG_Ctl_Port_Pad0},
	{"Ctl_Port_Pad1", ADAU1361_REG_Ctl_Port_Pad1},
	{"Jack_Detect_Pin", ADAU1361_REG_Jack_Detect_Pin},
	{"Dejitter_Ctl", ADAU1361_REG_Dejitter_Ctl},
};

#define ADAU1361_NREGISTERS (sizeof(g_adau1361_debug)/sizeof(struct adau1361_regdump_s))

void adau1361_dump_registers(struct audio_lowerhalf_s *dev, const char *msg) {
	syslog(LOG_INFO, "ADAU1361 Registers: %s\n", msg);
	for (int i = 0; i < ADAU1361_NREGISTERS; i++) {
		const char *name = g_adau1361_debug[i].name;
		uint8_t addr = g_adau1361_debug[i].addr;
		uint8_t val = adau1361_readreg((struct adau1361_dev_s *)dev, addr);
		syslog(LOG_INFO, "%16s[%04x]: %02x\n", name, addr, val);
	}
}

//-----------------------------------------------------------------------------

// Get the lower-half device capabilities
static int adau1361_getcaps(struct audio_lowerhalf_s *dev, int type, struct audio_caps_s *pCaps) {
	return 0;
}

// Shutdown the driver (called after close)
static int adau1361_shutdown(struct audio_lowerhalf_s *dev) {
	return 0;
}

// Allocate an audio pipeline buffer
static int adau1361_allocbuffer(struct audio_lowerhalf_s *dev, struct audio_buf_desc_s *apb) {
	return 0;
}

// Free an audio pipeline buffer
static int adau1361_freebuffer(struct audio_lowerhalf_s *dev, struct audio_buf_desc_s *apb) {
	return 0;
}

// Enqueue a buffer for processing
static int adau1361_enqueuebuffer(struct audio_lowerhalf_s *dev, struct ap_buffer_s *apb) {
	return 0;
}

// Cancel a previously enqueued buffer
static int adau1361_cancelbuffer(struct audio_lowerhalf_s *dev, struct ap_buffer_s *apb) {
	return 0;
}

// Driver specific ioctl commands
static int adau1361_ioctl(struct audio_lowerhalf_s *dev, int cmd, unsigned long arg) {
	return 0;
}

// Driver specific read commands
static int adau1361_read(struct audio_lowerhalf_s *dev, char *buffer, size_t buflen) {
	return 0;
}

// Driver specific write commands
static int adau1361_write(struct audio_lowerhalf_s *dev, const char *buffer, size_t buflen) {
	return 0;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION

// Release a session
static int adau1361_release(struct audio_lowerhalf_s *dev, void *session) {
	return 0;
}

// Reserve a session
static int adau1361_reserve(struct audio_lowerhalf_s *dev, void **psession) {
	return 0;
}

// Resumes audio streaming after a pause
static int adau1361_resume(struct audio_lowerhalf_s *dev, void *session) {
	return 0;
}

// Pause the audio stream
static int adau1361_pause(struct audio_lowerhalf_s *dev, void *session) {
	return 0;
}

// Start audio streaming
static int adau1361_start(struct audio_lowerhalf_s *dev, void *session) {
	return 0;
}

// Stop audio streaming
static int adau1361_stop(struct audio_lowerhalf_s *dev, void *session) {
	return 0;
}

// Configure the driver
static int adau1361_configure(struct audio_lowerhalf_s *dev, void *session, const struct audio_caps_s *pCaps) {
	return 0;
}

#else

// Release a session
static int adau1361_release(struct audio_lowerhalf_s *dev) {
	return 0;
}

// Reserve a session
static int adau1361_reserve(struct audio_lowerhalf_s *dev) {
	return 0;
}

// Resumes audio streaming after a pause
static int adau1361_resume(struct audio_lowerhalf_s *dev) {
	return 0;
}

// Pause the audio stream
static int adau1361_pause(struct audio_lowerhalf_s *dev) {
	return 0;
}

// Start audio streaming
static int adau1361_start(struct audio_lowerhalf_s *dev) {
	return 0;
}

// Stop audio streaming
static int adau1361_stop(struct audio_lowerhalf_s *dev) {
	return 0;
}

// Configure the driver
static int adau1361_configure(struct audio_lowerhalf_s *dev, const struct audio_caps_s *pCaps) {
	return 0;
}

#endif

//-----------------------------------------------------------------------------

static const struct audio_ops_s g_audioops = {
	.getcaps = adau1361_getcaps,
	.configure = adau1361_configure,
	.shutdown = adau1361_shutdown,
	.start = adau1361_start,
	.stop = adau1361_stop,
	.pause = adau1361_pause,
	.resume = adau1361_resume,
	.allocbuffer = adau1361_allocbuffer,
	.freebuffer = adau1361_freebuffer,
	.enqueuebuffer = adau1361_enqueuebuffer,
	.cancelbuffer = adau1361_cancelbuffer,
	.ioctl = adau1361_ioctl,
	.read = adau1361_read,
	.write = adau1361_write,
	.reserve = adau1361_reserve,
	.release = adau1361_release,
};

//-----------------------------------------------------------------------------

struct audio_lowerhalf_s *adau1361_initialize(struct i2c_master_s *i2c, struct i2s_dev_s *i2s, const struct adau1361_lower_s *lower) {

	// Sanity check
	DEBUGASSERT(i2c && i2s && lower);

	// Allocate a ADAU1361 device structure
	struct adau1361_dev_s *priv = (struct adau1361_dev_s *)kmm_zalloc(sizeof(struct adau1361_dev_s));
	if (priv) {
		// Initialize the ADAU1361 device structure.

		priv->dev.ops = &g_audioops;
		priv->lower = lower;
		priv->i2c = i2c;
		priv->i2s = i2s;

		nxsem_init(&priv->pendsem, 0, 1);
		dq_init(&priv->pendq);
		dq_init(&priv->doneq);

		// Initialize I2C
		audinfo("address=%02x frequency=%d\n", lower->address, lower->frequency);

		// Software reset. Put all ADAU1361 registers to their default state.
		ADAU1361_HW_RESET(priv->lower);
		adau1361_dump_registers(&priv->dev, "After reset");

		// Verify the ADAU1361 is present and available on this I2C
		uint8_t val = adau1361_readreg(priv, ADAU1361_REG_Serial_Port_Pad);
		if (val != 0xaa) {
			auderr("adau1361 not found\n");
			goto errout_with_dev;
		}
		// Reset and reconfigure the ADAU1361 hardware
		adau1361_reset(priv);
		return &priv->dev;
	}

	return NULL;

 errout_with_dev:
	nxsem_destroy(&priv->pendsem);
	kmm_free(priv);
	return NULL;
}

//-----------------------------------------------------------------------------
