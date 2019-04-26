//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>

#include <nuttx/audio/audio.h>

#include "scnprintf.h"

//-----------------------------------------------------------------------------

#define AUDIO_DEVICE "/dev/audio/pcm1"

//-----------------------------------------------------------------------------

static char *audio_caps_str(struct audio_caps_s *caps, char *str, size_t n) {
	int ofs = 0;
	ofs += scnprintf(&str[ofs], n - ofs, "ac_len %d\n", caps->ac_len);
	ofs += scnprintf(&str[ofs], n - ofs, "ac_type %d\n", caps->ac_type);
	ofs += scnprintf(&str[ofs], n - ofs, "ac_subtype %d\n", caps->ac_subtype);
	ofs += scnprintf(&str[ofs], n - ofs, "ac_channels %d\n", caps->ac_channels);
	return str;
}

#if 0

struct audio_caps_s {
	uint8_t ac_len;		/* Length of the structure */
	uint8_t ac_type;	/* Capabilities (device) type */
	uint8_t ac_subtype;	/* Capabilities sub-type, if needed */
	uint8_t ac_channels;	/* Number of channels (1, 2, 5, 7) */

	union {			/* Audio data format(s) for this device */
		uint8_t b[2];
		uint16_t hw;
	} ac_format;

	union {			/* Device specific controls. For AUDIO_DEVICE_QUERY, *//*   this field reports the device type supported */
		uint8_t b[4];	/*   by this lower-half driver. */
		uint16_t hw[2];
		uint32_t w;
	} ac_controls;
};

#endif

static int get_caps(int fd) {
	struct audio_caps_s caps;
	int rc = 0;

	memset(&caps, 0, sizeof(caps));
	caps.ac_len = sizeof(caps);
	caps.ac_type = AUDIO_TYPE_QUERY;
	caps.ac_subtype = AUDIO_TYPE_QUERY;

	rc = ioctl(fd, AUDIOIOC_GETCAPS, (unsigned long)&caps);
	if (rc != sizeof(caps)) {
		printf("ioctl() failed %d (%s %d)\n", fd, __FILE__, __LINE__);
		rc = -1;
		goto exit;
	}

	char tmp[128];
	printf("%s\n", audio_caps_str(&caps, tmp, sizeof(tmp)));

 exit:
	return rc;
}

//-----------------------------------------------------------------------------

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int sinosc_main(int argc, char *argv[])
#endif
{
	int rc = 0;

	int fd = open(AUDIO_DEVICE, O_RDWR);

	if (fd < 0) {
		printf("open() failed %d (%s %d)\n", fd, __FILE__, __LINE__);
		rc = -1;
		goto exit;
	}

	rc = ioctl(fd, AUDIOIOC_HWRESET, 0);
	if (rc < 0) {
		printf("ioctl() failed %d (%s %d)\n", rc, __FILE__, __LINE__);
		rc = -1;
		goto exit;
	}

	get_caps(fd);

	printf("closing: %s %d\n", __FILE__, __LINE__);

 exit:
	close(fd);
	return rc;
}

//-----------------------------------------------------------------------------
