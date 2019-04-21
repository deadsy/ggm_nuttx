//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <nuttx/input/rei2c.h>

//-----------------------------------------------------------------------------

#define CONFIG_REI2C_DEVNAME "/dev/re0"

//-----------------------------------------------------------------------------

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int sinosc_main(int argc, char *argv[])
#endif
{

	int errcode = EXIT_SUCCESS;

	int fd = open(CONFIG_REI2C_DEVNAME, O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "failed to open %s (%d)\n", CONFIG_REI2C_DEVNAME, errno);
		return EXIT_FAILURE;
	}

	while (1) {
		struct rei2c_status status;
		int ret = read(fd, &status, sizeof(status));
		if (ret < 0) {
			fprintf(stderr, "read() failed %d\n", ret);
			goto exit;
		}
		//printf("enc 0x%02x\n", status.enc);
		//printf("int2 0x%02x\n", status.int2);
		//printf("fade 0x%02x\n", status.fade);
		printf("cnt %d\n", status.cnt);

		usleep(100000);
	}

 exit:
	close(fd);
	return errcode;
}

//-----------------------------------------------------------------------------
