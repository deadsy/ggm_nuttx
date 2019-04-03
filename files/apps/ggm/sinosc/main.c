
#include <nuttx/config.h>
#include <stdio.h>

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int sinosc_main(int argc, char *argv[])
#endif
{
	printf("This is a sine wave oscillator\n");
	return 0;
}
