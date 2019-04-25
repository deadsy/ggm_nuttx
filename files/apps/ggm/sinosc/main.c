//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int sinosc_main(int argc, char *argv[])
#endif
{
	printf("%s %d\n", __FILE__, __LINE__);
	return EXIT_SUCCESS;
}

//-----------------------------------------------------------------------------
