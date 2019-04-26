//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------

#include <stdio.h>
#include "scnprintf.h"

//-----------------------------------------------------------------------------

int vscnprintf(char *buf, size_t size, const char *fmt, va_list args) {
	int i = vsnprintf(buf, size, fmt, args);
	if (i < size) {
		return i;
	}
	if (size != 0) {
		return size - 1;
	}
	return 0;
}

int scnprintf(char *buf, size_t size, const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	int i = vscnprintf(buf, size, fmt, args);
	va_end(args);
	return i;
}

//-----------------------------------------------------------------------------
