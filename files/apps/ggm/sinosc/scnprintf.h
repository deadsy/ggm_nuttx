#ifndef __INCLUDE_SCNPRINTF_H
#define __INCLUDE_SCNPRINTF_H

#include <stddef.h>
#include <stdarg.h>

int vscnprintf(char *buf, size_t size, const char *fmt, va_list args);
int scnprintf(char *buf, size_t size, const char *fmt, ...);

#endif				// __INCLUDE_SCNPRINTF_H
