/* MicroArray Fingerprint
 * print.h
 * date: 2015-08-15
 * version: v2.0
 * Author: czl
 */

#ifndef XPRINT_H
#define XPRINT_H

static u8 sdeb = 1; // 调试：1打印信息，0不显示

#define DEBUG

void printd(char *fmt, ...) {
#ifdef DEBUG
	if (sdeb == 1) {
		va_list args;
		va_start(args, fmt);
		vprintk(fmt, args);
		va_end(args);
	}
#endif
}

void printw(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vprintk(fmt, args);
	va_end(args);
}

void printn(const char *func, u8 *buf, int len) {
#ifdef DEBUG
	int i;
	printd("%s: buf:\n", func);
	for (i = 0; i < len; i++) {
		printd("%.2x", (int)buf[i]);
		if (i > 0 && (i + 1) % 30 == 0) printd("\n");
	}
	printd("\n");
#endif
}

#endif
