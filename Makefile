KMOD=	QSGoodix
SRCS=	goodix_acpi.c
SRCS+=	acpi_if.h bus_if.h device_if.h iicbus_if.h opt_acpi.h opt_evdev.h opt_kbd.h
SRCS+=	gpio_if.h gpiobus_if.h opt_platform.h

SYSDIR?=	/usr/src/sys
.PATH:	${SYSDIR}/dev/gpio
CFLAGS+=	-I${SYSDIR}/dev/gpio

.include <bsd.kmod.mk>
