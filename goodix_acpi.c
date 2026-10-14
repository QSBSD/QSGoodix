/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 <your name>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Goodix GT9110 I2C touchscreen (ACPI id GDIX1002, bus address 0x14,
 * sensor grid 3840x2190, 10 contacts).  Touches are exported through evdev.
 *
 * The controller interrupt is used when the bus provides it.  If it does
 * not, or if it turns out never to fire, the driver polls the status
 * register instead, slowly while idle and quickly while touched.
 *
 * The GpioInt (INT) and GpioIo (RST) lines from _CRS are looked up through
 * gpiobus.  FreeBSD cannot hand a GpioInt interrupt to a consumer on amd64,
 * so the lines are used to reset the controller (RST low/high, INT selects
 * the bus address) when it stops answering on the bus; data is still polled.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/time.h>
#include <sys/gpio.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/gpio/gpiobusvar.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include <dev/evdev/evdev.h>
#include <dev/evdev/input.h>

/* Controller registers. */
#define	GT_REG_PRODUCT		0x8140	/* 4 ascii digits + 2 bytes fw version */
#define	GT_REG_CONFIG		0x8047	/* config block */
#define	GT_REG_STATE		0x814E	/* bit 7: report valid, bits 3:0: count */
#define	GT_REG_POINTS		0x814F	/* array of point records */
#define	GT_REG_COMMAND		0x8040
#define	GT_CMD_SLEEP		0x05	/* screen off, wake with an INT pulse */

#define	GT_STATE_VALID		0x80
#define	GT_STATE_COUNT		0x0f
#define	GT_REC_LEN		8
#define	GT_MAX_SLOTS		10
#define	GT_DEFAULT_X		3840	/* used if the config block is empty */
#define	GT_DEFAULT_Y		2190
#define	GT_PRODUCT		9110
#define	GT_MAX_WIDTH		255

#define	GT_ID_RETRIES		3
#define	GT_ERR_LIMIT		5	/* consecutive errors before recovery */
#define	GT_IRQ_MISS_LIMIT	3	/* touches seen only by the watchdog */
#define	GT_WATCHDOG_HZ		2	/* poll rate while the interrupt is used */
#define	GT_DEF_ACTIVE_HZ	100
#define	GT_DEF_IDLE_HZ		20

#define	GT_DBG_INIT		0x01	/* init, reset, sleep, wake */
#define	GT_DBG_IO		0x02	/* bus errors */
#define	GT_DBG_TOUCH		0x04	/* every reported point */

struct gt_softc {
	device_t		dev;
	struct evdev_dev	*evdev;
	struct taskqueue	*tq;
	struct task		irq_task;
	struct timeout_task	poll_task;
	struct resource		*irq_res;
	void			*irq_cookie;
	int			irq_rid;
	u_int			irq_flag;

	gpio_pin_t		gpio_int;	/* GpioInt line, may be NULL */
	gpio_pin_t		gpio_rst;	/* GpioIo reset line, may be NULL */

	uint8_t			addr;		/* address as iicbus wants it */
	uint16_t		product;
	uint16_t		fw;
	uint16_t		x_max;		/* sensor grid */
	uint16_t		y_max;
	u_int			nslots;

	int32_t			track[GT_MAX_SLOTS];	/* -1 when slot is free */
	int32_t			next_track;

	int			active_until;	/* ticks */
	int			errors;
	u_int			irq_miss;
	u_int			last_n;		/* points in the previous report */
	int			recover_fail;	/* consecutive failed recoveries */
	bool			err_reported;
	bool			stopping;
	bool			suspended;
	bool			opened;		/* evdev has readers */

	/* sysctl */
	int			invert_x;
	int			invert_y;
	int			swap_xy;	/* exchange raw X and Y */
	int			x_override;	/* panel size override, 0 = from chip */
	int			y_override;
	int			debug;
	int			suspend_sleep;	/* put the chip to sleep in suspend */
	uint16_t		abs_x;		/* reported axis ranges */
	uint16_t		abs_y;
	bool			asleep;
	int			poll_active_hz;
	int			poll_idle_hz;
	int			using_irq;
	int			gpio_pins;	/* bit 0: INT, bit 1: RST */
	u_int			stat_irq;
	u_int			stat_hits;
	u_int			stat_recover;

	/* error counters */
	u_int			cnt_read;
	u_int			cnt_write;
	u_int			cnt_ident;
	u_int			cnt_count;
	u_int			cnt_slot;
	u_int			cnt_sleep;
	u_int			cnt_wake;
	int			last_err;
};

#define	GT_DBG(sc, lvl, ...) do {					\
	if ((sc)->debug & (lvl))					\
		device_printf((sc)->dev, __VA_ARGS__);			\
} while (0)

static char *gt_acpi_ids[] = { "GDIX1002", NULL };

static int
gt_read(struct gt_softc *sc, uint16_t reg, uint8_t *buf, int len)
{
	uint8_t reg_be[2] = { reg >> 8, reg & 0xff };
	int err;
	struct iic_msg m[2] = {
		{ .slave = sc->addr, .flags = IIC_M_WR,
		  .len = sizeof(reg_be), .buf = reg_be },
		{ .slave = sc->addr, .flags = IIC_M_RD,
		  .len = len, .buf = buf },
	};

	err = iicbus_transfer_excl(sc->dev, m, nitems(m), IIC_INTRWAIT);
	if (err != 0) {
		sc->cnt_read++;
		sc->last_err = err;
		GT_DBG(sc, GT_DBG_IO, "read 0x%04x len %d: error %d\n", reg,
		    len, err);
	}
	return (err);
}

static int
gt_write8(struct gt_softc *sc, uint16_t reg, uint8_t val)
{
	uint8_t out[3] = { reg >> 8, reg & 0xff, val };
	struct iic_msg m = {
		.slave = sc->addr, .flags = IIC_M_WR,
		.len = sizeof(out), .buf = out,
	};
	int err;

	err = iicbus_transfer_excl(sc->dev, &m, 1, IIC_INTRWAIT);
	if (err != 0) {
		sc->cnt_write++;
		sc->last_err = err;
		GT_DBG(sc, GT_DBG_IO, "write 0x%04x: error %d\n", reg, err);
	}
	return (err);
}

static void
gt_acpi_power_on(struct gt_softc *sc)
{
	ACPI_HANDLE h = acpi_get_handle(sc->dev);
	ACPI_STATUS st;

	if (h == NULL)
		return;
	st = AcpiEvaluateObject(h, "_PS0", NULL, NULL);
	if (ACPI_SUCCESS(st))
		GT_DBG(sc, GT_DBG_INIT, "_PS0 evaluated\n");
	else if (st != AE_NOT_FOUND)
		device_printf(sc->dev, "_PS0: %s\n", AcpiFormatException(st));
}

/*
 * GPIO lines.  The first GpioInt resource of _CRS is INT, the first GpioIo
 * resource is RST.  Levels are physical: GpioIo carries no polarity.
 */
struct gt_gpio_crs {
	ACPI_HANDLE	int_ctrl;
	ACPI_HANDLE	rst_ctrl;
	uint32_t	int_pin;
	uint32_t	rst_pin;
};

static ACPI_STATUS
gt_gpio_crs_cb(ACPI_RESOURCE *res, void *arg)
{
	struct gt_gpio_crs *c = arg;
	ACPI_RESOURCE_GPIO *g;
	ACPI_HANDLE h;

	if (res->Type != ACPI_RESOURCE_TYPE_GPIO)
		return (AE_OK);
	g = &res->Data.Gpio;
	if (g->PinTableLength == 0 || g->ResourceSource.StringPtr == NULL)
		return (AE_OK);
	if (ACPI_FAILURE(AcpiGetHandle(ACPI_ROOT_OBJECT,
	    g->ResourceSource.StringPtr, &h)))
		return (AE_OK);

	if (g->ConnectionType == ACPI_RESOURCE_GPIO_TYPE_INT) {
		if (c->int_ctrl == NULL) {
			c->int_ctrl = h;
			c->int_pin = g->PinTable[0];
		}
	} else if (g->ConnectionType == ACPI_RESOURCE_GPIO_TYPE_IO) {
		if (c->rst_ctrl == NULL) {
			c->rst_ctrl = h;
			c->rst_pin = g->PinTable[0];
		}
	}
	return (AE_OK);
}

static gpio_pin_t
gt_gpio_get(struct gt_softc *sc, ACPI_HANDLE ch, uint32_t pin,
    const char *what)
{
	device_t ctrl, bus;
	gpio_pin_t gp;
	int maxpin;

	ctrl = acpi_get_device(ch);
	if (ctrl == NULL || !device_is_attached(ctrl)) {
		device_printf(sc->dev, "%s: GPIO controller not attached\n",
		    what);
		return (NULL);
	}
	bus = GPIO_GET_BUS(ctrl);
	if (bus == NULL || !device_is_attached(bus)) {
		device_printf(sc->dev, "%s: %s has no gpiobus\n", what,
		    device_get_nameunit(ctrl));
		return (NULL);
	}
	if (GPIO_PIN_MAX(ctrl, &maxpin) != 0 || pin > (uint32_t)maxpin) {
		device_printf(sc->dev, "%s: pin %u out of range on %s\n", what,
		    pin, device_get_nameunit(ctrl));
		return (NULL);
	}
	if (gpio_pin_get_by_bus_pinnum(bus, pin, &gp) != 0) {
		device_printf(sc->dev, "%s: pin %u of %s is busy\n", what, pin,
		    device_get_nameunit(ctrl));
		return (NULL);
	}
	device_printf(sc->dev, "%s: %s pin %u\n", what,
	    device_get_nameunit(ctrl), pin);
	return (gp);
}

static void
gt_gpio_attach(struct gt_softc *sc)
{
	struct gt_gpio_crs c = { NULL, NULL, 0, 0 };
	ACPI_HANDLE h = acpi_get_handle(sc->dev);

	if (h == NULL ||
	    ACPI_FAILURE(AcpiWalkResources(h, "_CRS", gt_gpio_crs_cb, &c)))
		return;
	if (c.int_ctrl != NULL) {
		sc->gpio_int = gt_gpio_get(sc, c.int_ctrl, c.int_pin, "INT");
		if (sc->gpio_int != NULL)
			sc->gpio_pins |= 1;
	}
	if (c.rst_ctrl != NULL) {
		sc->gpio_rst = gt_gpio_get(sc, c.rst_ctrl, c.rst_pin, "RST");
		if (sc->gpio_rst != NULL)
			sc->gpio_pins |= 2;
	}
}

static void
gt_gpio_detach(struct gt_softc *sc)
{
	if (sc->gpio_int != NULL) {
		gpio_pin_release(sc->gpio_int);
		sc->gpio_int = NULL;
	}
	if (sc->gpio_rst != NULL) {
		gpio_pin_release(sc->gpio_rst);
		sc->gpio_rst = NULL;
	}
	sc->gpio_pins = 0;
}

static void
gt_msleep(int ms)
{
	pause_sbt("gtgpio", ms * SBT_1MS, 0, C_HARDCLOCK);
}

static void
gt_int_drive(struct gt_softc *sc, bool level)
{
	if (sc->gpio_int != NULL &&
	    gpio_pin_setflags(sc->gpio_int, GPIO_PIN_OUTPUT) == 0)
		(void)gpio_pin_set_active(sc->gpio_int, level);
}

static void
gt_int_release(struct gt_softc *sc)
{
	if (sc->gpio_int != NULL)
		(void)gpio_pin_setflags(sc->gpio_int, GPIO_PIN_INPUT);
}

/*
 * Hardware reset with bus address selection: INT is driven while RST rises,
 * high selects 0x14 (written 0x28), low selects 0x5d.  Sleeps, so it must be
 * called from a thread with the poll task not running.
 */
static int
gt_gpio_reset(struct gt_softc *sc)
{
	int err;

	if (sc->gpio_rst == NULL)
		return (ENXIO);
	device_printf(sc->dev, "resetting controller through GPIO\n");

	err = gpio_pin_setflags(sc->gpio_rst, GPIO_PIN_OUTPUT);
	if (err == 0)
		err = gpio_pin_set_active(sc->gpio_rst, false);
	if (err != 0)
		return (err);
	gt_msleep(20);				/* RST low, > 10 ms */

	gt_int_drive(sc, sc->addr == 0x28);
	DELAY(200);				/* > 100 us */

	err = gpio_pin_set_active(sc->gpio_rst, true);
	if (err != 0)
		return (err);
	gt_msleep(6);				/* > 5 ms */

	if (sc->gpio_int != NULL) {
		gt_int_drive(sc, false);
		gt_msleep(50);
		gt_int_release(sc);
	} else
		gt_msleep(50);			/* controller boot */
	return (0);
}

/*
 * Sleep: INT low, then the screen-off command.  The controller stops
 * answering on the bus until it is woken with an INT pulse (or reset).
 */
static void
gt_sleep(struct gt_softc *sc)
{
	if (!sc->suspend_sleep ||
	    (sc->gpio_int == NULL && sc->gpio_rst == NULL))
		return;

	gt_int_drive(sc, false);
	gt_msleep(20);
	if (gt_write8(sc, GT_REG_COMMAND, GT_CMD_SLEEP) == 0) {
		sc->asleep = true;
		sc->cnt_sleep++;
		GT_DBG(sc, GT_DBG_INIT, "controller asleep\n");
	} else {
		gt_int_release(sc);
		device_printf(sc->dev, "sleep command failed\n");
	}
}

/* Wake: INT high for a few ms, then low for 50 ms, then back to input. */
static void
gt_wake(struct gt_softc *sc)
{
	if (!sc->asleep)
		return;
	sc->asleep = false;
	sc->cnt_wake++;
	GT_DBG(sc, GT_DBG_INIT, "waking controller\n");

	if (sc->gpio_int != NULL) {
		gt_int_drive(sc, true);
		gt_msleep(5);
		gt_int_drive(sc, false);
		gt_msleep(50);
		gt_int_release(sc);
	}
	gt_msleep(20);
}

/*
 * Read the controller identity and panel geometry.  The bus child address
 * (0x14) is stored as a 7-bit value, iicbus wants it shifted: 0x28.
 */
static int
gt_identify(struct gt_softc *sc, bool quiet)
{
	uint8_t raw[6], cfg[6];
	int attempt, err = ENXIO;

	sc->addr = (uint8_t)(iicbus_get_addr(sc->dev) << 1);
	for (attempt = 0; attempt < GT_ID_RETRIES; attempt++) {
		memset(raw, 0, sizeof(raw));
		err = gt_read(sc, GT_REG_PRODUCT, raw, sizeof(raw));
		if (err == 0 && isdigit(raw[0]) && isdigit(raw[1]) &&
		    isdigit(raw[2]) && isdigit(raw[3]))
			break;
		err = ENXIO;
		if (attempt < GT_ID_RETRIES - 1)
			pause("gtid", hz / 50);
	}
	if (err != 0) {
		sc->cnt_ident++;
		return (err);
	}

	sc->fw = raw[4] | (raw[5] << 8);
	sc->product = (raw[0] - '0') * 1000 + (raw[1] - '0') * 100 +
	    (raw[2] - '0') * 10 + (raw[3] - '0');
	if (!quiet) {
		device_printf(sc->dev, "addr 0x%02x, product %.4s, fw 0x%04x\n",
		    sc->addr, (const char *)raw, sc->fw);
		if (sc->product != GT_PRODUCT)
			device_printf(sc->dev,
			    "unexpected product, expected %d\n", GT_PRODUCT);
	}

	err = gt_read(sc, GT_REG_CONFIG, cfg, sizeof(cfg));
	if (err != 0)
		return (err);

	sc->x_max = cfg[1] | (cfg[2] << 8);
	sc->y_max = cfg[3] | (cfg[4] << 8);
	sc->nslots = cfg[5] & 0x0f;
	if (sc->x_max == 0 || sc->y_max == 0) {
		sc->x_max = GT_DEFAULT_X;
		sc->y_max = GT_DEFAULT_Y;
	}
	if (sc->x_override > 0 && sc->x_override <= 0xffff)
		sc->x_max = sc->x_override;
	if (sc->y_override > 0 && sc->y_override <= 0xffff)
		sc->y_max = sc->y_override;
	if (sc->nslots == 0 || sc->nslots > GT_MAX_SLOTS)
		sc->nslots = GT_MAX_SLOTS;

	if (!quiet)
		device_printf(sc->dev, "panel %ux%u, %u slots\n", sc->x_max,
		    sc->y_max, sc->nslots);
	return (0);
}

/* Identify; if the controller is silent, reset it through GPIO and retry. */
static int
gt_identify_reset(struct gt_softc *sc, bool quiet)
{
	int err;

	err = gt_identify(sc, quiet);
	if (err != 0 && sc->gpio_rst != NULL && gt_gpio_reset(sc) == 0)
		err = gt_identify(sc, quiet);
	return (err);
}

static void
gt_release_all(struct gt_softc *sc)
{
	bool any = false;
	u_int i;

	for (i = 0; i < sc->nslots; i++) {
		if (sc->track[i] < 0)
			continue;
		evdev_push_event(sc->evdev, EV_ABS, ABS_MT_SLOT, i);
		evdev_push_event(sc->evdev, EV_ABS, ABS_MT_TRACKING_ID, -1);
		sc->track[i] = -1;
		any = true;
	}
	if (any) {
		evdev_push_event(sc->evdev, EV_KEY, BTN_TOUCH, 0);
		evdev_sync(sc->evdev);
	}
}

static void
gt_report(struct gt_softc *sc, const uint8_t *rec, u_int n)
{
	struct evdev_dev *ev = sc->evdev;
	uint16_t present = 0;
	int first_x = 0, first_y = 0;
	u_int i;

	for (i = 0; i < n; i++) {
		const uint8_t *r = rec + i * GT_REC_LEN;
		u_int slot = r[0] & 0x0f;
		int x = r[1] | (r[2] << 8);
		int y = r[3] | (r[4] << 8);
		int width = r[5] | (r[6] << 8);

		if (slot >= sc->nslots) {
			sc->cnt_slot++;
			continue;
		}
		if (sc->swap_xy) {
			int t = x;

			x = y;
			y = t;
		}
		if (sc->invert_x)
			x = (int)sc->abs_x - 1 - x;
		if (sc->invert_y)
			y = (int)sc->abs_y - 1 - y;
		x = MIN(MAX(x, 0), (int)sc->abs_x - 1);
		y = MIN(MAX(y, 0), (int)sc->abs_y - 1);
		GT_DBG(sc, GT_DBG_TOUCH, "slot %u x %d y %d w %d\n", slot, x, y,
		    width);
		if (width > GT_MAX_WIDTH)
			width = GT_MAX_WIDTH;

		evdev_push_event(ev, EV_ABS, ABS_MT_SLOT, slot);
		if (sc->track[slot] < 0) {
			sc->track[slot] = sc->next_track;
			sc->next_track = (sc->next_track + 1) & 0x7fff;
			evdev_push_event(ev, EV_ABS, ABS_MT_TRACKING_ID,
			    sc->track[slot]);
		}
		evdev_push_event(ev, EV_ABS, ABS_MT_POSITION_X, x);
		evdev_push_event(ev, EV_ABS, ABS_MT_POSITION_Y, y);
		evdev_push_event(ev, EV_ABS, ABS_MT_TOUCH_MAJOR, width);
		if (present == 0) {
			first_x = x;
			first_y = y;
		}
		present |= 1 << slot;
	}

	for (i = 0; i < sc->nslots; i++) {
		if (sc->track[i] >= 0 && (present & (1 << i)) == 0) {
			evdev_push_event(ev, EV_ABS, ABS_MT_SLOT, i);
			evdev_push_event(ev, EV_ABS, ABS_MT_TRACKING_ID, -1);
			sc->track[i] = -1;
		}
	}

	if (present != 0) {
		evdev_push_event(ev, EV_ABS, ABS_X, first_x);
		evdev_push_event(ev, EV_ABS, ABS_Y, first_y);
	}
	evdev_push_event(ev, EV_KEY, BTN_TOUCH, present != 0);
	evdev_sync(ev);
}

/*
 * One bus pass: the status byte and as many point records as the previous
 * report had (at least one) are fetched in a single read, further records
 * (if any) in a second one.
 * Returns -1 on a bus error (including a failed acknowledge), 0 if there
 * was nothing to report, 1 if a report was delivered.
 */
static int
gt_fetch(struct gt_softc *sc)
{
	uint8_t buf[1 + GT_MAX_SLOTS * GT_REC_LEN];
	u_int n, want;
	int err;

	want = MIN(MAX(sc->last_n, 1), sc->nslots);
	if (gt_read(sc, GT_REG_STATE, buf, 1 + want * GT_REC_LEN) != 0)
		return (-1);
	if ((buf[0] & GT_STATE_VALID) == 0)
		return (0);

	n = buf[0] & GT_STATE_COUNT;
	if (n > sc->nslots) {
		sc->cnt_count++;
		GT_DBG(sc, GT_DBG_IO, "bad point count %u\n", n);
		(void)gt_write8(sc, GT_REG_STATE, 0);
		return (0);
	}
	if (n > want && gt_read(sc, GT_REG_POINTS + want * GT_REC_LEN,
	    buf + 1 + want * GT_REC_LEN, (n - want) * GT_REC_LEN) != 0)
		return (-1);

	sc->last_n = n;
	err = gt_write8(sc, GT_REG_STATE, 0);	/* acknowledge */
	gt_report(sc, buf + 1, n);
	return (err != 0 ? -1 : 1);
}

static void
gt_clear_errors(struct gt_softc *sc)
{
	sc->errors = 0;
	sc->err_reported = false;
	sc->recover_fail = 0;
}

static void
gt_recover(struct gt_softc *sc)
{
	device_printf(sc->dev, "bus errors (last %d), reinitialising controller\n",
	    sc->last_err);
	sc->stat_recover++;
	gt_release_all(sc);
	gt_acpi_power_on(sc);
	pause("gtrec", hz / 20);
	if (gt_identify_reset(sc, true) == 0) {
		(void)gt_write8(sc, GT_REG_STATE, 0);
		gt_clear_errors(sc);
	} else {
		sc->errors = 1;		/* try again after a few more failures */
		if (sc->recover_fail < 3)
			sc->recover_fail++;
	}
}

static void
gt_irq_teardown(struct gt_softc *sc)
{
	if (sc->irq_res == NULL)
		return;
	bus_teardown_intr(sc->dev, sc->irq_res, sc->irq_cookie);
	bus_release_resource(sc->dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
	sc->irq_res = NULL;
	sc->using_irq = 0;
}

static void
gt_schedule(struct gt_softc *sc)
{
	sbintime_t pr = 0;
	int t, hzv;

	if (sc->stopping || sc->suspended || !sc->opened)
		return;
	if (sc->irq_res != NULL) {
		t = hz / GT_WATCHDOG_HZ;
		pr = 50 * SBT_1MS;
	} else if ((int)(sc->active_until - ticks) > 0) {
		hzv = MAX(1, sc->poll_active_hz);
		t = hz / hzv;
	} else {
		hzv = MAX(1, sc->poll_idle_hz);
		t = hz / hzv;
		pr = (t * tick_sbt) / 4;
	}
	if (sc->errors > 1)
		t = MAX(t, hz / 5);
	if (sc->recover_fail > 0)
		t = MAX(t, hz << sc->recover_fail);
	t = MAX(1, t);
	taskqueue_enqueue_timeout_sbt(sc->tq, &sc->poll_task, t * tick_sbt,
	    pr, C_HARDCLOCK);
}

static void
gt_work(void *arg, int pending __unused)
{
	struct gt_softc *sc = arg;
	bool from_irq;
	int r;

	if (sc->stopping || sc->suspended || !sc->opened)
		return;

	from_irq = atomic_readandclear_int(&sc->irq_flag) != 0;
	r = gt_fetch(sc);
	if (r < 0) {
		if (!sc->err_reported) {
			device_printf(sc->dev, "bus error\n");
			sc->err_reported = true;
		}
		if (++sc->errors >= GT_ERR_LIMIT)
			gt_recover(sc);
	} else {
		gt_clear_errors(sc);
		if (r > 0) {
			sc->stat_hits++;
			sc->active_until = ticks + hz / 2;
			if (from_irq)
				sc->irq_miss = 0;
			else if (sc->irq_res != NULL &&
			    ++sc->irq_miss >= GT_IRQ_MISS_LIMIT) {
				device_printf(sc->dev,
				    "interrupt does not fire, polling\n");
				gt_irq_teardown(sc);
			}
		}
	}
	gt_schedule(sc);
}

static int
gt_intr(void *arg)
{
	struct gt_softc *sc = arg;

	atomic_add_int(&sc->stat_irq, 1);
	if (sc->stopping || sc->suspended || !sc->opened)
		return (FILTER_HANDLED);
	atomic_set_int(&sc->irq_flag, 1);
	taskqueue_enqueue(sc->tq, &sc->irq_task);
	return (FILTER_HANDLED);
}

static void
gt_irq_setup(struct gt_softc *sc)
{
	int err;

	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ,
	    &sc->irq_rid, RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res == NULL) {
		device_printf(sc->dev, "no interrupt resource, polling\n");
		return;
	}
	err = bus_setup_intr(sc->dev, sc->irq_res,
	    INTR_TYPE_MISC | INTR_MPSAFE, gt_intr, NULL, sc, &sc->irq_cookie);
	if (err != 0) {
		device_printf(sc->dev, "bus_setup_intr: %d, polling\n", err);
		bus_release_resource(sc->dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);
		sc->irq_res = NULL;
		return;
	}
	sc->using_irq = 1;
	device_printf(sc->dev, "using interrupt\n");
}

static void
gt_tunable(device_t dev, const char *name, int *var)
{
	char path[64];

	snprintf(path, sizeof(path), "dev.%s.%d.%s", device_get_name(dev),
	    device_get_unit(dev), name);
	TUNABLE_INT_FETCH(path, var);
}

static void
gt_sysctl(struct gt_softc *sc)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
	struct sysctl_oid_list *kids =
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));
	struct sysctl_oid *node;
	struct sysctl_oid_list *st;

	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "invert_x", CTLFLAG_RW,
	    &sc->invert_x, 0, "Mirror the X axis");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "invert_y", CTLFLAG_RW,
	    &sc->invert_y, 0, "Mirror the Y axis");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "poll_active_hz", CTLFLAG_RW,
	    &sc->poll_active_hz, 0, "Polling rate while touched");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "poll_idle_hz", CTLFLAG_RW,
	    &sc->poll_idle_hz, 0, "Polling rate while idle");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "using_irq", CTLFLAG_RD,
	    &sc->using_irq, 0, "Interrupt in use (0 = polling)");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "gpio_pins", CTLFLAG_RD,
	    &sc->gpio_pins, 0, "GPIO lines found (bit 0: INT, bit 1: RST)");
	SYSCTL_ADD_UINT(ctx, kids, OID_AUTO, "irq_count", CTLFLAG_RD,
	    &sc->stat_irq, 0, "Interrupts received");
	SYSCTL_ADD_UINT(ctx, kids, OID_AUTO, "reports", CTLFLAG_RD,
	    &sc->stat_hits, 0, "Reports delivered");
	SYSCTL_ADD_UINT(ctx, kids, OID_AUTO, "recoveries", CTLFLAG_RD,
	    &sc->stat_recover, 0, "Controller reinitialisations");

	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "swap_xy", CTLFLAG_RD,
	    &sc->swap_xy, 0, "Axes swapped (set before load)");
	SYSCTL_ADD_U16(ctx, kids, OID_AUTO, "x_max", CTLFLAG_RD,
	    &sc->x_max, 0, "Panel width (override: x_override tunable)");
	SYSCTL_ADD_U16(ctx, kids, OID_AUTO, "y_max", CTLFLAG_RD,
	    &sc->y_max, 0, "Panel height (override: y_override tunable)");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "suspend_sleep", CTLFLAG_RW,
	    &sc->suspend_sleep, 0, "Put the controller to sleep in suspend");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "debug", CTLFLAG_RW,
	    &sc->debug, 0,
	    "Debug mask: 1 init/power, 2 bus errors, 4 touch reports");

	node = SYSCTL_ADD_NODE(ctx, kids, OID_AUTO, "stats",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "Error counters");
	st = SYSCTL_CHILDREN(node);
	SYSCTL_ADD_UINT(ctx, st, OID_AUTO, "read_errors", CTLFLAG_RD,
	    &sc->cnt_read, 0, "Failed register reads");
	SYSCTL_ADD_UINT(ctx, st, OID_AUTO, "write_errors", CTLFLAG_RD,
	    &sc->cnt_write, 0, "Failed register writes");
	SYSCTL_ADD_UINT(ctx, st, OID_AUTO, "identify_failures", CTLFLAG_RD,
	    &sc->cnt_ident, 0, "Controller identification failures");
	SYSCTL_ADD_UINT(ctx, st, OID_AUTO, "bad_count", CTLFLAG_RD,
	    &sc->cnt_count, 0, "Reports with an invalid point count");
	SYSCTL_ADD_UINT(ctx, st, OID_AUTO, "bad_slot", CTLFLAG_RD,
	    &sc->cnt_slot, 0, "Points with an invalid slot");
	SYSCTL_ADD_UINT(ctx, st, OID_AUTO, "sleeps", CTLFLAG_RD,
	    &sc->cnt_sleep, 0, "Times the controller was put to sleep");
	SYSCTL_ADD_UINT(ctx, st, OID_AUTO, "wakeups", CTLFLAG_RD,
	    &sc->cnt_wake, 0, "Times the controller was woken");
	SYSCTL_ADD_INT(ctx, st, OID_AUTO, "last_error", CTLFLAG_RD,
	    &sc->last_err, 0, "errno of the last bus error");
}

static int
gt_ev_open(struct evdev_dev *ev)
{
	struct gt_softc *sc = evdev_get_softc(ev);

	sc->opened = true;
	atomic_store_rel_int(&sc->irq_flag, 0);
	if (sc->tq != NULL && !sc->stopping)
		taskqueue_enqueue_timeout(sc->tq, &sc->poll_task, 0);
	return (0);
}

static int
gt_ev_close(struct evdev_dev *ev)
{
	struct gt_softc *sc = evdev_get_softc(ev);

	sc->opened = false;
	return (0);
}

static const struct evdev_methods gt_evdev_methods = {
	.ev_open = gt_ev_open,
	.ev_close = gt_ev_close,
};

static int
gt_probe(device_t dev)
{
	ACPI_HANDLE h;
	int i;

	h = acpi_get_handle(dev);
	if (h == NULL)
		return (ENXIO);
	for (i = 0; gt_acpi_ids[i] != NULL; i++) {
		if (acpi_MatchHid(h, gt_acpi_ids[i]) != ACPI_MATCHHID_NOMATCH) {
			device_set_desc(dev, "Goodix I2C touchscreen (ACPI)");
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

static int
gt_attach(device_t dev)
{
	struct gt_softc *sc = device_get_softc(dev);
	struct evdev_dev *ev;
	u_int i;
	int err;

	sc->dev = dev;
	for (i = 0; i < GT_MAX_SLOTS; i++)
		sc->track[i] = -1;
	sc->poll_active_hz = GT_DEF_ACTIVE_HZ;
	sc->poll_idle_hz = GT_DEF_IDLE_HZ;
	sc->suspend_sleep = 1;
	gt_tunable(dev, "invert_x", &sc->invert_x);
	gt_tunable(dev, "invert_y", &sc->invert_y);
	gt_tunable(dev, "swap_xy", &sc->swap_xy);
	gt_tunable(dev, "x_override", &sc->x_override);
	gt_tunable(dev, "y_override", &sc->y_override);
	gt_tunable(dev, "debug", &sc->debug);
	gt_tunable(dev, "suspend_sleep", &sc->suspend_sleep);
	gt_tunable(dev, "poll_active_hz", &sc->poll_active_hz);
	gt_tunable(dev, "poll_idle_hz", &sc->poll_idle_hz);

	gt_gpio_attach(sc);

	err = gt_identify(sc, false);
	if (err != 0) {
		gt_acpi_power_on(sc);
		pause("gtps0", hz / 20);
		err = gt_identify_reset(sc, false);
	}
	if (err != 0) {
		device_printf(dev, "controller did not respond (%d)\n", err);
		gt_gpio_detach(sc);
		return (err);
	}

	sc->abs_x = sc->swap_xy ? sc->y_max : sc->x_max;
	sc->abs_y = sc->swap_xy ? sc->x_max : sc->y_max;
	if (sc->swap_xy)
		device_printf(dev, "axes swapped, reporting %ux%u\n",
		    sc->abs_x, sc->abs_y);

	ev = evdev_alloc();
	if (ev == NULL) {
		gt_gpio_detach(sc);
		return (ENOMEM);
	}
	evdev_set_name(ev, "Goodix Capacitive TouchScreen");
	evdev_set_phys(ev, device_get_nameunit(dev));
	evdev_set_id(ev, BUS_I2C, 0x0416, sc->product, sc->fw);
	evdev_support_prop(ev, INPUT_PROP_DIRECT);
	evdev_support_event(ev, EV_SYN);
	evdev_support_event(ev, EV_ABS);
	evdev_support_event(ev, EV_KEY);
	evdev_support_key(ev, BTN_TOUCH);
	evdev_support_abs(ev, ABS_X, 0, sc->abs_x - 1, 0, 0, 0);
	evdev_support_abs(ev, ABS_Y, 0, sc->abs_y - 1, 0, 0, 0);
	evdev_support_abs(ev, ABS_MT_SLOT, 0, sc->nslots - 1, 0, 0, 0);
	evdev_support_abs(ev, ABS_MT_TRACKING_ID, -1, 0x7fff, 0, 0, 0);
	evdev_support_abs(ev, ABS_MT_POSITION_X, 0, sc->abs_x - 1, 0, 0, 0);
	evdev_support_abs(ev, ABS_MT_POSITION_Y, 0, sc->abs_y - 1, 0, 0, 0);
	evdev_support_abs(ev, ABS_MT_TOUCH_MAJOR, 0, GT_MAX_WIDTH, 0, 0, 0);

	sc->tq = taskqueue_create_fast("goodix_acpi", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->tq);
	taskqueue_start_threads(&sc->tq, 1, PI_TTY, "%s taskq",
	    device_get_nameunit(dev));
	TASK_INIT(&sc->irq_task, 0, gt_work, sc);
	TIMEOUT_TASK_INIT(sc->tq, &sc->poll_task, 0, gt_work, sc);

	evdev_set_methods(ev, sc, &gt_evdev_methods);
	err = evdev_register(ev);
	if (err != 0) {
		device_printf(dev, "evdev_register: %d\n", err);
		taskqueue_free(sc->tq);
		sc->tq = NULL;
		evdev_free(ev);
		gt_gpio_detach(sc);
		return (err);
	}
	sc->evdev = ev;

	(void)gt_write8(sc, GT_REG_STATE, 0);	/* drop stale report */
	gt_sysctl(sc);
	gt_irq_setup(sc);

	return (0);
}

static int
gt_detach(device_t dev)
{
	struct gt_softc *sc = device_get_softc(dev);

	sc->stopping = true;
	if (sc->tq != NULL) {
		taskqueue_cancel_timeout(sc->tq, &sc->poll_task, NULL);
		taskqueue_drain_timeout(sc->tq, &sc->poll_task);
		taskqueue_drain(sc->tq, &sc->irq_task);
	}
	gt_irq_teardown(sc);
	if (sc->tq != NULL) {
		taskqueue_drain(sc->tq, &sc->irq_task);
		taskqueue_free(sc->tq);
		sc->tq = NULL;
	}
	if (sc->evdev != NULL) {
		evdev_unregister(sc->evdev);
		evdev_free(sc->evdev);
		sc->evdev = NULL;
	}
	gt_gpio_detach(sc);
	return (0);
}

static int
gt_suspend(device_t dev)
{
	struct gt_softc *sc = device_get_softc(dev);

	sc->suspended = true;
	taskqueue_cancel_timeout(sc->tq, &sc->poll_task, NULL);
	taskqueue_drain_timeout(sc->tq, &sc->poll_task);
	taskqueue_drain(sc->tq, &sc->irq_task);
	gt_release_all(sc);
	gt_sleep(sc);
	return (0);
}

static int
gt_resume(device_t dev)
{
	struct gt_softc *sc = device_get_softc(dev);

	gt_acpi_power_on(sc);
	pause("gtres", hz / 20);
	gt_wake(sc);
	if (gt_identify_reset(sc, true) != 0)
		device_printf(dev, "controller silent after resume\n");
	(void)gt_write8(sc, GT_REG_STATE, 0);
	gt_clear_errors(sc);
	atomic_store_rel_int(&sc->irq_flag, 0);
	sc->irq_miss = 0;
	sc->suspended = false;
	if (sc->opened)
		taskqueue_enqueue_timeout(sc->tq, &sc->poll_task, hz / 10);
	return (0);
}

static device_method_t gt_methods[] = {
	DEVMETHOD(device_probe,		gt_probe),
	DEVMETHOD(device_attach,	gt_attach),
	DEVMETHOD(device_detach,	gt_detach),
	DEVMETHOD(device_suspend,	gt_suspend),
	DEVMETHOD(device_resume,	gt_resume),
	DEVMETHOD_END
};

static driver_t gt_driver = {
	"goodix_acpi",
	gt_methods,
	sizeof(struct gt_softc)
};

DRIVER_MODULE(goodix_acpi, iicbus, gt_driver, 0, 0);
MODULE_DEPEND(goodix_acpi, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
MODULE_DEPEND(goodix_acpi, evdev, 1, 1, 1);
MODULE_DEPEND(goodix_acpi, gpiobus, 1, 1, 1);
MODULE_VERSION(goodix_acpi, 1);
