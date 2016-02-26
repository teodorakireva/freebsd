/*-
 * Copyright (c) 2016 Teodora Kireva <teodora_kireva@smartcom.bg> 
 * Copyright (c) 2016 Tsvetko Tsvetkov <tsvetko_tsvetkov@smartcom.bg>
 * All rights reserved.
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "h3_pwmreg.h"

#define PWM_INIT_PERIOD		1000
#define PWM_INIT_DUTY		50 /* 50% duty */
#define PWM_CLOCK		24000000UL

#define DUTY_TO_ACT_CYCLES(_duty, _period) (_duty * _period / 100)

#define PWM_LOCK_INIT(_sc)	mtx_init(&(_sc)->mtx, \
    device_get_nameunit(_sc->dev), "h3 pwm softc", MTX_DEF)
#define PWM_DESTROY(_sc)	mtx_destroy(&(_sc)->mtx)
#define PWM_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define PWM_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)

#define PWM_WRITE4(_sc, reg, value) \
    bus_write_4((_sc)->mem_res, reg, value)
#define PWM_READ4(_sc, reg)	bus_read_4((_sc)->mem_res, reg)

#define ARRAY_SIZE(x)		(sizeof(x)/sizeof(x[0]))

struct pwm {
	int 			clkdiv;
	int			freq;
	int			duty;
	int			period;
	struct sysctl_oid	*clkdiv_oid;
	struct sysctl_oid	*freq_oid;
	struct sysctl_oid	*period_oid;
	struct sysctl_oid	*duty_oid;
};

struct h3_pwm_softc {
	device_t 		dev;
	struct mtx		mtx;
	struct resource 	*mem_res;
	struct pwm		pwm0, pwm1;
	int			regread;
};

static void h3_pwm_sysctl_init(struct h3_pwm_softc *sc);
static void h3_pwm_def_init(struct h3_pwm_softc *sc);
static int h3_pwm_sysctl_period(SYSCTL_HANDLER_ARGS);
static int h3_pwm_sysctl_duty(SYSCTL_HANDLER_ARGS);
static int h3_pwm_sysctl_clkdiv(SYSCTL_HANDLER_ARGS);
static int h3_pwm_sysctl_freq(SYSCTL_HANDLER_ARGS);
static void h3_pwm_calc_freq(struct pwm *pwm);

static int h3_pwm_sysctl_regread(SYSCTL_HANDLER_ARGS);

static int h3_pwm_clkdiv[] = { 
		120,
		180,
		240,
		360,
		48,
		-1,
		-1,
		-1,
		12000,
		24000,
		36000,
		48000,
		72000,
		-1,
		-1,
		1
};

static int
h3_pwm_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-pwm"))
		return (ENXIO);

	device_set_desc(dev, "H3 PWM");

	return (BUS_PROBE_DEFAULT);
}

static int
h3_pwm_attach(device_t dev)
{
	int 			rid;
	struct h3_pwm_softc 	*sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	PWM_LOCK_INIT(sc);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate memory resources\n");
		return (ENXIO);
	}

	h3_pwm_sysctl_init(sc);
	h3_pwm_def_init(sc);

	return (0);
}

static int
h3_pwm_sysctl_regread(SYSCTL_HANDLER_ARGS)
{
	int err;
	int reg_read, regval;
	struct h3_pwm_softc *sc  = (struct h3_pwm_softc *)arg1;

	reg_read = sc->regread;
	err = sysctl_handle_int(oidp, &reg_read, sizeof(reg_read), req);
	if (err != 0 || req->newptr == NULL)
		return (err);

	regval = PWM_READ4(sc, reg_read);
	printf("Reg: 0x%x, val: 0x%x\n", reg_read, regval);

	return (0);
}

static void
h3_pwm_sysctl_init(struct h3_pwm_softc *sc)
{
	struct sysctl_ctx_list 	*ctx;
	struct sysctl_oid 	*tree;

	struct sysctl_oid	*oid;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);
		/* PWM 0 sysctls */

	oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "regread", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_regread, "I", "PWM Read Reg");

	sc->pwm0.clkdiv_oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "clkdiv0", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_clkdiv, "I", "PWM 0 clock prescaler");

	sc->pwm0.freq_oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "freq0", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_freq, "I", "PWM 0 frequency");

	sc->pwm0.period_oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "period0", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_period, "I", "PWM 0 period");

	sc->pwm0.duty_oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "duty0", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_duty, "I", "PWM 0 duty in percents");
		/* PWM 1 sysctls */
	sc->pwm1.clkdiv_oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "clkdiv1", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_clkdiv, "I", "PWM 1 clock prescaler");

	sc->pwm1.freq_oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "freq1", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_freq, "I", "PWM 1 frequency");

	sc->pwm1.period_oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "period1", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_period, "I", "PWM 1 period");

	sc->pwm1.duty_oid = SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
	    OID_AUTO, "duty1", CTLTYPE_INT | CTLFLAG_RW, sc, 0,
	    h3_pwm_sysctl_duty, "I", "PWM 1 duty in percents");
}

static void
h3_pwm_def_init(struct h3_pwm_softc *sc)
{
	int ctrl, period, ent_cys, act_cys;

		/* Setup PWM's ch 0 */
		/* clkdiv = 1, active state = high */
	ctrl = (0xf | PWM_CH_ACT_STA) << PWM_CH0_CTRL_SHIFT_N;
	PWM_WRITE4(sc, PWM_CH_CTRL, ctrl);

	ent_cys = PWM_INIT_PERIOD << 16;
	act_cys = DUTY_TO_ACT_CYCLES(PWM_INIT_DUTY, PWM_INIT_PERIOD);

	period = ent_cys | act_cys;
	PWM_WRITE4(sc, PWM_CH0_PERIOD, period);

	sc->pwm0.period = PWM_INIT_PERIOD;
	sc->pwm0.duty = PWM_INIT_DUTY;
	sc->pwm0.clkdiv = 0xf;
	h3_pwm_calc_freq(&sc->pwm0);

		/* pass PWM0 clk */
	ctrl |= SCLK_CH_GATING << PWM_CH0_CTRL_SHIFT_N;
	PWM_WRITE4(sc, PWM_CH_CTRL, ctrl);
		/* en ch0 */
	ctrl |= PWM_CH_EN << PWM_CH0_CTRL_SHIFT_N;
	PWM_WRITE4(sc, PWM_CH_CTRL, ctrl);

		/* Setup PWM's ch 1 */
		/* clkdiv = 1, active state = high */
	ctrl |= (0xf | PWM_CH_ACT_STA) << PWM_CH1_CTRL_SHIFT_N;
	PWM_WRITE4(sc, PWM_CH_CTRL, ctrl);

	ent_cys = PWM_INIT_PERIOD << 16;
	act_cys = DUTY_TO_ACT_CYCLES(PWM_INIT_DUTY, PWM_INIT_PERIOD);

	period = ent_cys | act_cys;
	PWM_WRITE4(sc, PWM_CH1_PERIOD, period);

	sc->pwm1.period = PWM_INIT_PERIOD;
	sc->pwm1.duty = PWM_INIT_DUTY;
	sc->pwm1.clkdiv = 0xf;
	h3_pwm_calc_freq(&sc->pwm1);

		/* pass PWM1 clk */
	ctrl |= SCLK_CH_GATING << PWM_CH1_CTRL_SHIFT_N;
	PWM_WRITE4(sc, PWM_CH_CTRL, ctrl);
		/* en ch1 */
	ctrl |= PWM_CH_EN << PWM_CH1_CTRL_SHIFT_N;
	PWM_WRITE4(sc, PWM_CH_CTRL, ctrl);
}

static int
h3_pwm_sysctl_clkdiv(SYSCTL_HANDLER_ARGS)
{
	int 			err, r_shift, i;
	int	 		regval, clkdiv;
	struct pwm 		*p;
	struct h3_pwm_softc 	*sc;

	sc = (struct h3_pwm_softc *)arg1;

	if (oidp == sc->pwm0.duty_oid) {
		p = &sc->pwm0;
		r_shift = PWM_CH0_CTRL_SHIFT_N;
	} else {
		p = &sc->pwm1;
		r_shift = PWM_CH1_CTRL_SHIFT_N;
	}

	PWM_LOCK(sc);
	clkdiv = h3_pwm_clkdiv[p->clkdiv];
	PWM_UNLOCK(sc);

	err = sysctl_handle_int(oidp, &clkdiv, sizeof(clkdiv), req);
	if (err != 0 || req->newptr == NULL)
		return (err);

	PWM_LOCK(sc);
	if (clkdiv <= 0) {
		PWM_UNLOCK(sc);
		return (EINVAL);
	}
		
	if (clkdiv != h3_pwm_clkdiv[p->clkdiv]) {
		for (i = 0; i < ARRAY_SIZE(h3_pwm_clkdiv); i++)
			if (clkdiv >= h3_pwm_clkdiv[i])
				p->clkdiv = i;
		/* Disable PWM's clock first */
		regval = PWM_READ4(sc, PWM_CH_CTRL);
		if (regval & (SCLK_CH_GATING << r_shift)) {
			regval &= ~(SCLK_CH_GATING << r_shift);
			PWM_WRITE4(sc, PWM_CH_CTRL, regval);
		}

		/* Write the new prescaler value */
		regval = PWM_READ4(sc, PWM_CH_CTRL);
		regval &= ~(0xf << r_shift);
		regval |= (p->clkdiv << r_shift);

		PWM_WRITE4(sc, PWM_CH_CTRL, regval);
		/* Enable PWM's clock */
		regval = PWM_READ4(sc, PWM_CH_CTRL);
		regval |= SCLK_CH_GATING << r_shift;
		PWM_WRITE4(sc, PWM_CH_CTRL, regval);

		/*printf("Control reg : 0x%x\n", regval);*/
	}
	PWM_UNLOCK(sc);

	return (0);
}

static int
h3_pwm_sysctl_period(SYSCTL_HANDLER_ARGS)
{
	int 			err;
	int 			period, reg, regval;
	short int 		act_cys, ent_cys;
	struct pwm 		*p;
	struct h3_pwm_softc 	*sc;

	sc = (struct h3_pwm_softc *)arg1;

	if (oidp == sc->pwm0.period_oid) {
		reg = PWM_CH0_PERIOD;
		p = &sc->pwm0;
	} else {
		reg = PWM_CH1_PERIOD;
		p = &sc->pwm1;
	}

	PWM_LOCK(sc);
	period = p->period;
	PWM_UNLOCK(sc);

	err = sysctl_handle_int(oidp, &period, 0, req);
	if (err || req->newptr == NULL)
		return (err);

	PWM_LOCK(sc);
	if (period > USHRT_MAX)
		period = USHRT_MAX;

	if (period <= 0) {
		PWM_UNLOCK(sc);
		return (EINVAL);
	}

	if (period != p->period) {
		regval = PWM_READ4(sc, reg);
		ent_cys = (regval >> 16) & 0xffff;
		act_cys = regval & 0xffff;

		if (act_cys > ent_cys) {
			device_printf(sc->dev, "Err: active > entire cycles\n");
			PWM_UNLOCK(sc);
			return (EINVAL);
		}

		regval &= ~(0xffff << 16);
		regval |= period << 16;
		PWM_WRITE4(sc, reg, regval);

		p->period = period;
		h3_pwm_calc_freq(p);
	}
	PWM_UNLOCK(sc);

	return (err);
}

static int
h3_pwm_sysctl_freq(SYSCTL_HANDLER_ARGS)
{
	return (0);
}

static int
h3_pwm_sysctl_duty(SYSCTL_HANDLER_ARGS)
{
	int			duty, act_cys, ent_cys;
	int			regval, reg, err;
	struct pwm 		*p;
	struct h3_pwm_softc 	*sc;

	sc = (struct h3_pwm_softc *)arg1;

	if (oidp == sc->pwm0.duty_oid) {
		p = &sc->pwm0;
		reg = PWM_CH0_PERIOD;
	} else {
		p = &sc->pwm1;
		reg = PWM_CH1_PERIOD;
	}

	PWM_LOCK(sc);
	duty = p->duty;
	PWM_UNLOCK(sc);

	err = sysctl_handle_int(oidp, &duty, 0, req);
	if (err || req->newptr == NULL)
		return (err);

	PWM_LOCK(sc);
	if (duty < 0 || duty > 100) {
		PWM_UNLOCK(sc);
		return (EINVAL);
	}

	if (duty != p->duty) {
		regval = PWM_READ4(sc, reg);
		act_cys = regval & 0xFFFF;
		ent_cys = (regval >> 16) & 0xFFFF;

		/* Calculate active cycles for x percents duty */
		act_cys = DUTY_TO_ACT_CYCLES(duty, ent_cys);
		regval = (ent_cys << 16) | act_cys;
		PWM_WRITE4(sc, reg, regval);

		p->duty = duty;
	}
	PWM_UNLOCK(sc);

	return (0);
}

static void
h3_pwm_calc_freq(struct pwm *pwm)
{
	int div;

	div = h3_pwm_clkdiv[pwm->clkdiv];
	pwm->freq = PWM_CLOCK / div / pwm->period;
}

static device_method_t h3_pwm_methods[] = {
	DEVMETHOD(device_probe,		h3_pwm_probe),
	DEVMETHOD(device_attach,	h3_pwm_attach),

	DEVMETHOD_END
};

driver_t h3_pwm_driver = {
	"h3_pwm",
	h3_pwm_methods,
	sizeof(struct h3_pwm_softc),
};

static devclass_t h3_pwm_devclass;
DRIVER_MODULE(h3_pwm, simplebus, h3_pwm_driver, h3_pwm_devclass, 0, 0);
