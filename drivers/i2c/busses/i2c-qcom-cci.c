// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
// Copyright (c) 2017-2022 Linaro Limited.

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_opp.h>
#include <linux/spinlock.h>

#define CCI_HW_VERSION				0x0
#define CCI_HW_VERSION_MAJOR_MASK		GENMASK(31, 28)
#define CCI_HW_VERSION_MINOR_MASK		GENMASK(27, 16)
#define CCI_HW_VERSION_STEP_MASK		GENMASK(15, 0)
#define QCOM_CCI_VERSION(major, minor)		.hw_major = (major), .hw_minor = (minor)
#define CCI_RESET_CMD				0x004
#define CCI_RESET_CMD_MASK			0x0f73f3f7
#define CCI_RESET_CMD_M0_MASK			0x000003f1
#define CCI_RESET_CMD_M1_MASK			0x0003f001
#define CCI_QUEUE_START				0x008
#define CCI_HALT_REQ				0x034
#define CCI_HALT_REQ_I2C_M0_Q0Q1		BIT(0)
#define CCI_HALT_REQ_I2C_M1_Q0Q1		BIT(1)

#define CCI_I2C_Mm_SCL_CTL(m)			(0x100 + 0x100 * (m))
#define CCI_I2C_Mm_SDA_CTL_0(m)			(0x104 + 0x100 * (m))
#define CCI_I2C_Mm_SDA_CTL_1(m)			(0x108 + 0x100 * (m))
#define CCI_I2C_Mm_SDA_CTL_2(m)			(0x10c + 0x100 * (m))
#define CCI_I2C_Mm_MISC_CTL(m)			(0x110 + 0x100 * (m))

#define CCI_I2C_Mm_READ_DATA(m)			(0x118 + 0x100 * (m))
#define CCI_I2C_Mm_READ_BUF_LEVEL(m)		(0x11c + 0x100 * (m))
#define CCI_I2C_Mm_RD_THRESHOLD(m)		(0x120 + 0x100 * (m))
#define CCI_I2C_Mm_Qn_EXEC_WORD_CNT(m, n)	(0x300 + 0x200 * (m) + 0x100 * (n))
#define CCI_I2C_Mm_Qn_CUR_WORD_CNT(m, n)	(0x304 + 0x200 * (m) + 0x100 * (n))
#define CCI_I2C_Mm_Qn_CUR_CMD(m, n)		(0x308 + 0x200 * (m) + 0x100 * (n))
#define CCI_I2C_Mm_Qn_REPORT_STATUS(m, n)	(0x30c + 0x200 * (m) + 0x100 * (n))
#define CCI_I2C_Mm_Qn_LOAD_DATA(m, n)		(0x310 + 0x200 * (m) + 0x100 * (n))

#define CCI_IRQ_GLOBAL_CLEAR_CMD		0xc00
#define CCI_IRQ_MASK_0				0xc04
#define CCI_IRQ_MASK_0_I2C_M0_RD_DONE		BIT(0)
#define CCI_IRQ_MASK_0_I2C_M0_Q0_REPORT		BIT(4)
#define CCI_IRQ_MASK_0_I2C_M0_Q1_REPORT		BIT(8)
#define CCI_IRQ_MASK_0_I2C_M1_RD_DONE		BIT(12)
#define CCI_IRQ_MASK_0_I2C_M1_Q0_REPORT		BIT(16)
#define CCI_IRQ_MASK_0_I2C_M1_Q1_REPORT		BIT(20)
#define CCI_IRQ_MASK_0_RST_DONE_ACK		BIT(24)
#define CCI_IRQ_MASK_0_I2C_M0_Q0Q1_HALT_ACK	BIT(25)
#define CCI_IRQ_MASK_0_I2C_M1_Q0Q1_HALT_ACK	BIT(26)
#define CCI_IRQ_MASK_0_I2C_M0_ERROR		0x18000ee6
#define CCI_IRQ_MASK_0_I2C_M1_ERROR		0x60ee6000
#define CCI_IRQ_CLEAR_0				0xc08
#define CCI_IRQ_STATUS_0			0xc0c
#define CCI_IRQ_STATUS_0_I2C_M0_RD_DONE		BIT(0)
#define CCI_IRQ_STATUS_0_I2C_M0_RD_UNDERFLOW	BIT(1)
#define CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT	BIT(4)
#define CCI_IRQ_STATUS_0_I2C_M0_Q0_UNDERFLOW	BIT(5)
#define CCI_IRQ_STATUS_0_I2C_M0_Q0_OVERFLOW	BIT(6)
#define CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT	BIT(8)
#define CCI_IRQ_STATUS_0_I2C_M0_Q1_UNDERFLOW	BIT(9)
#define CCI_IRQ_STATUS_0_I2C_M0_Q1_OVERFLOW	BIT(10)
#define CCI_IRQ_STATUS_0_I2C_M1_RD_DONE		BIT(12)
#define CCI_IRQ_STATUS_0_I2C_M1_RD_UNDERFLOW	BIT(13)
#define CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT	BIT(16)
#define CCI_IRQ_STATUS_0_I2C_M1_Q0_UNDERFLOW	BIT(17)
#define CCI_IRQ_STATUS_0_I2C_M1_Q0_OVERFLOW	BIT(18)
#define CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT	BIT(20)
#define CCI_IRQ_STATUS_0_I2C_M1_Q1_UNDERFLOW	BIT(21)
#define CCI_IRQ_STATUS_0_I2C_M1_Q1_OVERFLOW	BIT(22)
#define CCI_IRQ_STATUS_0_RST_DONE_ACK		BIT(24)
#define CCI_IRQ_STATUS_0_I2C_M0_Q0Q1_HALT_ACK	BIT(25)
#define CCI_IRQ_STATUS_0_I2C_M1_Q0Q1_HALT_ACK	BIT(26)
#define CCI_IRQ_STATUS_0_I2C_M0_Q0_NACK_ERR	BIT(27)
#define CCI_IRQ_STATUS_0_I2C_M0_Q1_NACK_ERR	BIT(28)
#define CCI_IRQ_STATUS_0_I2C_M1_Q0_NACK_ERR	BIT(29)
#define CCI_IRQ_STATUS_0_I2C_M1_Q1_NACK_ERR	BIT(30)
#define CCI_IRQ_STATUS_0_I2C_M0_ERROR		0x18000ee6
#define CCI_IRQ_STATUS_0_I2C_M1_ERROR		0x60ee6000

#define CCI_IRQ_MASK_1				0xc10
#define CCI_IRQ_MASK_1_I2C_M0_RD_THRESHOLD	BIT(16)
#define CCI_IRQ_MASK_1_I2C_M0_RD_PAUSE		BIT(17)
#define CCI_IRQ_MASK_1_I2C_M1_RD_THRESHOLD	BIT(20)
#define CCI_IRQ_MASK_1_I2C_M1_RD_PAUSE		BIT(21)
#define CCI_IRQ_CLEAR_1				0xc14
#define CCI_IRQ_STATUS_1			0xc18
#define CCI_IRQ_STATUS_1_I2C_M0_RD_THRESHOLD	BIT(16)
#define CCI_IRQ_STATUS_1_I2C_M0_RD_PAUSE	BIT(17)
#define CCI_IRQ_STATUS_1_I2C_M1_RD_THRESHOLD	BIT(20)
#define CCI_IRQ_STATUS_1_I2C_M1_RD_PAUSE	BIT(21)

#define CCI_TIMEOUT	(msecs_to_jiffies(100))
#define NUM_MASTERS	2
#define NUM_QUEUES	2

#define CCI_I2C_SET_PARAM	1
#define CCI_I2C_LOCK		6
#define CCI_I2C_UNLOCK		7
#define CCI_I2C_REPORT		8
#define CCI_I2C_WRITE		9
#define CCI_I2C_READ		10

#define CCI_I2C_REPORT_IRQ_EN	BIT(8)

#define CCI_I2C_WRITE_WC	3
#define CCI_I2C_WRITE_MAX_LEN	(CCI_I2C_WRITE_WC * sizeof(u32) - 1)
#define CCI_I2C_WRITE_NWORDS(len) DIV_ROUND_UP((len) + 1, sizeof(u32))
#define CCI_I2C_WRITE_BURST	0xf
#define CCI_I2C_BURST_OVERHEAD	4 /* LOCK + SET_PARAM + UNLOCK + REPORT */

/* Largest write that can be bursted through a queue of @qsize words */
#define CCI_I2C_BURST_MAX_LEN(qsize)					\
	(((qsize) - CCI_I2C_BURST_OVERHEAD) / CCI_I2C_WRITE_WC *	\
	 CCI_I2C_WRITE_MAX_LEN)

/* The first and last bytes are reserved - not payload */
#define CCI_I2C_READ_MAX_LEN(fsize) ((fsize) - 2)

/* Unlimited read via threshold-irq support */
#define CCI_I2C_READ_BURST_NOLIMIT 0

enum {
	I2C_MODE_STANDARD,
	I2C_MODE_FAST,
	I2C_MODE_FAST_PLUS,
	NUM_I2C_MODES,
};

enum {
	CCI_CLK_RATE_19_2MHZ,
	CCI_CLK_RATE_37_5MHZ,
	NUM_CCI_CLK_RATES,
};

enum cci_i2c_queue_t {
	QUEUE_0,
	QUEUE_1
};

struct hw_params {
	u16 thigh; /* HIGH period of the SCL clock in clock ticks */
	u16 tlow; /* LOW period of the SCL clock */
	u16 tsu_sto; /* set-up time for STOP condition */
	u16 tsu_sta; /* set-up time for a repeated START condition */
	u16 thd_dat; /* data hold time */
	u16 thd_sta; /* hold time (repeated) START condition */
	u16 tbuf; /* bus free time between a STOP and START condition */
	u8 scl_stretch_en;
	u16 trdhld;
	u16 tsp; /* pulse width of spikes suppressed by the input filter */
};

struct cci;

struct cci_master {
	struct i2c_adapter adap;
	u16 master;
	u8 mode;
	int status;
	struct completion irq_complete;
	struct cci *cci;
};

struct cci_data {
	u16 hw_major;
	u16 hw_minor;
	struct i2c_adapter_quirks quirks;
	u16 queue_size[NUM_QUEUES];
	/* Highest I2C mode supported by this variant. */
	u8 max_mode;
};

struct cci {
	struct device *dev;
	void __iomem *base;
	unsigned int irq;
	unsigned int num_masters;
	const struct cci_data *data;
	struct clk_bulk_data *clocks;
	int nclocks;
	struct clk *cci_clk;
	struct cci_master master[NUM_MASTERS];
	/* Protects read-modify-write access to CCI_IRQ_MASK_1 */
	spinlock_t threshold_lock;
};

static bool cci_has_threshold_irq(const struct cci *cci)
{
	return cci->data->quirks.max_read_len == CCI_I2C_READ_BURST_NOLIMIT;
}

static void cci_set_rd_threshold_irq(struct cci *cci, u16 master, bool enable)
{
	u32 bits, val;

	if (!cci_has_threshold_irq(cci))
		return;

	guard(spinlock_irqsave)(&cci->threshold_lock);

	bits = master ? CCI_IRQ_MASK_1_I2C_M1_RD_THRESHOLD :
			CCI_IRQ_MASK_1_I2C_M0_RD_THRESHOLD;

	writel(bits, cci->base + CCI_IRQ_CLEAR_1);

	val = readl(cci->base + CCI_IRQ_MASK_1);
	if (enable)
		val |= bits;
	else
		val &= ~bits;

	writel(val, cci->base + CCI_IRQ_MASK_1);
}

static irqreturn_t cci_isr(int irq, void *dev)
{
	struct cci *cci = dev;
	u32 val, val1 = 0, reset = 0;
	int ret = IRQ_NONE;

	val = readl(cci->base + CCI_IRQ_STATUS_0);
	writel(val, cci->base + CCI_IRQ_CLEAR_0);

	if (cci_has_threshold_irq(cci)) {
		val1 = readl(cci->base + CCI_IRQ_STATUS_1);
		writel(val1, cci->base + CCI_IRQ_CLEAR_1);
	}

	writel(0x1, cci->base + CCI_IRQ_GLOBAL_CLEAR_CMD);

	if (val & CCI_IRQ_STATUS_0_RST_DONE_ACK) {
		complete(&cci->master[0].irq_complete);
		if (cci->master[1].master)
			complete(&cci->master[1].irq_complete);
		ret = IRQ_HANDLED;
	}

	if (val & CCI_IRQ_STATUS_0_I2C_M0_RD_DONE ||
			val & CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT ||
			val & CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT) {
		cci->master[0].status = 0;
		complete(&cci->master[0].irq_complete);
		ret = IRQ_HANDLED;
	}

	if (val & CCI_IRQ_STATUS_0_I2C_M1_RD_DONE ||
			val & CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT ||
			val & CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT) {
		cci->master[1].status = 0;
		complete(&cci->master[1].irq_complete);
		ret = IRQ_HANDLED;
	}

	if (val1 & CCI_IRQ_STATUS_1_I2C_M0_RD_THRESHOLD) {
		cci_set_rd_threshold_irq(cci, 0, false);
		complete(&cci->master[0].irq_complete);
		ret = IRQ_HANDLED;
	}

	if (val1 & CCI_IRQ_STATUS_1_I2C_M1_RD_THRESHOLD) {
		cci_set_rd_threshold_irq(cci, 1, false);
		complete(&cci->master[1].irq_complete);
		ret = IRQ_HANDLED;
	}

	if (val1 & CCI_IRQ_STATUS_1_I2C_M0_RD_PAUSE) {
		dev_warn_ratelimited(cci->dev, "master 0 read paused (fifo full)\n");
		ret = IRQ_HANDLED;
	}

	if (val1 & CCI_IRQ_STATUS_1_I2C_M1_RD_PAUSE) {
		dev_warn_ratelimited(cci->dev, "master 1 read paused (fifo full)\n");
		ret = IRQ_HANDLED;
	}

	if (unlikely(val & CCI_IRQ_STATUS_0_I2C_M0_Q0Q1_HALT_ACK)) {
		reset |= CCI_RESET_CMD_M0_MASK;
		ret = IRQ_HANDLED;
	}

	if (unlikely(val & CCI_IRQ_STATUS_0_I2C_M1_Q0Q1_HALT_ACK)) {
		reset |= CCI_RESET_CMD_M1_MASK;
		ret = IRQ_HANDLED;
	}

	if (unlikely(reset))
		writel(reset, cci->base + CCI_RESET_CMD);

	if (unlikely(val & CCI_IRQ_STATUS_0_I2C_M0_ERROR)) {
		cci->master[0].status = -EIO;

		if (val & CCI_IRQ_STATUS_0_I2C_M0_Q0_NACK_ERR ||
			val & CCI_IRQ_STATUS_0_I2C_M0_Q1_NACK_ERR)
			cci->master[0].status = -ENXIO;

		if (val & CCI_IRQ_STATUS_0_I2C_M0_Q0_OVERFLOW)
			dev_warn_ratelimited(cci->dev, "master 0 queue 0 overflow\n");
		if (val & CCI_IRQ_STATUS_0_I2C_M0_Q0_UNDERFLOW)
			dev_warn_ratelimited(cci->dev, "master 0 queue 0 underflow\n");
		if (val & CCI_IRQ_STATUS_0_I2C_M0_Q1_OVERFLOW)
			dev_warn_ratelimited(cci->dev, "master 0 queue 1 overflow\n");
		if (val & CCI_IRQ_STATUS_0_I2C_M0_Q1_UNDERFLOW)
			dev_warn_ratelimited(cci->dev, "master 0 queue 1 underflow\n");
		if (val & CCI_IRQ_STATUS_0_I2C_M0_RD_UNDERFLOW)
			dev_warn_ratelimited(cci->dev, "master 0 FIFO read underflow\n");

		writel(CCI_HALT_REQ_I2C_M0_Q0Q1, cci->base + CCI_HALT_REQ);
		ret = IRQ_HANDLED;
	}

	if (unlikely(val & CCI_IRQ_STATUS_0_I2C_M1_ERROR)) {
		cci->master[1].status = -EIO;

		if (val & CCI_IRQ_STATUS_0_I2C_M1_Q0_NACK_ERR ||
			val & CCI_IRQ_STATUS_0_I2C_M1_Q1_NACK_ERR)
			cci->master[1].status = -ENXIO;

		if (val & CCI_IRQ_STATUS_0_I2C_M1_Q0_OVERFLOW)
			dev_warn_ratelimited(cci->dev, "master 1 queue 0 overflow\n");
		if (val & CCI_IRQ_STATUS_0_I2C_M1_Q0_UNDERFLOW)
			dev_warn_ratelimited(cci->dev, "master 1 queue 0 underflow\n");
		if (val & CCI_IRQ_STATUS_0_I2C_M1_Q1_OVERFLOW)
			dev_warn_ratelimited(cci->dev, "master 1 queue 1 overflow\n");
		if (val & CCI_IRQ_STATUS_0_I2C_M1_Q1_UNDERFLOW)
			dev_warn_ratelimited(cci->dev, "master 1 queue 1 underflow\n");
		if (val & CCI_IRQ_STATUS_0_I2C_M1_RD_UNDERFLOW)
			dev_warn_ratelimited(cci->dev, "master 1 FIFO read underflow\n");

		writel(CCI_HALT_REQ_I2C_M1_Q0Q1, cci->base + CCI_HALT_REQ);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int cci_halt(struct cci *cci, u8 master_num)
{
	struct cci_master *master;
	u32 val;

	if (master_num >= cci->num_masters) {
		dev_err(cci->dev, "Unsupported master idx (%u)\n", master_num);
		return -EINVAL;
	}

	val = BIT(master_num);
	master = &cci->master[master_num];

	reinit_completion(&master->irq_complete);
	writel(val, cci->base + CCI_HALT_REQ);

	if (!wait_for_completion_timeout(&master->irq_complete, CCI_TIMEOUT)) {
		dev_err(cci->dev, "CCI halt timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static const unsigned long cci_clk_rates[NUM_CCI_CLK_RATES] = {
	[CCI_CLK_RATE_19_2MHZ] = 19200000,
	[CCI_CLK_RATE_37_5MHZ] = 37500000,
};

static int cci_clk_rate_idx(unsigned long rate)
{
	int i;

	for (i = 0; i < NUM_CCI_CLK_RATES; i++)
		if (cci_clk_rates[i] == rate)
			return i;

	return -EINVAL;
}

static const struct hw_params cci_hw_params[NUM_CCI_CLK_RATES][NUM_I2C_MODES] = {
	[CCI_CLK_RATE_19_2MHZ][I2C_MODE_STANDARD] = {
		.thigh = 78,
		.tlow = 114,
		.tsu_sto = 28,
		.tsu_sta = 28,
		.thd_dat = 10,
		.thd_sta = 77,
		.tbuf = 118,
		.scl_stretch_en = 0,
		.trdhld = 6,
		.tsp = 1
	},
	[CCI_CLK_RATE_19_2MHZ][I2C_MODE_FAST] = {
		.thigh = 20,
		.tlow = 28,
		.tsu_sto = 21,
		.tsu_sta = 21,
		.thd_dat = 13,
		.thd_sta = 18,
		.tbuf = 32,
		.scl_stretch_en = 0,
		.trdhld = 6,
		.tsp = 3
	},
	[CCI_CLK_RATE_37_5MHZ][I2C_MODE_STANDARD] = {
		.thigh = 201,
		.tlow = 174,
		.tsu_sto = 204,
		.tsu_sta = 231,
		.thd_dat = 22,
		.thd_sta = 162,
		.tbuf = 227,
		.scl_stretch_en = 0,
		.trdhld = 6,
		.tsp = 3
	},
	[CCI_CLK_RATE_37_5MHZ][I2C_MODE_FAST] = {
		.thigh = 38,
		.tlow = 56,
		.tsu_sto = 40,
		.tsu_sta = 40,
		.thd_dat = 22,
		.thd_sta = 35,
		.tbuf = 62,
		.scl_stretch_en = 0,
		.trdhld = 6,
		.tsp = 3
	},
	[CCI_CLK_RATE_37_5MHZ][I2C_MODE_FAST_PLUS] = {
		.thigh = 16,
		.tlow = 22,
		.tsu_sto = 17,
		.tsu_sta = 18,
		.thd_dat = 16,
		.thd_sta = 15,
		.tbuf = 24,
		.scl_stretch_en = 0,
		.trdhld = 3,
		.tsp = 3
	},
};

static const struct hw_params *cci_get_hw_params(struct cci *cci, int mode)
{
	unsigned long rate = clk_get_rate(cci->cci_clk);
	int ri = cci_clk_rate_idx(rate);

	if (ri >= 0 && mode <= cci->data->max_mode)
		return &cci_hw_params[ri][mode];

	return NULL;
}

static int cci_init(struct cci *cci)
{
	u32 val = CCI_IRQ_MASK_0_I2C_M0_RD_DONE |
			CCI_IRQ_MASK_0_I2C_M0_Q0_REPORT |
			CCI_IRQ_MASK_0_I2C_M0_Q1_REPORT |
			CCI_IRQ_MASK_0_I2C_M1_RD_DONE |
			CCI_IRQ_MASK_0_I2C_M1_Q0_REPORT |
			CCI_IRQ_MASK_0_I2C_M1_Q1_REPORT |
			CCI_IRQ_MASK_0_RST_DONE_ACK |
			CCI_IRQ_MASK_0_I2C_M0_Q0Q1_HALT_ACK |
			CCI_IRQ_MASK_0_I2C_M1_Q0Q1_HALT_ACK |
			CCI_IRQ_MASK_0_I2C_M0_ERROR |
			CCI_IRQ_MASK_0_I2C_M1_ERROR;
	int i;

	writel(val, cci->base + CCI_IRQ_MASK_0);

	val = 0;

	if (cci_has_threshold_irq(cci))
		val |= CCI_IRQ_MASK_1_I2C_M0_RD_PAUSE | CCI_IRQ_MASK_1_I2C_M1_RD_PAUSE;

	if (val)
		writel(val, cci->base + CCI_IRQ_MASK_1);

	for (i = 0; i < cci->num_masters; i++) {
		int mode = cci->master[i].mode;
		const struct hw_params *hw;

		if (!cci->master[i].cci)
			continue;

		hw = cci_get_hw_params(cci, mode);
		if (!hw) {
			dev_err(cci->dev, "no timing for mode %d at CCI clock %lu Hz\n",
				mode, clk_get_rate(cci->cci_clk));
			return -EOPNOTSUPP;
		}

		val = hw->thigh << 16 | hw->tlow;
		writel(val, cci->base + CCI_I2C_Mm_SCL_CTL(i));

		val = hw->tsu_sto << 16 | hw->tsu_sta;
		writel(val, cci->base + CCI_I2C_Mm_SDA_CTL_0(i));

		val = hw->thd_dat << 16 | hw->thd_sta;
		writel(val, cci->base + CCI_I2C_Mm_SDA_CTL_1(i));

		val = hw->tbuf;
		writel(val, cci->base + CCI_I2C_Mm_SDA_CTL_2(i));

		val = hw->scl_stretch_en << 8 | hw->trdhld << 4 | hw->tsp;
		writel(val, cci->base + CCI_I2C_Mm_MISC_CTL(i));

		if (cci_has_threshold_irq(cci)) {
			/* Hardware supporting the RD_THRESHOLD interrupt provides a minimum FIFO
			 * depth of 64 words, with the threshold reset to the full FIFO size by
			 * default. Configure the threshold to half the FIFO depth to allow burst
			 * reads while maintaining sufficient headroom to drain the FIFO before it
			 * fills up.
			 */
			val = readl(cci->base + CCI_I2C_Mm_RD_THRESHOLD(i));
			val = max_t(u32, val, 64);
			writel(val / 2, cci->base + CCI_I2C_Mm_RD_THRESHOLD(i));
		}
	}

	return 0;
}

static int cci_reset(struct cci *cci)
{
	/*
	 * we reset the whole controller, here and for implicity use
	 * master[0].xxx for waiting on it.
	 */
	reinit_completion(&cci->master[0].irq_complete);
	writel(CCI_RESET_CMD_MASK, cci->base + CCI_RESET_CMD);

	if (!wait_for_completion_timeout(&cci->master[0].irq_complete,
					 CCI_TIMEOUT)) {
		dev_err(cci->dev, "CCI reset timeout\n");
		return -ETIMEDOUT;
	}

	return cci_init(cci);
}

static int cci_run_queue(struct cci *cci, u8 master, u8 queue)
{
	u32 val;

	val = readl(cci->base + CCI_I2C_Mm_Qn_CUR_WORD_CNT(master, queue));
	writel(val, cci->base + CCI_I2C_Mm_Qn_EXEC_WORD_CNT(master, queue));

	reinit_completion(&cci->master[master].irq_complete);
	val = BIT(master * 2 + queue);
	writel(val, cci->base + CCI_QUEUE_START);

	if (!wait_for_completion_timeout(&cci->master[master].irq_complete,
					 CCI_TIMEOUT)) {
		dev_err(cci->dev, "master %d queue %d timeout\n",
			master, queue);
		cci_reset(cci);
		return -ETIMEDOUT;
	}

	return cci->master[master].status;
}

static int cci_validate_queue(struct cci *cci, u8 master, u8 queue)
{
	u32 val;

	val = readl(cci->base + CCI_I2C_Mm_Qn_CUR_WORD_CNT(master, queue));
	if (val == cci->data->queue_size[queue])
		return -EINVAL;

	if (!val)
		return 0;

	val = CCI_I2C_REPORT | CCI_I2C_REPORT_IRQ_EN;
	writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));

	return cci_run_queue(cci, master, queue);
}

/*
 * @status describes a single transfer, but only the REPORT and RD_DONE
 * interrupts clear it. A transfer woken by a threshold interrupt would
 * otherwise observe the error left behind by a previous one.
 */
static void cci_clear_status(struct cci *cci, u16 master)
{
	cci->master[master].status = 0;
}

static int cci_i2c_read(struct cci *cci, u16 master,
			u16 addr, u8 *buf, u16 len)
{
	u32 val, words_read;
	u8 queue = QUEUE_1;
	int i, index = 0, ret;

	cci_clear_status(cci, master);

	/*
	 * Call validate queue to make sure queue is empty before starting.
	 * This is to avoid overflow / underflow of queue.
	 */
	ret = cci_validate_queue(cci, master, queue);
	if (ret < 0)
		return ret;

	val = CCI_I2C_SET_PARAM | (addr & 0x7f) << 4;
	writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));

	val = CCI_I2C_READ | len << 4;
	writel(val, cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, queue));

	val = readl(cci->base + CCI_I2C_Mm_Qn_CUR_WORD_CNT(master, queue));
	writel(val, cci->base + CCI_I2C_Mm_Qn_EXEC_WORD_CNT(master, queue));

	reinit_completion(&cci->master[master].irq_complete);

	cci_set_rd_threshold_irq(cci, master, true);

	val = BIT(master * 2 + queue);
	writel(val, cci->base + CCI_QUEUE_START);

	/*
	 * Hardware supporting RD_THRESHOLD can handle reads larger than the FIFO size.
	 * The FIFO is drained each time an RD_THRESHOLD interrupt is received, until
	 * the final RD_DONE interrupt completes the transfer.
	 * On hardware lacking RD_THRESHOLD support, reads are limited to the FIFO
	 * size and only one completion (RD_DONE interrupt) is expected.
	 */
	while (index < len) {
		if (!wait_for_completion_timeout(&cci->master[master].irq_complete,
						 CCI_TIMEOUT)) {
			dev_err(cci->dev, "master %d queue %d read timeout\n",
				master, queue);
			cci_reset(cci);
			return -ETIMEDOUT;
		}

		if (cci->master[master].status)
			return cci->master[master].status;

		words_read = readl(cci->base + CCI_I2C_Mm_READ_BUF_LEVEL(master));

		while (words_read--) {
			val = readl_relaxed(cci->base + CCI_I2C_Mm_READ_DATA(master));
			/* The first byte of the read is the slave ID echo, not payload */
			for (i = (index == 0) ? 1 : 0; i < 4 && index < len; i++)
				buf[index++] = (val >> (i * 8)) & 0xff;
		}

		/*
		 * Do not reinit the completion here: RD_DONE may already have
		 * fired while the fifo was being drained, and dropping it would
		 * leave nothing left to wake the final round. The completion
		 * counts, and is reinitialised once before the queue is started.
		 */
		if (index < len)
			cci_set_rd_threshold_irq(cci, master, true);
	}

	return 0;
}

static void cci_i2c_load_write(void __iomem *loadq, u8 *buf, u16 len, bool burst)
{
	u32 cmd[CCI_I2C_WRITE_WC] = { };

	cmd[0] = CCI_I2C_WRITE | (burst ? CCI_I2C_WRITE_BURST : len) << 4;
	memcpy((u8 *)cmd + 1, buf, len);
	iowrite32_rep(loadq, cmd, CCI_I2C_WRITE_NWORDS(len));
}

static int cci_i2c_write(struct cci *cci, u16 master,
			 u16 addr, u8 *buf, u16 len)
{
	void __iomem *loadq = cci->base + CCI_I2C_Mm_Qn_LOAD_DATA(master, QUEUE_0);
	bool burst = len > CCI_I2C_WRITE_MAX_LEN;
	int ret;
	u32 val;

	dev_dbg(cci->dev, "master %d write: len=%u\n", master, len);

	/*
	 * A transfer longer than the limit advertised through the adapter
	 * quirks does not fit in the command queue, and is already rejected
	 * by the I2C core. Check it here as well, a transfer that overruns
	 * the queue would corrupt it.
	 */
	if (len > cci->data->quirks.max_write_len)
		return -EOPNOTSUPP;

	/*
	 * Call validate queue to make sure queue is empty before starting.
	 * This is to avoid overflow / underflow of queue.
	 */
	ret = cci_validate_queue(cci, master, QUEUE_0);
	if (ret < 0)
		return ret;

	if (burst)
		writel(CCI_I2C_LOCK, loadq);

	val = CCI_I2C_SET_PARAM | (addr & 0x7f) << 4;
	writel(val, loadq);

	do {
		u16 to_write = min_t(u16, len, CCI_I2C_WRITE_MAX_LEN);

		len -= to_write;
		cci_i2c_load_write(loadq, buf, to_write, len > 0);
		buf += to_write;
	} while (len);

	if (burst)
		writel(CCI_I2C_UNLOCK, loadq);

	val = CCI_I2C_REPORT | CCI_I2C_REPORT_IRQ_EN;
	writel(val, loadq);

	return cci_run_queue(cci, master, QUEUE_0);
}

static int cci_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct cci_master *cci_master = i2c_get_adapdata(adap);
	struct cci *cci = cci_master->cci;
	int i, ret;

	ret = pm_runtime_get_sync(cci->dev);
	if (ret < 0)
		goto err;

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = cci_i2c_read(cci, cci_master->master,
					   msgs[i].addr, msgs[i].buf,
					   msgs[i].len);
		else
			ret = cci_i2c_write(cci, cci_master->master,
					    msgs[i].addr, msgs[i].buf,
					    msgs[i].len);

		if (ret < 0)
			break;
	}

	if (!ret)
		ret = num;

err:
	pm_runtime_put_autosuspend(cci->dev);

	return ret;
}

static u32 cci_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm cci_algo = {
	.xfer = cci_xfer,
	.functionality = cci_func,
};

static int cci_enable_clocks(struct cci *cci)
{
	return clk_bulk_prepare_enable(cci->nclocks, cci->clocks);
}

static void cci_disable_clocks(struct cci *cci)
{
	clk_bulk_disable_unprepare(cci->nclocks, cci->clocks);
}

/*
 * The single CCI clock is shared by all masters, which may run in different
 * modes. Pick the lowest rate that has a valid timing set for every active
 * master's mode.
 */
static int cci_get_required_rate(struct cci *cci, unsigned long *rate)
{
	for (int ri = 0; ri < NUM_CCI_CLK_RATES; ri++) {
		bool supported = true;

		for (int i = 0; i < cci->num_masters; i++) {
			int mode = cci->master[i].mode;

			if (!cci->master[i].cci)
				continue;

			if (mode > cci->data->max_mode ||
			    !cci_hw_params[ri][mode].thigh) {
				supported = false;
				break;
			}
		}

		if (supported) {
			*rate = cci_clk_rates[ri];
			return 0;
		}
	}

	return -EOPNOTSUPP;
}

static int cci_set_core_rate(struct cci *cci)
{
	struct device *dev = cci->dev;
	unsigned long rate;
	int ret;

	ret = cci_get_required_rate(cci, &rate);
	if (ret) {
		dev_err(dev, "no CCI clock rate satisfies all masters\n");
		return ret;
	}

	ret = dev_pm_opp_set_rate(dev, rate);
	if (ret) {
		dev_err(dev, "CCI clock could not be set to %lu Hz\n", rate);
		return ret;
	}

	/*
	 * Sanity: The hw_params timings are only valid at the exact
	 * expected rate, verify what landed on the hardware.
	 */
	if (clk_get_rate(cci->cci_clk) != rate)
		dev_warn(dev, "CCI clock is not at expected %lu Hz\n", rate);

	return 0;
}

static int __maybe_unused cci_suspend_runtime(struct device *dev)
{
	struct cci *cci = dev_get_drvdata(dev);

	cci_disable_clocks(cci);
	return 0;
}

static int __maybe_unused cci_resume_runtime(struct device *dev)
{
	struct cci *cci = dev_get_drvdata(dev);
	int ret;

	ret = cci_enable_clocks(cci);
	if (ret)
		return ret;

	return cci_init(cci);
}

static const struct dev_pm_ops qcom_cci_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(cci_suspend_runtime, cci_resume_runtime, NULL)
};

/* Hardware capabilities, ascending by version major,minor */
static const struct cci_data cci_hw_catalog[] = {
	{
		QCOM_CCI_VERSION(1, 0),
		.max_mode = I2C_MODE_FAST,
		.queue_size = { 64, 16 },
		.quirks = {
			.max_write_len = 10,
			.max_read_len = CCI_I2C_READ_MAX_LEN(16),
		},
	},
	{
		QCOM_CCI_VERSION(1, 2),
		.max_mode = I2C_MODE_FAST_PLUS,
		.queue_size = { 64, 16 },
		.quirks = {
			.max_write_len = CCI_I2C_BURST_MAX_LEN(64),
			.max_read_len = CCI_I2C_READ_MAX_LEN(16),
		},
	},
	{
		QCOM_CCI_VERSION(1, 6),
		.max_mode = I2C_MODE_FAST_PLUS,
		.queue_size = { 128, 32 },
		.quirks = {
			.max_write_len = CCI_I2C_BURST_MAX_LEN(128),
			.max_read_len = CCI_I2C_READ_MAX_LEN(16),
		},
	},
	{
		QCOM_CCI_VERSION(1, 7),
		.max_mode = I2C_MODE_FAST_PLUS,
		.queue_size = { 128, 32 },
		.quirks = {
			.max_write_len = CCI_I2C_BURST_MAX_LEN(128),
			.max_read_len = CCI_I2C_READ_BURST_NOLIMIT,
		},
	},
	{
		QCOM_CCI_VERSION(1, 10),
		.max_mode = I2C_MODE_FAST_PLUS,
		.queue_size = { 1024, 32 },
		.quirks = {
			.max_write_len = CCI_I2C_BURST_MAX_LEN(1024),
			.max_read_len = CCI_I2C_READ_BURST_NOLIMIT,
		},
	},
};

static void cci_detect_hw_caps(struct cci *cci, u32 hw_version)
{
	u8 major = FIELD_GET(CCI_HW_VERSION_MAJOR_MASK, hw_version);
	u16 minor = FIELD_GET(CCI_HW_VERSION_MINOR_MASK, hw_version);
	u16 step = FIELD_GET(CCI_HW_VERSION_STEP_MASK, hw_version);
	const struct cci_data *data = &cci_hw_catalog[0];
	int i;

	for (i = 0; i < ARRAY_SIZE(cci_hw_catalog); i++) {
		if (cci_hw_catalog[i].hw_major > major ||
		    (cci_hw_catalog[i].hw_major == major && cci_hw_catalog[i].hw_minor > minor))
			break;
		data = &cci_hw_catalog[i];
	}

	dev_dbg(cci->dev, "CCI HW version %u.%u.%u, using caps tier %u.%u\n",
		major, minor, step, data->hw_major, data->hw_minor);

	cci->data = data;

	for (i = 0; i < cci->num_masters; i++) {
		if (cci->master[i].cci)
			cci->master[i].adap.quirks = &cci->data->quirks;
	}
}

static int cci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct resource *r;
	struct cci *cci;
	int ret, i;
	u32 val;

	cci = devm_kzalloc(dev, sizeof(*cci), GFP_KERNEL);
	if (!cci)
		return -ENOMEM;

	cci->dev = dev;
	platform_set_drvdata(pdev, cci);

	spin_lock_init(&cci->threshold_lock);

	for_each_available_child_of_node(dev->of_node, child) {
		struct cci_master *master;
		u32 idx;

		ret = of_property_read_u32(child, "reg", &idx);
		if (ret) {
			dev_err(dev, "%pOF invalid 'reg' property", child);
			continue;
		}

		if (idx >= NUM_MASTERS) {
			dev_err(dev, "%pOF invalid 'reg' value: %u (max is %u)",
				child, idx, NUM_MASTERS - 1);
			continue;
		}

		if (idx >= cci->num_masters)
			cci->num_masters = idx + 1;

		master = &cci->master[idx];
		master->adap.algo = &cci_algo;
		master->adap.dev.parent = dev;
		master->adap.dev.of_node = of_node_get(child);
		master->master = idx;
		master->cci = cci;

		i2c_set_adapdata(&master->adap, master);
		snprintf(master->adap.name, sizeof(master->adap.name), "Qualcomm-CCI");

		master->mode = I2C_MODE_STANDARD;
		ret = of_property_read_u32(child, "clock-frequency", &val);
		if (!ret) {
			if (val == I2C_MAX_FAST_MODE_FREQ)
				master->mode = I2C_MODE_FAST;
			else if (val == I2C_MAX_FAST_MODE_PLUS_FREQ)
				master->mode = I2C_MODE_FAST_PLUS;
		}

		init_completion(&master->irq_complete);
	}

	/* Memory */

	cci->base = devm_platform_get_and_ioremap_resource(pdev, 0, &r);
	if (IS_ERR(cci->base))
		return PTR_ERR(cci->base);

	/* Clocks */

	ret = devm_clk_bulk_get_all(dev, &cci->clocks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	else if (!ret)
		return dev_err_probe(dev, -EINVAL, "not enough clocks in DT\n");
	cci->nclocks = ret;

	cci->cci_clk = devm_clk_get(dev, "cci");
	if (IS_ERR(cci->cci_clk))
		return dev_err_probe(dev, PTR_ERR(cci->cci_clk),
				     "failed to get CCI clock\n");

	ret = devm_pm_opp_set_clkname(dev, "cci");
	if (ret)
		return dev_err_probe(dev, ret, "failed to set CCI OPP clk\n");

	/* OPP table is optional */
	ret = devm_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to add OPP table\n");

	ret = cci_enable_clocks(cci);
	if (ret < 0)
		return ret;

	/* Interrupt */

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto disable_clocks;
	cci->irq = ret;

	ret = devm_request_irq(dev, cci->irq, cci_isr, 0, dev_name(dev), cci);
	if (ret < 0) {
		dev_err(dev, "request_irq failed, ret: %d\n", ret);
		goto disable_clocks;
	}

	val = readl(cci->base + CCI_HW_VERSION);
	cci_detect_hw_caps(cci, val);

	ret = cci_set_core_rate(cci);
	if (ret)
		goto disable_clocks;

	ret = cci_reset(cci);
	if (ret < 0)
		goto disable_clocks;

	pm_runtime_set_autosuspend_delay(dev, MSEC_PER_SEC);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	for (i = 0; i < cci->num_masters; i++) {
		if (!cci->master[i].cci)
			continue;

		ret = i2c_add_adapter(&cci->master[i].adap);
		if (ret < 0) {
			of_node_put(cci->master[i].adap.dev.of_node);
			goto error_i2c;
		}
	}

	return 0;

error_i2c:
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);

	for (--i ; i >= 0; i--) {
		if (cci->master[i].cci) {
			i2c_del_adapter(&cci->master[i].adap);
			of_node_put(cci->master[i].adap.dev.of_node);
		}
	}
disable_clocks:
	cci_disable_clocks(cci);

	return ret;
}

static void cci_remove(struct platform_device *pdev)
{
	struct cci *cci = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < cci->num_masters; i++) {
		if (cci->master[i].cci) {
			i2c_del_adapter(&cci->master[i].adap);
			of_node_put(cci->master[i].adap.dev.of_node);
			cci_halt(cci, i);
		}
	}

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
}

static const struct of_device_id cci_dt_match[] = {
	{ .compatible = "qcom,msm8226-cci" },
	{ .compatible = "qcom,msm8953-cci" },
	{ .compatible = "qcom,msm8974-cci" },
	{ .compatible = "qcom,msm8996-cci" },

	/*
	 * Legacy compatibles kept for backwards compatibility.
	 * Do not add any new ones unless they introduce a new config
	 */
	{ .compatible = "qcom,msm8916-cci" },
	{ .compatible = "qcom,sdm845-cci" },
	{ .compatible = "qcom,sm8250-cci" },
	{ .compatible = "qcom,sm8450-cci" },
	{}
};
MODULE_DEVICE_TABLE(of, cci_dt_match);

static struct platform_driver qcom_cci_driver = {
	.probe  = cci_probe,
	.remove = cci_remove,
	.driver = {
		.name = "i2c-qcom-cci",
		.of_match_table = cci_dt_match,
		.pm = &qcom_cci_pm,
	},
};

module_platform_driver(qcom_cci_driver);

MODULE_DESCRIPTION("Qualcomm Camera Control Interface driver");
MODULE_AUTHOR("Todor Tomov <todor.tomov@linaro.org>");
MODULE_AUTHOR("Loic Poulain <loic.poulain@linaro.org>");
MODULE_LICENSE("GPL v2");
