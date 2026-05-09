// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx29x cameras.
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#define IMX29X_REG_MODE_SELECT		0x3000
#define IMX29X_MODE_STANDBY		0x01
#define IMX29X_MODE_STREAMING		0x00
#define IMX29X_STREAM_DELAY_US		25000
#define IMX29X_STREAM_DELAY_RANGE_US	1000
#define IMX29X_AUTOSUSPEND_DELAY_MS	1000

#define IMX29X_LINK_RATE		1728000000ULL
#define IMX29X_LINK_FREQ		(IMX29X_LINK_RATE / 2)
#define IMX29X_NUM_DATA_LANES	4
#define IMX29X_TIMING_CLOCK_HZ		72000000ULL

/* VMAX internal VBLANK*/
#define IMX29X_REG_VMAX		0x30A9
#define IMX29X_VMAX_MAX		0xfffff

/* HMAX internal HBLANK*/
#define IMX29X_REG_HMAX		0x30AC
#define IMX29X_HMAX_MAX		0xffff

#define IMX29X_REG_HCOUNT1     0x3084
#define IMX29X_REG_HCOUNT2     0x3086
#define IMX29X_REG_HOPBOUT_EN  0x3034
#define IMX29X_REG_HTRIMMING_EN 0x3035
#define IMX29X_REG_HTRIMMING_START 0x3036
#define IMX29X_REG_HTRIMMING_END 0x3038
#define IMX29X_REG_VWINPOS     0x30E0
#define IMX29X_REG_PSSLVS1 0x332C
#define IMX29X_REG_PSSLVS2 0x334A
#define IMX29X_REG_PSSLVS3 0x35B6
#define IMX29X_REG_PSSLVS4 0x35B8
#define IMX29X_REG_PSSLVS0 0x36BC
#define IMX29X_REG_OPB_SIZE_V  0x312F
#define IMX29X_REG_WRITE_VSIZE 0x3130
#define IMX29X_REG_Y_OUT_SIZE  0x3132

/* SHR internal */
#define IMX29X_REG_SHR		0x302C

/* Exposure control */
#define IMX29X_EXPOSURE_MIN			1
#define IMX29X_EXPOSURE_STEP		1
#define IMX29X_EXPOSURE_DEFAULT		1000
#define IMX29X_EXPOSURE_MAX		0xfffff

/* Analog gain control */
#define IMX29X_REG_ANALOG_GAIN		0x300A
#define IMX29X_ANA_GAIN_MIN		0
#define IMX29X_ANA_GAIN_MAX		1957
#define IMX29X_ANA_GAIN_STEP		1
#define IMX29X_ANA_GAIN_DEFAULT		0x0

/*
 * BLKLEVEL is an 8-bit register. In 12-bit mode each register step moves the
 * output black level by 4 digits, so the support package's 200-digit nominal
 * black level corresponds to a register value of 50.
 */
#define IMX29X_REG_BLKLEVEL		0x3042
#define IMX29X_BLKLEVEL_DEFAULT		50

#define IMX29X_REG_PLRD10		0x311F
#define IMX29X_REG_PLRD2		0x3122
#define IMX29X_REG_PLRD11		0x3123
#define IMX29X_REG_PLRD12		0x3124
#define IMX29X_REG_PLRD13		0x3125
#define IMX29X_REG_PLRD14		0x3127
#define IMX29X_REG_PLRD3		0x3129
#define IMX29X_REG_PLRD4		0x312A
#define IMX29X_REG_PLRD15		0x312D
#define IMX29X_REG_PLRD1_LSB		0x31E8
#define IMX29X_REG_PLRD1_MSB		0x31E9

#define IMX29X_REG_TEST_PATTERN_CTRL	0x303A
#define IMX29X_REG_TEST_PATTERN_SEL	0x303B
#define IMX29X_TEST_PATTERN_ENABLE_MIPI	0x11

/* Embedded metadata stream structure */
#define IMX29X_EMBEDDED_LINE_WIDTH 16384
#define IMX29X_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/*
 * Native image payload and active/effective pixel array. The sensor can
 * deliver OPB/dummy margin pixels around the active area, so the advertised
 * frame size may be larger than the active crop rectangle returned through
 * selection targets.
 */
#define IMX29X_NATIVE_WIDTH		8432U
#define IMX29X_NATIVE_HEIGHT		5648U
#define IMX29X_PIXEL_ARRAY_LEFT	0U
#define IMX29X_PIXEL_ARRAY_TOP		0U
#define IMX29X_PIXEL_ARRAY_WIDTH	8240U
#define IMX29X_PIXEL_ARRAY_HEIGHT	5628U

struct imx29x_reg {
	u16 address;
	u8 val;
};

struct imx29x_reg_list {
	unsigned int num_of_regs;
	const struct imx29x_reg *regs;
};

struct imx29x_plrd_setup {
	u32 xclk_freq;
	struct imx29x_reg regs[11];
};

/*
 * CSI-2 PLRD1..PLRD15 input-clock setup. The datasheets require these
 * registers to be programmed according to the external INCK frequency before
 * standby cancel. HMAX/VMAX timing remains expressed in 72 MHz converted
 * clocks after the matching row is selected.
 */
static const struct imx29x_plrd_setup imx29x_plrd_setups[] = {
	{
		.xclk_freq = 6000000,
		.regs = {
			{IMX29X_REG_PLRD1_LSB, 0x20},
			{IMX29X_REG_PLRD1_MSB, 0x01},
			{IMX29X_REG_PLRD2, 0x00},
			{IMX29X_REG_PLRD3, 0x90},
			{IMX29X_REG_PLRD4, 0x00},
			{IMX29X_REG_PLRD10, 0x00},
			{IMX29X_REG_PLRD11, 0x00},
			{IMX29X_REG_PLRD12, 0x00},
			{IMX29X_REG_PLRD13, 0x01},
			{IMX29X_REG_PLRD14, 0x02},
			{IMX29X_REG_PLRD15, 0x02},
		},
	},
	{
		.xclk_freq = 12000000,
		.regs = {
			{IMX29X_REG_PLRD1_LSB, 0x20},
			{IMX29X_REG_PLRD1_MSB, 0x01},
			{IMX29X_REG_PLRD2, 0x01},
			{IMX29X_REG_PLRD3, 0x90},
			{IMX29X_REG_PLRD4, 0x01},
			{IMX29X_REG_PLRD10, 0x00},
			{IMX29X_REG_PLRD11, 0x00},
			{IMX29X_REG_PLRD12, 0x00},
			{IMX29X_REG_PLRD13, 0x01},
			{IMX29X_REG_PLRD14, 0x02},
			{IMX29X_REG_PLRD15, 0x02},
		},
	},
	{
		.xclk_freq = 18000000,
		.regs = {
			{IMX29X_REG_PLRD1_LSB, 0xC0},
			{IMX29X_REG_PLRD1_MSB, 0x00},
			{IMX29X_REG_PLRD2, 0x01},
			{IMX29X_REG_PLRD3, 0x60},
			{IMX29X_REG_PLRD4, 0x01},
			{IMX29X_REG_PLRD10, 0x00},
			{IMX29X_REG_PLRD11, 0x00},
			{IMX29X_REG_PLRD12, 0x00},
			{IMX29X_REG_PLRD13, 0x01},
			{IMX29X_REG_PLRD14, 0x02},
			{IMX29X_REG_PLRD15, 0x02},
		},
	},
	{
		.xclk_freq = 24000000,
		.regs = {
			{IMX29X_REG_PLRD1_LSB, 0x20},
			{IMX29X_REG_PLRD1_MSB, 0x01},
			{IMX29X_REG_PLRD2, 0x02},
			{IMX29X_REG_PLRD3, 0x90},
			{IMX29X_REG_PLRD4, 0x02},
			{IMX29X_REG_PLRD10, 0x00},
			{IMX29X_REG_PLRD11, 0x00},
			{IMX29X_REG_PLRD12, 0x00},
			{IMX29X_REG_PLRD13, 0x01},
			{IMX29X_REG_PLRD14, 0x02},
			{IMX29X_REG_PLRD15, 0x02},
		},
	},
};

/* Mode : resolution and related config&values */
struct imx29x_mode {
	/*
	 * Delivered media-bus frame dimensions. These are payload dimensions
	 * and may include delivered OPB/dummy pixels, or be smaller than the
	 * native-coordinate crop for binned/scaled modes.
	 */
	unsigned int width;

	/* Delivered media-bus frame height. */
	unsigned int height;

	/* minimum H-timing */
	u64 min_hmax;

	/* minimum V-timing */
	u64 min_vmax;

	/* default H-timing */
	u64 default_hmax;

	/* default V-timing */
	u64 default_vmax;

	/* V-timing Scaling */
	u64 vmax_scale;

	/* minimum SHR */
	u64 min_shr;

	/* maximum SHR is (SVR + 1) * VMAX - max_shr_margin */
	u64 max_shr_margin;

	unsigned int integration_offset;

	/*
	 * Active/effective pixel-array crop in native sensor coordinates.
	 * Selection targets report this coordinate space, not payload size.
	 */
	struct v4l2_rect crop;

	/* Default register values */
	struct imx29x_reg_list reg_list;

	/* Explicit output-size registers for modes that deliver OPB/dummy data. */
	bool use_output_overrides;
	u8 opb_size_v;
	u16 write_vsize;
	u16 y_out_size;
};

static const struct imx29x_reg imx29x_startup_pre_regs[] = {
	{0x3033, 0x30},
	{0x303C, 0x01},
};

static const struct imx29x_reg imx29x_startup_post_plrd_regs[] = {
	{IMX29X_REG_MODE_SELECT, 0x12},
	{0x310B, 0x00},
};

static const struct imx29x_reg imx29x_standby_release_regs[] = {
	{0xFFFE, 0x0A},
	{IMX29X_REG_MODE_SELECT, 0x02},
	{0x35E5, 0x92},
	{0x35E5, 0x9A},
};

static const struct imx29x_reg imx29x_stream_on_regs[] = {
	{IMX29X_REG_MODE_SELECT, IMX29X_MODE_STREAMING},
	{0xFFFE, 0x0A},
	{0x3033, 0x20},
	{0x3017, 0xA8},
};

#include "imx29x_mode_tables.h"

static const struct imx29x_reg imx294_common_regs[] = {
	/* STANDBY = 0, STBLOGIC = 1h, STBMIPI = 0h, STBDV = 1h */
	{0x3000, 0x12},
	{0x310B, 0x00}, /* PLL release */
	{0x3047, 0x01}, /* PLSTMG11 */
	{0x304E, 0x0B}, /* PLSTMG12 */
	{0x304F, 0x24}, /* PLSTMG13 */
	{0x3062, 0x25}, /* PLSTMG14 */
	{0x3064, 0x78}, /* PLSTMG15 */
	{0x3065, 0x33}, /* PLSTMG16 */
	{0x3067, 0x71}, /* PLSTMG17 */
	{0x3088, 0x75}, /* PLSTMG18 */
	{0x308A, 0x09}, /* PLSTMG19 */
	{0x308B, 0x01}, /* PLSTMG19 */
	{0x308C, 0x61}, /* PLSTMG20 */
	{0x3146, 0x00}, /* PLSTMG10 */
	{0x3234, 0x32}, /* PLSTMG21 */
	{0x3235, 0x00}, /* PLSTMG21 */
	{0x3248, 0xBC}, /* PLSTMG22 */
	{0x3249, 0x00}, /* PLSTMG22 */
	{0x3250, 0xBC}, /* PLSTMG23 */
	{0x3251, 0x00}, /* PLSTMG23 */
	{0x3258, 0xBC}, /* PLSTMG24 */
	{0x3259, 0x00}, /* PLSTMG24 */
	{0x3260, 0xBC}, /* PLSTMG25 */
	{0x3261, 0x00}, /* PLSTMG25 */
	{0x3274, 0x13}, /* PLSTMG26 */
	{0x3275, 0x00}, /* PLSTMG26 */
	{0x3276, 0x1F}, /* PLSTMG27 */
	{0x3277, 0x00}, /* PLSTMG27 */
	{0x3278, 0x30}, /* PLSTMG28 */
	{0x3279, 0x00}, /* PLSTMG28 */
	{0x327C, 0x13}, /* PLSTMG29 */
	{0x327D, 0x00}, /* PLSTMG29 */
	{0x327E, 0x1F}, /* PLSTMG30 */
	{0x327F, 0x00}, /* PLSTMG30 */
	{0x3280, 0x30}, /* PLSTMG31 */
	{0x3281, 0x00}, /* PLSTMG31 */
	{0x3284, 0x13}, /* PLSTMG32 */
	{0x3285, 0x00}, /* PLSTMG32 */
	{0x3286, 0x1F}, /* PLSTMG33 */
	{0x3287, 0x00}, /* PLSTMG33 */
	{0x3288, 0x30}, /* PLSTMG34 */
	{0x3289, 0x00}, /* PLSTMG34 */
	{0x328C, 0x13}, /* PLSTMG35 */
	{0x328D, 0x00}, /* PLSTMG35 */
	{0x328E, 0x1F}, /* PLSTMG36 */
	{0x328F, 0x00}, /* PLSTMG36 */
	{0x3290, 0x30}, /* PLSTMG37 */
	{0x3291, 0x00}, /* PLSTMG37 */
	{0x32AE, 0x00}, /* PLSTMG38 */
	{0x32AF, 0x00}, /* PLSTMG39 */
	{0x32CA, 0x5A}, /* PLSTMG40 */
	{0x32CB, 0x00}, /* PLSTMG40 */
	{0x332F, 0x00}, /* PLSTMG41 */
	{0x334C, 0x01}, /* PLSTMG09 */
	{0x335A, 0x79}, /* PLSTMG43 */
	{0x335B, 0x00}, /* PLSTMG43 */
	{0x335E, 0x56}, /* PLSTMG44 */
	{0x335F, 0x00}, /* PLSTMG44 */
	{0x3360, 0x6A}, /* PLSTMG45 */
	{0x3361, 0x00}, /* PLSTMG45 */
	{0x336A, 0x56}, /* PLSTMG46 */
	{0x336B, 0x00}, /* PLSTMG46 */
	{0x33D6, 0x79}, /* PLSTMG47 */
	{0x33D7, 0x00}, /* PLSTMG47 */
	{0x340C, 0x6E}, /* PLSTMG48 */
	{0x340D, 0x00}, /* PLSTMG48 */
	{0x3448, 0x7E}, /* PLSTMG49 */
	{0x3449, 0x00}, /* PLSTMG49 */
	{0x348E, 0x6F}, /* PLSTMG50 */
	{0x348F, 0x00}, /* PLSTMG50 */
	{0x3492, 0x11}, /* PLSTMG51 */
	{0x34C4, 0x5A}, /* PLSTMG52 */
	{0x34C5, 0x00}, /* PLSTMG52 */
	{0x3506, 0x56}, /* PLSTMG53 */
	{0x3507, 0x00}, /* PLSTMG53 */
	{0x350C, 0x56}, /* PLSTMG54 */
	{0x350D, 0x00}, /* PLSTMG54 */
	{0x350E, 0x58}, /* PLSTMG55 */
	{0x350F, 0x00}, /* PLSTMG55 */
	{0x3549, 0x04}, /* PLSTMG56 */
	{0x355D, 0x03}, /* PLSTMG57 */
	{0x355E, 0x03}, /* PLSTMG58 */
	{0x3574, 0x56}, /* PLSTMG59 */
	{0x3575, 0x00}, /* PLSTMG59 */
	{0x3587, 0x01}, /* PLSTMG60 */
	{0x35D0, 0x5E}, /* PLSTMG61 */
	{0x35D1, 0x00}, /* PLSTMG61 */
	{0x35D4, 0x63}, /* PLSTMG62 */
	{0x35D5, 0x00}, /* PLSTMG62 */
	{0x366A, 0x1A}, /* PLSTMG63 */
	{0x366B, 0x16}, /* PLSTMG64 */
	{0x366C, 0x10}, /* PLSTMG65 */
	{0x366D, 0x09}, /* PLSTMG66 */
	{0x366E, 0x00}, /* PLSTMG67 */
	{0x366F, 0x00}, /* PLSTMG68 */
	{0x3670, 0x00}, /* PLSTMG69 */
	{0x3671, 0x00}, /* PLSTMG70 */
	{0x3676, 0x83}, /* PLSTMG73 */
	{0x3677, 0x03}, /* PLSTMG73 */
	{0x3678, 0x00}, /* PLSTMG74 */
	{0x3679, 0x04}, /* PLSTMG74 */
	{0x367A, 0x2C}, /* PLSTMG75 */
	{0x367B, 0x05}, /* PLSTMG75 */
	{0x367C, 0x00}, /* PLSTMG76 */
	{0x367D, 0x06}, /* PLSTMG76 */
	{0x367E, 0x00}, /* PLSTMG77 */
	{0x367F, 0x07}, /* PLSTMG77 */
	{0x3680, 0x4B}, /* PLSTMG78 */
	{0x3681, 0x07}, /* PLSTMG78 */
	{0x3690, 0x27}, /* PLSTMG79 */
	{0x3691, 0x00}, /* PLSTMG79 */
	{0x3692, 0x65}, /* PLSTMG80 */
	{0x3693, 0x00}, /* PLSTMG80 */
	{0x3694, 0x4F}, /* PLSTMG81 */
	{0x3695, 0x00}, /* PLSTMG81 */
	{0x3696, 0xA1}, /* PLSTMG82 */
	{0x3697, 0x00}, /* PLSTMG82 */
	{0x382B, 0x68}, /* PLSTMG83 */
	{0x3C00, 0x01}, /* PLSTMG84 */
	{0x3C01, 0x01}, /* PLSTMG85 */
	{0x3686, 0x00}, /* PLSTMG101 */
	{0x3687, 0x00}, /* PLSTMG101 */
	{0x36BE, 0x01}, /* PLSTMG102 */
	{0x36BF, 0x00}, /* PLSTMG102 */
	{0x36C0, 0x01}, /* PLSTMG103 */
	{0x36C1, 0x00}, /* PLSTMG103 */
	{0x36C2, 0x01}, /* PLSTMG104 */
	{0x36C3, 0x00}, /* PLSTMG104 */
	{0x36C4, 0x01}, /* PLSTMG105 */
	{0x36C5, 0x01}, /* PLSTMG106 */
	{0x36C6, 0x01}, /* PLSTMG107 */
	{0x3134, 0xAF}, /* tclkpost */
	{0x3135, 0x00},
	{0x3136, 0xC7}, /* thszero */
	{0x3137, 0x00},
	{0x3138, 0x7F}, /* thsprepare */
	{0x3139, 0x00},
	{0x313A, 0x6F}, /* tclktrail */
	{0x313B, 0x00},
	{0x313C, 0x6F}, /* thstrail */
	{0x313D, 0x00},
	{0x313E, 0xCF}, /* tclkzero */
	{0x313F, 0x01},
	{0x3140, 0x77}, /* tclkprepare */
	{0x3141, 0x00},
	{0x3142, 0x5F}, /* tlpx */
	{0x3143, 0x00},

	{0x3004, 0x1A}, /* MDSEL1 */
	{0x3005, 0x06}, /* MDSEL2 */
	{0x3006, 0x00}, /* MDSEL3 */
	{0x3007, 0xA0}, /* MDSEL4 */
	{0x3019, 0x00}, /* MDVREV */
	{0x3030, 0x77}, /* MDSEL5 */
	{0x3034, 0x00}, /* HOPBOUT_EN */
	{0x3035, 0x01}, /* HTRIMMING_EN */
	{0x3036, 0x30}, /* HTRIMMING_START */
	{0x3037, 0x00}, /* HTRIMMING_START */
	{0x3038, 0x60}, /* HTRIMMING_END */
	{0x3039, 0x10}, /* HTRIMMING_END */
	{0x3068, 0x1A}, /* MDSEL15 */
	{0x3069, 0x00}, /* MDSEL15 */
	{0x3080, 0x00}, /* MDSEL6 */
	{0x3081, 0x01}, /* MDSEL7 */
	{0x30A8, 0x02}, /* MDSEL8 */
	{0x30E2, 0x00}, /* VCUTMODE */
	{0x312F, 0x08}, /* OPB_SIZE_V */
	{0x3130, 0x88}, /* WRITE_VSIZE */
	{0x3131, 0x08}, /* WRITE_VSIZE */
	{0x3132, 0x80}, /* OUT_SIZE */
	{0x3133, 0x08}, /* Y_OUT_SIZE */
	{0x357F, 0x0C}, /* MDSEL11 */
	{0x3580, 0x0A}, /* MDSEL12 */
	{0x3581, 0x08}, /* MDSEL13 */
	{0x3583, 0x72}, /* MDSEL14 */
	{0x3600, 0x90}, /* MDSEL16 */
	{0x3601, 0x00}, /* MDSEL16 */
	{0x3846, 0x00}, /* MDSEL9 */
	{0x3847, 0x00}, /* MDSEL9 */
	{0x384A, 0x00}, /* MDSEL10 */
	{0x384B, 0x00}, /* MDSEL10 */
	/* SVR = 0 */
	{0x300E, 0x00},
	{0x300F, 0x00},

	/* SHR = 100 */
	{0x302C, 0x10},
	{0x302D, 0x00},

	/* VMAX = 5000 */
	{0x30A9, 0x88},
	{0x30AA, 0x13},
	{0x30AB, 0x00},

	/* HMAX = 1200 */
	{0x30AC, 0xB0},
	{0x30AD, 0x04},
	/* HCOUNT1 Set the same value as HMAX */
	{0x3084, 0xB0},
	{0x3085, 0x04},
	/* HCOUNT2 Set the same value as HMAX */
	{0x3086, 0xB0},
	{0x3087, 0x04},

	/* PSSLVS1 = VBLK = VMAX * (SVR + 1) - minimum VMAX */
	{0x332C, 0x00},
	{0x332D, 0x00},
	{0x334A, 0x00}, /* PSSLVS2 = VBLK */
	{0x334B, 0x00},
	{0x35B6, 0x00}, /* PSSLVS3 = VBLK */
	{0x35B7, 0x00},
	{0x35B8, 0x00}, /* PSSLVS4 = VBLK - 5 */
	{0x35B9, 0x00},
	{0x36BC, 0x00}, /* PSSLVS0 = VBLK */
	{0x36BD, 0x00},
};

#include "imx294_quad_mode_tables.h"

/* 3704 x 2778 readout mode 0 - 14bit */
static const struct imx29x_reg imx294_mode_00_14bit_regs[] = {
	{0x3004, 0x00}, /* MDSEL1 */
	{0x3005, 0x0B}, /* MDSEL2 */
	{0x3006, 0x02}, /* MDSEL3 */
	{0x3007, 0xA0}, /* MDSEL4 */
	{0x3019, 0x00}, /* MDVREV */
	{0x3030, 0x77}, /* MDSEL5 */
	{0x3034, 0x00}, /* HOPBOUT_EN */
	{0x3035, 0x01}, /* HTRIMMING_EN */
	{0x3036, 0x30}, /* HTRIMMING_START */
	{0x3037, 0x00}, /* HTRIMMING_START */
	{0x3038, 0x00}, /* HTRIMMING_END */
	{0x3039, 0x0F}, /* HTRIMMING_END */
	{0x3068, 0x44}, /* MDSEL15 */
	{0x3069, 0x00}, /* MDSEL15 */
	{0x3080, 0x00}, /* MDSEL6 */
	{0x3081, 0x01}, /* MDSEL7 */
	{0x30A8, 0x03}, /* MDSEL8 */
	{0x30E2, 0x00}, /* VCUTMODE */
	{0x312F, 0x10}, /* OPB_SIZE_V */
	{0x3130, 0x18}, /* WRITE_VSIZE */
	{0x3131, 0x0B}, /* WRITE_VSIZE */
	{0x3132, 0x08}, /* OUT_SIZE */
	{0x3133, 0x0B}, /* Y_OUT_SIZE */
	{0x357F, 0x0A}, /* MDSEL11 */
	{0x3580, 0x09}, /* MDSEL12 */
	{0x3581, 0x07}, /* MDSEL13 */
	{0x3583, 0x51}, /* MDSEL14 */
	{0x3600, 0x90}, /* MDSEL16 */
	{0x3601, 0x00}, /* MDSEL16 */
	{0x3846, 0x00}, /* MDSEL9 */
	{0x3847, 0x00}, /* MDSEL9 */
	{0x384A, 0x00}, /* MDSEL10 */
	{0x384B, 0x00}, /* MDSEL10 */
};

/* 3704 x 2778 readout mode 0 - 12bit */
static const struct imx29x_reg imx294_mode_00_regs[] = {
	{0x3004, 0x00}, /* MDSEL1 */
	{0x3005, 0x06}, /* MDSEL2 */
	{0x3006, 0x02}, /* MDSEL3 */
	{0x3007, 0xA0}, /* MDSEL4 */
	{0x3019, 0x00}, /* MDVREV */
	{0x3030, 0x77}, /* MDSEL5 */
	{0x3034, 0x00}, /* HOPBOUT_EN */
	{0x3035, 0x01}, /* HTRIMMING_EN */
	{0x3036, 0x30}, /* HTRIMMING_START */
	{0x3037, 0x00}, /* HTRIMMING_START */
	{0x3038, 0x00}, /* HTRIMMING_END */
	{0x3039, 0x0F}, /* HTRIMMING_END */
	{0x3068, 0x1A}, /* MDSEL15 */
	{0x3069, 0x00}, /* MDSEL15 */
	{0x3080, 0x00}, /* MDSEL6 */
	{0x3081, 0x01}, /* MDSEL7 */
	{0x30A8, 0x02}, /* MDSEL8 */
	{0x30E2, 0x00}, /* VCUTMODE */
	{0x312F, 0x10}, /* OPB_SIZE_V */
	{0x3130, 0x18}, /* WRITE_VSIZE */
	{0x3131, 0x0B}, /* WRITE_VSIZE */
	{0x3132, 0x08}, /* OUT_SIZE */
	{0x3133, 0x0B}, /* Y_OUT_SIZE */
	{0x357F, 0x0C}, /* MDSEL11 */
	{0x3580, 0x0A}, /* MDSEL12 */
	{0x3581, 0x08}, /* MDSEL13 */
	{0x3583, 0x72}, /* MDSEL14 */
	{0x3600, 0x90}, /* MDSEL16 */
	{0x3601, 0x00}, /* MDSEL16 */
	{0x3846, 0x00}, /* MDSEL9 */
	{0x3847, 0x00}, /* MDSEL9 */
	{0x384A, 0x00}, /* MDSEL10 */
	{0x384B, 0x00}, /* MDSEL10 */
};

/* 4096 x 2160 readout mode 1 */
static const struct imx29x_reg imx294_mode_01_regs[] = {
	{0x3004, 0x1A}, /* MDSEL1 */
	{0x3005, 0x06}, /* MDSEL2 */
	{0x3006, 0x00}, /* MDSEL3 */
	{0x3007, 0xA0}, /* MDSEL4 */
	{0x3019, 0x00}, /* MDVREV */
	{0x3030, 0x77}, /* MDSEL5 */
	{0x3034, 0x00}, /* HOPBOUT_EN */
	{0x3035, 0x01}, /* HTRIMMING_EN */
	{0x3036, 0x30}, /* HTRIMMING_START */
	{0x3037, 0x00}, /* HTRIMMING_START */
	{0x3038, 0x60}, /* HTRIMMING_END */
	{0x3039, 0x10}, /* HTRIMMING_END */
	{0x3068, 0x1A}, /* MDSEL15 */
	{0x3069, 0x00}, /* MDSEL15 */
	{0x3080, 0x00}, /* MDSEL6 */
	{0x3081, 0x01}, /* MDSEL7 */
	{0x30A8, 0x02}, /* MDSEL8 */
	{0x30E2, 0x00}, /* VCUTMODE */
	{0x312F, 0x08}, /* OPB_SIZE_V */
	{0x3130, 0x88}, /* WRITE_VSIZE */
	{0x3131, 0x08}, /* WRITE_VSIZE */
	{0x3132, 0x80}, /* OUT_SIZE */
	{0x3133, 0x08}, /* Y_OUT_SIZE */
	{0x357F, 0x0C}, /* MDSEL11 */
	{0x3580, 0x0A}, /* MDSEL12 */
	{0x3581, 0x08}, /* MDSEL13 */
	{0x3583, 0x72}, /* MDSEL14 */
	{0x3600, 0x90}, /* MDSEL16 */
	{0x3601, 0x00}, /* MDSEL16 */
	{0x3846, 0x00}, /* MDSEL9 */
	{0x3847, 0x00}, /* MDSEL9 */
	{0x384A, 0x00}, /* MDSEL10 */
	{0x384B, 0x00}, /* MDSEL10 */
};

/* 4096 x 2160 readout mode 2, 10-bit output */
static const struct imx29x_reg imx294_mode_02_10bit_regs[] = {
	{0x3004, 0x1A}, /* MDSEL1 */
	{0x3005, 0x01}, /* MDSEL2 */
	{0x3006, 0x00}, /* MDSEL3 */
	{0x3007, 0xA0}, /* MDSEL4 */
	{0x3019, 0x00}, /* MDVREV */
	{0x3030, 0x77}, /* MDSEL5 */
	{0x3034, 0x00}, /* HOPBOUT_EN */
	{0x3035, 0x01}, /* HTRIMMING_EN */
	{0x3036, 0x30}, /* HTRIMMING_START */
	{0x3037, 0x00}, /* HTRIMMING_START */
	{0x3038, 0x60}, /* HTRIMMING_END */
	{0x3039, 0x10}, /* HTRIMMING_END */
	{0x3068, 0x44}, /* MDSEL15 */
	{0x3069, 0x00}, /* MDSEL15 */
	{0x3080, 0x00}, /* MDSEL6 */
	{0x3081, 0x01}, /* MDSEL7 */
	{0x30A8, 0x02}, /* MDSEL8 */
	{0x30E2, 0x00}, /* VCUTMODE */
	{0x312F, 0x08}, /* OPB_SIZE_V */
	{0x3130, 0x88}, /* WRITE_VSIZE */
	{0x3131, 0x08}, /* WRITE_VSIZE */
	{0x3132, 0x80}, /* Y_OUT_SIZE */
	{0x3133, 0x08}, /* Y_OUT_SIZE */
	{0x357F, 0x0C}, /* MDSEL11 */
	{0x3580, 0x0A}, /* MDSEL12 */
	{0x3581, 0x0A}, /* MDSEL13 */
	{0x3583, 0x75}, /* MDSEL14 */
	{0x3600, 0x90}, /* MDSEL16 */
	{0x3601, 0x00}, /* MDSEL16 */
	{0x3846, 0x00}, /* MDSEL9 */
	{0x3847, 0x00}, /* MDSEL9 */
	{0x384A, 0x00}, /* MDSEL10 */
	{0x384B, 0x00}, /* MDSEL10 */
};

/* 4096 x 2160 low noise readout mode 1A */
static const struct imx29x_reg imx294_mode_01A_regs[] = {
	{0x3004, 0x01}, /* MDSEL1 */
	{0x3005, 0x06}, /* MDSEL2 */
	{0x3006, 0x00}, /* MDSEL3 */
	{0x3007, 0xA0}, /* MDSEL4 */
	{0x3019, 0x00}, /* MDVREV */
	{0x3030, 0x77}, /* MDSEL5 */
	{0x3034, 0x00}, /* HOPBOUT_EN */
	{0x3035, 0x01}, /* HTRIMMING_EN */
	{0x3036, 0x30}, /* HTRIMMING_START */
	{0x3037, 0x00}, /* HTRIMMING_START */
	{0x3038, 0x80}, /* HTRIMMING_END */
	{0x3039, 0x10}, /* HTRIMMING_END */
	{0x3068, 0x1A}, /* MDSEL15 */
	{0x3069, 0x00}, /* MDSEL15 */
	{0x3080, 0x01}, /* MDSEL6 */
	{0x3081, 0x01}, /* MDSEL7 */
	{0x30A8, 0x02}, /* MDSEL8 */
	{0x30E2, 0x00}, /* VCUTMODE */
	{0x312F, 0x08}, /* OPB_SIZE_V */
	{0x3130, 0x88}, /* WRITE_VSIZE */
	{0x3131, 0x08}, /* WRITE_VSIZE */
	{0x3132, 0x80}, /* OUT_SIZE */
	{0x3133, 0x08}, /* Y_OUT_SIZE */
	{0x357F, 0x0C}, /* MDSEL11 */
	{0x3580, 0x0A}, /* MDSEL12 */
	{0x3581, 0x08}, /* MDSEL13 */
	{0x3583, 0x72}, /* MDSEL14 */
	{0x3600, 0x7D}, /* MDSEL16 */
	{0x3601, 0x00}, /* MDSEL16 */
	{0x3846, 0x00}, /* MDSEL9 */
	{0x3847, 0x00}, /* MDSEL9 */
	{0x384A, 0x00}, /* MDSEL10 */
	{0x384B, 0x00}, /* MDSEL10 */
};

/* 3840 x 2160 readout mode 1B */
static const struct imx29x_reg imx294_mode_01B_regs[] = {
	{0x3004, 0x02}, /* MDSEL1 */
	{0x3005, 0x06}, /* MDSEL2 */
	{0x3006, 0x01}, /* MDSEL3 */
	{0x3007, 0xA0}, /* MDSEL4 */
	{0x3019, 0x00}, /* MDVREV */
	{0x3030, 0x77}, /* MDSEL5 */
	{0x3034, 0x00}, /* HOPBOUT_EN */
	{0x3035, 0x01}, /* HTRIMMING_EN */
	{0x3036, 0x30}, /* HTRIMMING_START */
	{0x3037, 0x00}, /* HTRIMMING_START */
	{0x3038, 0x50}, /* HTRIMMING_END */
	{0x3039, 0x0F}, /* HTRIMMING_END */
	{0x3068, 0x1A}, /* MDSEL15 */
	{0x3069, 0x00}, /* MDSEL15 */
	{0x3080, 0x00}, /* MDSEL6 */
	{0x3081, 0x01}, /* MDSEL7 */
	{0x30A8, 0x02}, /* MDSEL8 */
	{0x30E2, 0x00}, /* VCUTMODE */
	{0x312F, 0x08}, /* OPB_SIZE_V */
	{0x3130, 0x88}, /* WRITE_VSIZE */
	{0x3131, 0x08}, /* WRITE_VSIZE */
	{0x3132, 0x80}, /* OUT_SIZE */
	{0x3133, 0x08}, /* Y_OUT_SIZE */
	{0x357F, 0x0C}, /* MDSEL11 */
	{0x3580, 0x0A}, /* MDSEL12 */
	{0x3581, 0x08}, /* MDSEL13 */
	{0x3583, 0x72}, /* MDSEL14 */
	{0x3600, 0x90}, /* MDSEL16 */
	{0x3601, 0x00}, /* MDSEL16 */
	{0x3846, 0x00}, /* MDSEL9 */
	{0x3847, 0x00}, /* MDSEL9 */
	{0x384A, 0x00}, /* MDSEL10 */
	{0x384B, 0x00}, /* MDSEL10 */
};

/* Mode configs */
static const struct imx29x_mode imx294_modes_14bit[] = {
	{
		/* 3704 x 2778 readout mode 0, trimmed to active output lines */
		.width = 3792,
		.height = 2824,
		.min_hmax = 1730,
		.min_vmax = 1444,
		.default_hmax = 1875,
		.default_vmax = 1600, /* 24 FPS */
		.vmax_scale = 2,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 551,
		.crop = {
			.left = 80,
			.top = 48,
			.width = 7408,
			.height = 5524,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_mode_00_14bit_regs),
			.regs = imx294_mode_00_14bit_regs,
		},
	},
};

/* Mode configs */
static const struct imx29x_mode imx294_modes_10bit[] = {
	{
		/* 4096 x 2160 readout mode 2, trimmed to active output lines */
		.width = 4144,
		.height = 2176,
		.min_hmax = 947,
		.min_vmax = 1116,
		.default_hmax = 975,
		.default_vmax = 1232, /* 59.94 FPS */
		.vmax_scale = 2,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 256,
		.crop = {
			.left = 24,
			.top = 40,
			.width = 8192,
			.height = 4304,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_mode_02_10bit_regs),
			.regs = imx294_mode_02_10bit_regs,
		},
	},
};

/* Mode configs */
static const struct imx29x_mode imx294_modes_12bit[] = {
	{
		/* Quad Bayer all-pixel 12-bit mode */
		.width = 8432,
		.height = 5648,
		.min_hmax = 1202,
		.min_vmax = 5728,
		.default_hmax = 1202,
		.default_vmax = 5728,
		.vmax_scale = 1,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 256,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 8240,
			.height = 5628,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_quad_all_pixel_12bit_regs),
			.regs = imx294_quad_all_pixel_12bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x1630,
		.y_out_size = 0x1610,
	}, {
		/* Quad Bayer 17:9 12-bit mode */
		.width = 8432,
		.height = 4348,
		.min_hmax = 1202,
		.min_vmax = 4428,
		.default_hmax = 1202,
		.default_vmax = 4428,
		.vmax_scale = 1,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 256,
		.crop = {
			.left = 0,
			.top = 646,
			.width = 8240,
			.height = 4336,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_quad_wide_17_9_12bit_regs),
			.regs = imx294_quad_wide_17_9_12bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x111c,
		.y_out_size = 0x10fc,
	}, {
		/* Quad Bayer 4:3 12-bit mode */
		.width = 7680,
		.height = 5648,
		.min_hmax = 1108,
		.min_vmax = 5728,
		.default_hmax = 1108,
		.default_vmax = 5728,
		.vmax_scale = 1,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 256,
		.crop = {
			.left = 392,
			.top = 0,
			.width = 7456,
			.height = 5628,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_quad_four_three_12bit_regs),
			.regs = imx294_quad_four_three_12bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x1630,
		.y_out_size = 0x1610,
	},
	{
		/* 4096 x 2160 readout mode 1, trimmed to active output lines */
		.width = 4144,
		.height = 2176,
		.min_hmax = 1122,
		.min_vmax = 1111,
		.default_hmax = 1200,
		.default_vmax = 2500, /* 24 FPS */
		.vmax_scale = 2,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 256,
		.crop = {
			.left = 24,
			.top = 40,
			.width = 8192,
			.height = 4304,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_mode_01_regs),
			.regs = imx294_mode_01_regs,
		},
	},
	{
		/* 4096 x 2160 low noise readout mode 1A, trimmed to active output lines */
		.width = 4176,
		.height = 2176,
		.min_hmax = 1192,
		.min_vmax = 1111,
		.default_hmax = 1200,
		.default_vmax = 2500, /* 24 FPS */
		.vmax_scale = 2,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 361,
		.crop = {
			.left = 24,
			.top = 40,
			.width = 8192,
			.height = 4304,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_mode_01A_regs),
			.regs = imx294_mode_01A_regs,
		},
	},
	{
		/* 3840 x 2160 low noise readout mode 1B, trimmed to active output lines */
		.width = 3872,
		.height = 2176,
		.min_hmax = 1055,
		.min_vmax = 1111,
		.default_hmax = 1200,
		.default_vmax = 2500, /* 50 FPS */
		.vmax_scale = 2,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 256,
		.crop = {
			.left = 40,
			.top = 40,
			.width = 7680,
			.height = 4312,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_mode_01B_regs),
			.regs = imx294_mode_01B_regs,
		},
	},
	{
		/* 3740 x 2778 readout mode 0, trimmed to active output lines */
		.width = 3792,
		.height = 2824,
		.min_hmax = 1024,
		.min_vmax = 1444,
		.default_hmax = 1875,
		.default_vmax = 1600, /* 24 FPS */
		.vmax_scale = 2,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 551,
		.crop = {
			.left = 80,
			.top = 48,
			.width = 7408,
			.height = 5524,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx294_mode_00_regs),
			.regs = imx294_mode_00_regs,
		},
	},
};

#define IMX294_12BIT_QUAD_MODE_COUNT 3

/*
 * The supported IMX294 Bayer formats. Alternate Bayer orders are accepted as
 * requests and canonicalized to these fixed CFA codes because this driver does
 * not expose HFLIP/VFLIP controls that would change the active Bayer order.
 */
static const u32 imx294_color_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SRGGB14_1X14,
};

/*
 * The 12-bit binned modes use the IMX294-style startup table.  The shorter
 * IMX29X full-resolution startup sequence does not bring these modes out of
 * standby reliably on the colour IMX29X.
 */
static const struct imx29x_reg imx492_binned_common_regs[] = {
	{0x3000, 0x12},
	{0x310B, 0x00},
	{IMX29X_REG_BLKLEVEL, IMX29X_BLKLEVEL_DEFAULT},
	{0x3047, 0x01},
	{0x304E, 0x0B},
	{0x304F, 0x24},
	{0x3062, 0x25},
	{0x3064, 0x78},
	{0x3065, 0x33},
	{0x3067, 0x71},
	{0x3088, 0x75},
	{0x308A, 0x09},
	{0x308B, 0x01},
	{0x308C, 0x61},
	{0x3146, 0x00},
	{0x3234, 0x32},
	{0x3235, 0x00},
	{0x3248, 0xBC},
	{0x3249, 0x00},
	{0x3250, 0xBC},
	{0x3251, 0x00},
	{0x3258, 0xBC},
	{0x3259, 0x00},
	{0x3260, 0xBC},
	{0x3261, 0x00},
	{0x3274, 0x13},
	{0x3275, 0x00},
	{0x3276, 0x1F},
	{0x3277, 0x00},
	{0x3278, 0x30},
	{0x3279, 0x00},
	{0x327C, 0x13},
	{0x327D, 0x00},
	{0x327E, 0x1F},
	{0x327F, 0x00},
	{0x3280, 0x30},
	{0x3281, 0x00},
	{0x3284, 0x13},
	{0x3285, 0x00},
	{0x3286, 0x1F},
	{0x3287, 0x00},
	{0x3288, 0x30},
	{0x3289, 0x00},
	{0x328C, 0x13},
	{0x328D, 0x00},
	{0x328E, 0x1F},
	{0x328F, 0x00},
	{0x3290, 0x30},
	{0x3291, 0x00},
	{0x32AE, 0x00},
	{0x32AF, 0x00},
	{0x32CA, 0x5A},
	{0x32CB, 0x00},
	{0x332F, 0x00},
	{0x334C, 0x01},
	{0x335A, 0x79},
	{0x335B, 0x00},
	{0x335E, 0x56},
	{0x335F, 0x00},
	{0x3360, 0x6A},
	{0x3361, 0x00},
	{0x336A, 0x56},
	{0x336B, 0x00},
	{0x33D6, 0x79},
	{0x33D7, 0x00},
	{0x340C, 0x6E},
	{0x340D, 0x00},
	{0x3448, 0x7E},
	{0x3449, 0x00},
	{0x348E, 0x6F},
	{0x348F, 0x00},
	{0x3492, 0x11},
	{0x34C4, 0x5A},
	{0x34C5, 0x00},
	{0x3506, 0x56},
	{0x3507, 0x00},
	{0x350C, 0x56},
	{0x350D, 0x00},
	{0x350E, 0x58},
	{0x350F, 0x00},
	{0x3549, 0x04},
	{0x355D, 0x03},
	{0x355E, 0x03},
	{0x3574, 0x56},
	{0x3575, 0x00},
	{0x3587, 0x01},
	{0x35D0, 0x5E},
	{0x35D1, 0x00},
	{0x35D4, 0x63},
	{0x35D5, 0x00},
	{0x366A, 0x1A},
	{0x366B, 0x16},
	{0x366C, 0x10},
	{0x366D, 0x09},
	{0x366E, 0x00},
	{0x366F, 0x00},
	{0x3670, 0x00},
	{0x3671, 0x00},
	{0x3676, 0x83},
	{0x3677, 0x03},
	{0x3678, 0x00},
	{0x3679, 0x04},
	{0x367A, 0x2C},
	{0x367B, 0x05},
	{0x367C, 0x00},
	{0x367D, 0x06},
	{0x367E, 0x00},
	{0x367F, 0x07},
	{0x3680, 0x4B},
	{0x3681, 0x07},
	{0x3690, 0x27},
	{0x3691, 0x00},
	{0x3692, 0x65},
	{0x3693, 0x00},
	{0x3694, 0x4F},
	{0x3695, 0x00},
	{0x3696, 0xA1},
	{0x3697, 0x00},
	{0x382B, 0x68},
	{0x3C00, 0x01},
	{0x3C01, 0x01},
	{0x3686, 0x00},
	{0x3687, 0x00},
	{0x36BE, 0x01},
	{0x36BF, 0x00},
	{0x36C0, 0x01},
	{0x36C1, 0x00},
	{0x36C2, 0x01},
	{0x36C3, 0x00},
	{0x36C4, 0x01},
	{0x36C5, 0x01},
	{0x36C6, 0x01},
	{0x3134, 0xAF},
	{0x3135, 0x00},
	{0x3136, 0xC7},
	{0x3137, 0x00},
	{0x3138, 0x7F},
	{0x3139, 0x00},
	{0x313A, 0x6F},
	{0x313B, 0x00},
	{0x313C, 0x6F},
	{0x313D, 0x00},
	{0x313E, 0xCF},
	{0x313F, 0x01},
	{0x3140, 0x77},
	{0x3141, 0x00},
	{0x3142, 0x5F},
	{0x3143, 0x00},
	{0x3004, 0x1A},
	{0x3005, 0x06},
	{0x3006, 0x00},
	{0x3007, 0xA0},
	{0x3019, 0x00},
	{0x3030, 0x77},
	{0x3034, 0x00},
	{0x3035, 0x01},
	{0x3036, 0x30},
	{0x3037, 0x00},
	{0x3038, 0x60},
	{0x3039, 0x10},
	{0x3068, 0x1A},
	{0x3069, 0x00},
	{0x3080, 0x00},
	{0x3081, 0x01},
	{0x30A8, 0x02},
	{0x30E2, 0x00},
	{0x312F, 0x08},
	{0x3130, 0x88},
	{0x3131, 0x08},
	{0x3132, 0x80},
	{0x3133, 0x08},
	{0x357F, 0x0C},
	{0x3580, 0x0A},
	{0x3581, 0x08},
	{0x3583, 0x72},
	{0x3600, 0x90},
	{0x3601, 0x00},
	{0x3846, 0x00},
	{0x3847, 0x00},
	{0x384A, 0x00},
	{0x384B, 0x00},
	{0x300E, 0x00},
	{0x300F, 0x00},
	{0x302C, 0x10},
	{0x302D, 0x00},
	{0x30A9, 0x88},
	{0x30AA, 0x13},
	{0x30AB, 0x00},
	{0x30AC, 0xB0},
	{0x30AD, 0x04},
	{0x3084, 0xB0},
	{0x3085, 0x04},
	{0x3086, 0xB0},
	{0x3087, 0x04},
	{0x332C, 0x00},
	{0x332D, 0x00},
	{0x334A, 0x00},
	{0x334B, 0x00},
	{0x35B6, 0x00},
	{0x35B7, 0x00},
	{0x35B8, 0x00},
	{0x35B9, 0x00},
	{0x36BC, 0x00},
	{0x36BD, 0x00},
};

/* IMX492 color sensors can opt in to the IMX294-style 12-bit binned mode. */
static const struct imx29x_reg imx492_binned_mode_00_regs[] = {
	{0x3004, 0x00}, {0x3005, 0x06}, {0x3006, 0x02}, {0x3007, 0xA0},
	{0x3019, 0x00}, {0x3030, 0x77}, {0x3034, 0x00}, {0x3035, 0x01},
	{0x3036, 0x30}, {0x3037, 0x00}, {0x3038, 0x00}, {0x3039, 0x0F},
	{0x3068, 0x1A}, {0x3069, 0x00}, {0x3080, 0x00}, {0x3081, 0x01},
	{0x30A8, 0x02}, {0x30E2, 0x00}, {0x312F, 0x10}, {0x3130, 0x18},
	{0x3131, 0x0B}, {0x3132, 0x08}, {0x3133, 0x0B}, {0x357F, 0x0C},
	{0x3580, 0x0A}, {0x3581, 0x08}, {0x3583, 0x72}, {0x3600, 0x90},
	{0x3601, 0x00}, {0x3846, 0x00}, {0x3847, 0x00}, {0x384A, 0x00},
	{0x384B, 0x00},
};

#define IMX492_12BIT_FULLRES_MODE_COUNT 3

static const struct imx29x_mode imx492_modes_10bit[] = {
	{
		.width = 8432,
		.height = 5648,
		.min_hmax = 920,
		.min_vmax = 5728,
		.default_hmax = 920,
		.default_vmax = 5728,
		.vmax_scale = 1,
		.min_shr = 12,
		.max_shr_margin = 4,
		.integration_offset = 217,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 8240,
			.height = 5628,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx29x_all_pixel_10bit_regs),
			.regs = imx29x_all_pixel_10bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x1630,
		.y_out_size = 0x1610,
	}, {
		.width = 8432,
		.height = 4348,
		.min_hmax = 920,
		.min_vmax = 4428,
		.default_hmax = 920,
		.default_vmax = 4428,
		.vmax_scale = 1,
		.min_shr = 12,
		.max_shr_margin = 4,
		.integration_offset = 217,
		.crop = {
			.left = 0,
			.top = 646,
			.width = 8240,
			.height = 4336,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx29x_wide_17_9_10bit_regs),
			.regs = imx29x_wide_17_9_10bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x111c,
		.y_out_size = 0x10fc,
	}, {
		.width = 7680,
		.height = 5648,
		.min_hmax = 842,
		.min_vmax = 5728,
		.default_hmax = 842,
		.default_vmax = 5728,
		.vmax_scale = 1,
		.min_shr = 12,
		.max_shr_margin = 4,
		.integration_offset = 217,
		.crop = {
			.left = 392,
			.top = 0,
			.width = 7456,
			.height = 5628,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx29x_four_three_10bit_regs),
			.regs = imx29x_four_three_10bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x1630,
		.y_out_size = 0x1610,
	},
};

static const struct imx29x_mode imx492_modes_12bit[] = {
	{
		.width = 8432,
		.height = 5648,
		.min_hmax = 1202,
		.min_vmax = 5728,
		.default_hmax = 1202,
		.default_vmax = 5728,
		.vmax_scale = 1,
		.min_shr = 12,
		.max_shr_margin = 4,
		.integration_offset = 256,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 8240,
			.height = 5628,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx29x_all_pixel_12bit_regs),
			.regs = imx29x_all_pixel_12bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x1630,
		.y_out_size = 0x1610,
	}, {
		.width = 8432,
		.height = 4348,
		.min_hmax = 1202,
		.min_vmax = 4428,
		.default_hmax = 1202,
		.default_vmax = 4428,
		.vmax_scale = 1,
		.min_shr = 12,
		.max_shr_margin = 4,
		.integration_offset = 256,
		.crop = {
			.left = 0,
			.top = 646,
			.width = 8240,
			.height = 4336,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx29x_wide_17_9_12bit_regs),
			.regs = imx29x_wide_17_9_12bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x111c,
		.y_out_size = 0x10fc,
	}, {
		.width = 7680,
		.height = 5648,
		.min_hmax = 1108,
		.min_vmax = 5728,
		.default_hmax = 1108,
		.default_vmax = 5728,
		.vmax_scale = 1,
		.min_shr = 12,
		.max_shr_margin = 4,
		.integration_offset = 256,
		.crop = {
			.left = 392,
			.top = 0,
			.width = 7456,
			.height = 5628,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx29x_four_three_12bit_regs),
			.regs = imx29x_four_three_12bit_regs,
		},
		.use_output_overrides = true,
		.opb_size_v = 0x20,
		.write_vsize = 0x1630,
		.y_out_size = 0x1610,
	}, {
		.width = 3792,
		.height = 2824,
		// IMX294-style binned mode needs doubled HMAX/HCOUNT on colour IMX29X.
		.min_hmax = 2048,
		.min_vmax = 1444,
		.default_hmax = 3750,
		.default_vmax = 1600,
		.vmax_scale = 2,
		.min_shr = 5,
		.max_shr_margin = 1,
		.integration_offset = 551,
		.crop = {
			.left = 392,
			.top = 0,
			.width = 7456,
			.height = 5596,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx492_binned_mode_00_regs),
			.regs = imx492_binned_mode_00_regs,
		},
		.use_output_overrides = false,
		.opb_size_v = 0x10,
		.write_vsize = 0x0b18,
		.y_out_size = 0x0b08,
	},
};

/*
 * The supported native color Bayer formats. Alternate Bayer orders are
 * accepted as requests and canonicalized to these fixed CFA codes because this
 * driver does not expose HFLIP/VFLIP controls that would change the active
 * Bayer order.
 */
static const u32 imx492_color_codes[] = {
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12,
};

static const u32 imx492_mono_codes[] = {
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_Y12_1X12,
};

static const s64 imx29x_link_freq_menu[] = {
	IMX29X_LINK_FREQ,
};

static const char * const imx29x_test_pattern_menu[] = {
	"Disabled",
	"All 0000h pattern",
	"All 3FFFh pattern",
	"All 1555h pattern",
	"All 2AAAh pattern",
	"Horizontal color bar",
	"Vertical color bar",
};

static const u8 imx29x_test_pattern_sel[] = {
	0x00,
	0x01,
	0x02,
	0x03,
	0x0A,
	0x0B,
};

/* regulator supplies */
static const char * const imx29x_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define IMX29X_NUM_SUPPLIES ARRAY_SIZE(imx29x_supply_name)

/*
 * XCLR is active low. The datasheet specifies 100 ns minimum low pulse width
 * and permits standby-cancel register access after XCLR is released. Keep a
 * conservative 10 ms release guard before the first I2C transaction.
 */
#define IMX29X_XCLR_DEASSERT_DELAY_US	10000
#define IMX29X_XCLR_DELAY_RANGE_US	1000

/* XCLR is an active-low reset pin; descriptor values are logical. */
#define IMX29X_XCLR_ASSERTED		1
#define IMX29X_XCLR_DEASSERTED		0

enum imx29x_model {
	IMX29X_MODEL_IMX294,
	IMX29X_MODEL_IMX492,
};

struct imx29x_compatible_data {
	enum imx29x_model model;
	bool mono;
	bool supports_mono;
	bool supports_quad_bayer_modes;
	bool supports_color_binned_modes;
	const struct imx29x_reg *binned_common_regs;
	unsigned int num_binned_common_regs;
};

struct imx29x {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;
	const struct imx29x_plrd_setup *plrd_setup;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX29X_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *test_pattern;

	/* Current mode */
	const struct imx29x_mode *mode;

	u16 hmax;
	u32 vmax;
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Any extra information related to different compatible sensors */
	const struct imx29x_compatible_data *compatible_data;
	bool mono;
	bool quad_bayer_modes;
	bool color_binned_modes;
	bool active_state_initialized;
};

static inline struct imx29x *to_imx29x(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx29x, sd);
}

static inline void get_mode_table(struct imx29x *imx29x, unsigned int code,
				  const struct imx29x_mode **mode_list,
				  unsigned int *num_modes)
{
	if (imx29x->compatible_data->model == IMX29X_MODEL_IMX294) {
		switch (code) {
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			*mode_list = imx294_modes_10bit;
			*num_modes = ARRAY_SIZE(imx294_modes_10bit);
			return;
		case MEDIA_BUS_FMT_SRGGB14_1X14:
			*mode_list = imx294_modes_14bit;
			*num_modes = ARRAY_SIZE(imx294_modes_14bit);
			return;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			if (imx29x->quad_bayer_modes) {
				*mode_list = imx294_modes_12bit;
				*num_modes = ARRAY_SIZE(imx294_modes_12bit);
			} else {
				*mode_list = imx294_modes_12bit +
					     IMX294_12BIT_QUAD_MODE_COUNT;
				*num_modes = ARRAY_SIZE(imx294_modes_12bit) -
					     IMX294_12BIT_QUAD_MODE_COUNT;
			}
			return;
		default:
			break;
		}
	} else {
		switch (code) {
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_Y10_1X10:
			*mode_list = imx492_modes_10bit;
			*num_modes = ARRAY_SIZE(imx492_modes_10bit);
			return;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_Y12_1X12:
			*mode_list = imx492_modes_12bit;
			*num_modes = (imx29x->color_binned_modes ||
				     imx29x->mono) ?
				     ARRAY_SIZE(imx492_modes_12bit) :
				     IMX492_12BIT_FULLRES_MODE_COUNT;
			return;
		default:
			break;
		}
	}

	*mode_list = NULL;
	*num_modes = 0;
}

static u32 imx29x_get_format_bpp(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_Y10_1X10:
		return 10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_Y12_1X12:
		return 12;
	case MEDIA_BUS_FMT_SBGGR14_1X14:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
	case MEDIA_BUS_FMT_SRGGB14_1X14:
		return 14;
	default:
		return 12;
	}
}

static u64 imx29x_get_pixel_rate(u32 code)
{
	u64 pixel_rate = IMX29X_LINK_FREQ * 2 * IMX29X_NUM_DATA_LANES;

	do_div(pixel_rate, imx29x_get_format_bpp(code));

	return pixel_rate;
}

static u64 imx29x_get_mode_pixel_rate(struct imx29x *imx29x,
				      const struct imx29x_mode *mode)
{
	u64 pixel_rate;

	if (mode->use_output_overrides)
		return imx29x_get_pixel_rate(imx29x->fmt_code);

	pixel_rate = (u64)mode->width * IMX29X_TIMING_CLOCK_HZ * mode->vmax_scale;
	do_div(pixel_rate, mode->min_hmax);

	return pixel_rate;
}

static u64 imx29x_hblank_from_hmax(struct imx29x *imx29x,
				   const struct imx29x_mode *mode,
				   u64 hmax)
{
	u64 line_length;
	u64 denom = IMX29X_TIMING_CLOCK_HZ * mode->vmax_scale;

	line_length = DIV_ROUND_UP_ULL(hmax * imx29x_get_mode_pixel_rate(imx29x, mode),
				       denom);

	if (line_length <= mode->width)
		return 0;

	return line_length - mode->width;
}

static u64 imx29x_hmax_from_hblank(struct imx29x *imx29x,
				   const struct imx29x_mode *mode,
				   u64 hblank)
{
	u64 hmax = (u64)(mode->width + hblank) *
		   IMX29X_TIMING_CLOCK_HZ * mode->vmax_scale;

	do_div(hmax, imx29x_get_mode_pixel_rate(imx29x, mode));

	return hmax;
}

/* Read registers up to 2 at a time */
static int imx29x_read_reg(struct imx29x *imx29x, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers 1 byte at a time */
static int imx29x_write_reg_1byte(struct imx29x *imx29x, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	u8 buf[3];

	put_unaligned_be16(reg, buf);
	buf[2] = val;
	if (i2c_master_send(client, buf, 3) != 3)
		return -EIO;

	return 0;
}

/* Write registers 2 byte at a time */
static int imx29x_write_reg_2byte(struct imx29x *imx29x, u16 reg, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	u8 buf[4];

	put_unaligned_be16(reg, buf);
	buf[2] = val;
	buf[3] = val >> 8;
	if (i2c_master_send(client, buf, 4) != 4)
		return -EIO;

	return 0;
}

/* Write registers 3 byte at a time */
static int imx29x_write_reg_3byte(struct imx29x *imx29x, u16 reg, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	u8 buf[5];

	put_unaligned_be16(reg, buf);
	buf[2] = val;
	buf[3] = val >> 8;
	buf[4] = val >> 16;
	if (i2c_master_send(client, buf, 5) != 5)
		return -EIO;

	return 0;
}

/* Write a list of 1 byte registers */
static int imx29x_write_regs(struct imx29x *imx29x,
			     const struct imx29x_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		if (regs[i].address == 0xFFFE) {
			usleep_range(regs[i].val * 1000,
				     (regs[i].val + 1) * 1000);
		} else {
			ret = imx29x_write_reg_1byte(imx29x, regs[i].address,
						     regs[i].val);
			if (ret) {
				dev_err_ratelimited(&client->dev,
						    "Failed to write reg 0x%4.4x. error = %d\n",
						    regs[i].address, ret);

				return ret;
			}
		}
	}

	return 0;
}

static int imx29x_write_plrd_regs(struct imx29x *imx29x)
{
	return imx29x_write_regs(imx29x, imx29x->plrd_setup->regs,
				 ARRAY_SIZE(imx29x->plrd_setup->regs));
}

static u32 imx29x_default_format_code(const struct imx29x *imx29x, u32 code)
{
	u32 bpp = imx29x_get_format_bpp(code);

	if (imx29x->compatible_data->model == IMX29X_MODEL_IMX294)
		return bpp == 10 ? MEDIA_BUS_FMT_SRGGB10_1X10 :
		       bpp == 14 ? MEDIA_BUS_FMT_SRGGB14_1X14 :
				    MEDIA_BUS_FMT_SRGGB12_1X12;

	if (imx29x->mono)
		return bpp == 10 ? MEDIA_BUS_FMT_Y10_1X10 :
				   MEDIA_BUS_FMT_Y12_1X12;

	return bpp == 10 ? MEDIA_BUS_FMT_SRGGB10_1X10 :
			   MEDIA_BUS_FMT_SRGGB12_1X12;
}

static u32 imx29x_get_format_code(struct imx29x *imx29x, u32 code)
{
	unsigned int i;

	if (imx29x->compatible_data->model == IMX29X_MODEL_IMX294) {
		switch (code) {
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			return MEDIA_BUS_FMT_SRGGB10_1X10;
		case MEDIA_BUS_FMT_SBGGR14_1X14:
		case MEDIA_BUS_FMT_SGBRG14_1X14:
		case MEDIA_BUS_FMT_SGRBG14_1X14:
		case MEDIA_BUS_FMT_SRGGB14_1X14:
			return MEDIA_BUS_FMT_SRGGB14_1X14;
		case MEDIA_BUS_FMT_SBGGR12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		default:
			return MEDIA_BUS_FMT_SRGGB12_1X12;
		}
	}

	if (imx29x->mono) {
		for (i = 0; i < ARRAY_SIZE(imx492_mono_codes); i++)
			if (imx492_mono_codes[i] == code)
				return imx492_mono_codes[i];

		return imx29x_default_format_code(imx29x, code);
	}

	for (i = 0; i < ARRAY_SIZE(imx492_color_codes); i++)
		if (imx492_color_codes[i] == code)
			return imx492_color_codes[i];

	return imx29x_default_format_code(imx29x, code);
}

static void imx29x_set_default_format(struct imx29x *imx29x)
{
	if (imx29x->compatible_data->model == IMX29X_MODEL_IMX294) {
		/* Set default mode to the first exposed 12-bit IMX294 mode. */
		imx29x->mode = imx29x->quad_bayer_modes ?
			       &imx294_modes_12bit[0] :
			       &imx294_modes_12bit[IMX294_12BIT_QUAD_MODE_COUNT];
		imx29x->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
	} else {
		/* Set default mode to max resolution. */
		imx29x->mode = &imx492_modes_12bit[0];
		imx29x->fmt_code = imx29x->mono ? MEDIA_BUS_FMT_Y12_1X12 :
						    MEDIA_BUS_FMT_SRGGB12_1X12;
	}
}

static const struct imx29x_mode *
imx29x_find_mode(struct imx29x *imx29x, u32 code, u32 req_width,
		 u32 req_height)
{
	const struct imx29x_mode *mode_list;
	unsigned int num_modes;

	get_mode_table(imx29x, code, &mode_list, &num_modes);
	if (!mode_list || !num_modes)
		return imx29x->mode;

	return v4l2_find_nearest_size(mode_list, num_modes, width, height,
				      req_width, req_height);
}

static const struct imx29x_mode *imx29x_get_active_mode(struct imx29x *imx29x)
{
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;

	if (!imx29x->active_state_initialized)
		return imx29x->mode;

	if (!mutex_is_locked(&imx29x->mutex))
		return imx29x->mode;

	state = v4l2_subdev_get_locked_active_state(&imx29x->sd);
	if (!state)
		return imx29x->mode;

	fmt = v4l2_subdev_state_get_format(state, IMAGE_PAD);
	if (!fmt)
		return imx29x->mode;

	return imx29x_find_mode(imx29x, fmt->code, fmt->width, fmt->height);
}

static u64 calculate_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 shr,
				       u64 svr, u64 offset)
{
	u64 numerator;

	numerator = (vmax * (svr + 1) - shr) * hmax + offset;
	numerator += hmax - 1;
	do_div(numerator, hmax);

	return min_t(u64, numerator, 0xFFFFFFFF);
}

static u64 calculate_max_shr(u64 vmax, u64 svr, u64 max_shr_margin)
{
	u64 max_shr = (svr + 1) * vmax;

	if (max_shr <= max_shr_margin)
		return 0;

	max_shr -= max_shr_margin;

	return min_t(u64, max_shr, 0xFFFF);
}

static void calculate_min_max_v4l2_cid_exposure(u64 hmax, u64 vmax,
						u64 min_shr,
						u64 max_shr_margin,
						u64 svr, u64 offset,
						u64 *min_exposure,
						u64 *max_exposure)
{
	u64 max_shr = calculate_max_shr(vmax, svr, max_shr_margin);

	*min_exposure = calculate_v4l2_cid_exposure(hmax, vmax, max_shr,
						    svr, offset);
	*max_exposure = calculate_v4l2_cid_exposure(hmax, vmax, min_shr,
						    svr, offset);
}

static void imx29x_update_exposure_limits(struct imx29x *imx29x,
					  const struct imx29x_mode *mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	u64 current_exposure, max_exposure, min_exposure;

	calculate_min_max_v4l2_cid_exposure(imx29x->hmax, imx29x->vmax,
					    (u64)mode->min_shr,
					    (u64)mode->max_shr_margin, 0,
					    mode->integration_offset,
					    &min_exposure, &max_exposure);

	current_exposure = clamp_t(u64, imx29x->exposure->val,
				   min_exposure, max_exposure);

	dev_dbg(&client->dev,
		"exposure_max:%lld, exposure_min:%lld, current_exposure:%lld\n",
		max_exposure, min_exposure, current_exposure);
	dev_dbg(&client->dev, "\tVMAX:%d, HMAX:%d\n", imx29x->vmax,
		imx29x->hmax);
	__v4l2_ctrl_modify_range(imx29x->exposure, min_exposure, max_exposure,
				 IMX29X_EXPOSURE_STEP, current_exposure);
}

/*
 * Integration Time [s] = [{VMAX × (SVR + 1) – (SHR)}
 *  × HMAX + offset] / (72 × 10^6)
 *
 * Integration Time [s] = exposure * HMAX / (72 × 10^6)
 */

static u32 calculate_shr(u32 exposure, u32 hmax, u64 vmax, u32 svr,
			 u32 offset, u64 min_shr, u64 max_shr_margin)
{
	u64 max_shr = calculate_max_shr(vmax, svr, max_shr_margin);
	u64 period = vmax * (svr + 1);
	u64 temp;
	u64 shr;

	temp = (u64)exposure * hmax;
	if (temp > offset)
		temp -= offset;
	else
		temp = 0;
	do_div(temp, hmax);

	shr = period > temp ? period - temp : min_shr;
	max_shr = max_t(u64, max_shr, min_shr);

	return clamp_t(u64, shr, min_shr, max_shr);
}

static int imx29x_set_test_pattern(struct imx29x *imx29x, u32 pattern)
{
	int ret;

	if (pattern == 0) {
		ret = imx29x_write_reg_1byte(imx29x, IMX29X_REG_TEST_PATTERN_CTRL,
					     0x00);
		if (ret)
			return ret;

		return imx29x_write_reg_1byte(imx29x, IMX29X_REG_TEST_PATTERN_SEL,
					      0x00);
	}

	if (pattern > ARRAY_SIZE(imx29x_test_pattern_menu) - 1)
		return -EINVAL;

	ret = imx29x_write_reg_1byte(imx29x, IMX29X_REG_TEST_PATTERN_CTRL,
				     IMX29X_TEST_PATTERN_ENABLE_MIPI);
	if (ret)
		return ret;

	return imx29x_write_reg_1byte(imx29x, IMX29X_REG_TEST_PATTERN_SEL,
				      imx29x_test_pattern_sel[pattern - 1]);
}

static int imx29x_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx29x *imx29x =
		container_of(ctrl->handler, struct imx29x, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	const struct imx29x_mode *mode = imx29x_get_active_mode(imx29x);
	u64 shr, vblk, tmp;
	int ret = 0;

	/*
	 * Timing controls may change the limits of usable exposure, so check
	 * and adjust if necessary before applying register writes.
	 */
	if (ctrl->id == V4L2_CID_VBLANK) {
		tmp = ((u64)mode->height + ctrl->val);
		do_div(tmp, mode->vmax_scale);
		imx29x->vmax = tmp;
		imx29x_update_exposure_limits(imx29x, mode);
	} else if (ctrl->id == V4L2_CID_HBLANK) {
		imx29x->hmax = imx29x_hmax_from_hblank(imx29x, mode, ctrl->val);
		imx29x_update_exposure_limits(imx29x, mode);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "V4L2_CID_EXPOSURE : %d\n", ctrl->val);
		dev_dbg(&client->dev, "\tvblank:%d, hblank:%d\n",
			imx29x->vblank->val, imx29x->hblank->val);
		dev_dbg(&client->dev, "\tVMAX:%d, HMAX:%d\n",
			imx29x->vmax, imx29x->hmax);
		shr = calculate_shr(ctrl->val, imx29x->hmax, imx29x->vmax,
				    0, mode->integration_offset, mode->min_shr,
				    mode->max_shr_margin);
		dev_dbg(&client->dev, "\tSHR:%lld\n", shr);
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_SHR, shr);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "V4L2_CID_ANALOGUE_GAIN : %d\n",
			ctrl->val);
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_ANALOG_GAIN,
					     ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "V4L2_CID_VBLANK : %d\n", ctrl->val);
		dev_dbg(&client->dev, "\tVMAX : %d\n", imx29x->vmax);
		ret = imx29x_write_reg_3byte(imx29x, IMX29X_REG_VMAX,
					     imx29x->vmax);
		if (ret)
			break;
		vblk = imx29x->vmax - mode->min_vmax;
		dev_dbg(&client->dev, "\tvblk : %lld\n", vblk);
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_PSSLVS1, vblk);
		if (ret)
			break;
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_PSSLVS2, vblk);
		if (ret)
			break;
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_PSSLVS3, vblk);
		if (ret)
			break;
		if (vblk <= 5)
			ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_PSSLVS4, 0);
		else
			ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_PSSLVS4,
						     vblk - 5);
		if (ret)
			break;
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_PSSLVS0, vblk);
		break;
	case V4L2_CID_HBLANK:
		dev_dbg(&client->dev, "V4L2_CID_HBLANK : %d\n", ctrl->val);
		dev_dbg(&client->dev, "\tHMAX : %d\n", imx29x->hmax);
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_HMAX,
					     imx29x->hmax);
		if (ret)
			break;
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_HCOUNT1,
					     imx29x->hmax);
		if (ret)
			break;
		ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_HCOUNT2,
					     imx29x->hmax);
		break;
	case V4L2_CID_TEST_PATTERN:
		dev_dbg(&client->dev, "V4L2_CID_TEST_PATTERN : %d\n", ctrl->val);
		ret = imx29x_set_test_pattern(imx29x, ctrl->val);
		break;
	default:
		dev_err(&client->dev, "ctrl(id:0x%x,val:0x%x) is not handled\n",
			ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx29x_ctrl_ops = {
	.s_ctrl = imx29x_set_ctrl,
};

static int imx29x_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx29x *imx29x = to_imx29x(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (imx29x->compatible_data->model == IMX29X_MODEL_IMX294) {
			if (code->index >= ARRAY_SIZE(imx294_color_codes))
				return -EINVAL;

			code->code = imx294_color_codes[code->index];
		} else if (imx29x->mono) {
			if (code->index >= ARRAY_SIZE(imx492_mono_codes))
				return -EINVAL;

			code->code = imx492_mono_codes[code->index];
		} else {
			if (code->index >= ARRAY_SIZE(imx492_color_codes))
				return -EINVAL;

			code->code = imx492_color_codes[code->index];
		}
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx29x_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx29x *imx29x = to_imx29x(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx29x_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(imx29x, fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx29x_get_format_code(imx29x, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX29X_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX29X_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx29x_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx29x_update_image_pad_format(struct imx29x *imx29x,
					   const struct imx29x_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx29x_reset_colorspace(&fmt->format);
}

static void imx29x_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX29X_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX29X_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

/* Update timing-dependent control ranges for the selected image mode. */
static void imx29x_set_framing_limits(struct imx29x *imx29x,
				      const struct imx29x_mode *mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	u64 def_hblank;
	u64 def_vblank;
	u64 max_hblank;
	u64 max_vblank;
	u64 min_hblank;
	u64 min_vblank;
	u64 pixel_rate = imx29x_get_mode_pixel_rate(imx29x, mode);

	imx29x->mode = mode;
	imx29x->vmax = mode->default_vmax;
	imx29x->hmax = mode->default_hmax;
	imx29x_update_exposure_limits(imx29x, mode);

	dev_dbg(&client->dev, "Pixel Rate : %lld\n", pixel_rate);

	min_hblank = imx29x_hblank_from_hmax(imx29x, mode, mode->min_hmax);
	def_hblank = imx29x_hblank_from_hmax(imx29x, mode, mode->default_hmax);
	max_hblank = imx29x_hblank_from_hmax(imx29x, mode, IMX29X_HMAX_MAX);
	min_vblank = mode->min_vmax * mode->vmax_scale - mode->height;
	max_vblank = IMX29X_VMAX_MAX * mode->vmax_scale - mode->height;
	def_vblank = mode->default_vmax * mode->vmax_scale - mode->height;

	__v4l2_ctrl_modify_range(imx29x->hblank, min_hblank,
				 max_hblank, 1, def_hblank);

	__v4l2_ctrl_s_ctrl(imx29x->hblank, def_hblank);

	/* Update limits and set FPS to default. */
	__v4l2_ctrl_modify_range(imx29x->vblank, min_vblank, max_vblank,
				 mode->vmax_scale, def_vblank);
	__v4l2_ctrl_s_ctrl(imx29x->vblank, def_vblank);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(imx29x->link_freq, 0);
	__v4l2_ctrl_modify_range(imx29x->pixel_rate, pixel_rate, pixel_rate,
				 1, pixel_rate);

	dev_dbg(&client->dev,
		"Setting default HBLANK : %lld, VBLANK : %lld with PixelRate: %lld\n",
		def_hblank, def_vblank, pixel_rate);
}

/* Apply pad format requests and keep active mode controls in sync. */
static int imx29x_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	struct v4l2_rect *crop;
	const struct imx29x_mode *mode;
	struct imx29x *imx29x = to_imx29x(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->pad == IMAGE_PAD) {
		const struct imx29x_mode *mode_list;
		unsigned int num_modes;

		/* Alternate Bayer orders are canonicalized to fixed native CFA. */
		fmt->format.code = imx29x_get_format_code(imx29x,
							  fmt->format.code);

		get_mode_table(imx29x, fmt->format.code, &mode_list, &num_modes);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx29x_update_image_pad_format(imx29x, mode, fmt);
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;

		crop = v4l2_subdev_state_get_crop(sd_state, IMAGE_PAD);
		*crop = mode->crop;

		if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
		    (imx29x->mode != mode ||
		     imx29x->fmt_code != fmt->format.code)) {
			imx29x->fmt_code = fmt->format.code;
			imx29x_set_framing_limits(imx29x, mode);
		}
	} else {
		/* Only one embedded data mode is supported. */
		imx29x_update_metadata_pad_format(fmt);
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	}

	return 0;
}

/* Return the crop rectangle stored in the requested subdev state. */
static const struct v4l2_rect *
__imx29x_get_pad_crop(struct imx29x *imx29x,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	}

	return NULL;
}

static int imx29x_apply_output_overrides(struct imx29x *imx29x,
					 const struct imx29x_mode *mode)
{
	int ret;

	ret = imx29x_write_reg_1byte(imx29x, IMX29X_REG_HOPBOUT_EN, 0x01);
	if (ret)
		return ret;

	ret = imx29x_write_reg_1byte(imx29x, IMX29X_REG_HTRIMMING_EN, 0x00);
	if (ret)
		return ret;

	ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_HTRIMMING_START,
				     0x0000);
	if (ret)
		return ret;

	ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_HTRIMMING_END,
				     0x0000);
	if (ret)
		return ret;

	ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_HCOUNT1, 0x0000);
	if (ret)
		return ret;

	ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_HCOUNT2, 0x0000);
	if (ret)
		return ret;

	ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_VWINPOS, 0x0000);
	if (ret)
		return ret;

	ret = imx29x_write_reg_1byte(imx29x, IMX29X_REG_OPB_SIZE_V,
				     mode->opb_size_v);
	if (ret)
		return ret;

	ret = imx29x_write_reg_2byte(imx29x, IMX29X_REG_WRITE_VSIZE,
				     mode->write_vsize);
	if (ret)
		return ret;

	return imx29x_write_reg_2byte(imx29x, IMX29X_REG_Y_OUT_SIZE,
				      mode->y_out_size);
}

static int imx29x_write_stream_on_sequence(struct imx29x *imx29x)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	int ret;

	ret = imx29x_write_regs(imx29x, imx29x_stream_on_regs,
				ARRAY_SIZE(imx29x_stream_on_regs));
	if (ret) {
		dev_err(&client->dev,
			"%s failed to run final stream-on sequence\n", __func__);
		return ret;
	}

	usleep_range(IMX29X_STREAM_DELAY_US,
		     IMX29X_STREAM_DELAY_US + IMX29X_STREAM_DELAY_RANGE_US);

	return 0;
}

static int imx29x_start_streaming_output(struct imx29x *imx29x,
					 const struct imx29x_mode *mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	const struct imx29x_reg_list *reg_list = &mode->reg_list;
	int ret;

	ret = imx29x_write_regs(imx29x, imx29x_startup_pre_regs,
				ARRAY_SIZE(imx29x_startup_pre_regs));
	if (ret) {
		dev_err(&client->dev,
			"%s failed to run startup pre-sequence\n",
			__func__);
		return ret;
	}

	ret = imx29x_write_plrd_regs(imx29x);
	if (ret) {
		dev_err(&client->dev, "%s failed to set PLRD\n", __func__);
		return ret;
	}

	ret = imx29x_write_regs(imx29x, imx29x_startup_post_plrd_regs,
				ARRAY_SIZE(imx29x_startup_post_plrd_regs));
	if (ret) {
		dev_err(&client->dev,
			"%s failed to run startup post-PLRD sequence\n",
			__func__);
		return ret;
	}

	ret = imx29x_write_regs(imx29x, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	ret = imx29x_apply_output_overrides(imx29x, mode);
	if (ret)
		return ret;

	ret = imx29x_write_regs(imx29x, imx29x_standby_release_regs,
				ARRAY_SIZE(imx29x_standby_release_regs));
	if (ret) {
		dev_err(&client->dev, "%s failed to release standby\n",
			__func__);
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(imx29x->sd.ctrl_handler);
	if (ret)
		return ret;

	return imx29x_write_stream_on_sequence(imx29x);
}

static int imx29x_start_streaming_binned(struct imx29x *imx29x,
					 const struct imx29x_mode *mode)
{
	const struct imx29x_compatible_data *data = imx29x->compatible_data;
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	const struct imx29x_reg_list *reg_list = &mode->reg_list;
	int ret;

	ret = imx29x_write_regs(imx29x, imx29x_startup_pre_regs,
				ARRAY_SIZE(imx29x_startup_pre_regs));
	if (ret) {
		dev_err(&client->dev,
			"%s failed to run startup pre-sequence\n",
			__func__);
		return ret;
	}

	ret = imx29x_write_plrd_regs(imx29x);
	if (ret) {
		dev_err(&client->dev, "%s failed to set PLRD\n", __func__);
		return ret;
	}

	ret = imx29x_write_regs(imx29x, data->binned_common_regs,
				data->num_binned_common_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set common settings\n",
			__func__);
		return ret;
	}

	ret = imx29x_write_regs(imx29x, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(imx29x->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = imx29x_write_regs(imx29x, imx29x_standby_release_regs,
				ARRAY_SIZE(imx29x_standby_release_regs));
	if (ret) {
		dev_err(&client->dev, "%s failed to release binned standby\n",
			__func__);
		return ret;
	}

	return imx29x_write_stream_on_sequence(imx29x);
}

/* Start streaming */
static int imx29x_start_streaming(struct imx29x *imx29x)
{
	const struct imx29x_mode *mode = imx29x_get_active_mode(imx29x);

	imx29x->mode = mode;

	if (mode->use_output_overrides)
		return imx29x_start_streaming_output(imx29x, mode);

	return imx29x_start_streaming_binned(imx29x, mode);
}

/* Stop streaming */
static void imx29x_stop_streaming(struct imx29x *imx29x)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	int ret;

	/* set stream off register */
	ret = imx29x_write_reg_1byte(imx29x, IMX29X_REG_MODE_SELECT,
				     IMX29X_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx29x_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx29x *imx29x = to_imx29x(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if ((pad != IMAGE_PAD && pad != METADATA_PAD) ||
	    streams_mask != BIT_ULL(0))
		return -EINVAL;

	if (v4l2_subdev_is_streaming(sd))
		return 0;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	/*
	 * Apply default & customized values
	 * and then start streaming.
	 */
	ret = imx29x_start_streaming(imx29x);
	if (ret)
		goto err_rpm_put;

	imx29x->streaming = true;

	return ret;

err_rpm_put:
	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static int imx29x_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx29x *imx29x = to_imx29x(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if ((pad != IMAGE_PAD && pad != METADATA_PAD) ||
	    streams_mask != BIT_ULL(0))
		return -EINVAL;

	if (sd->enabled_pads & ~BIT_ULL(pad))
		return 0;

	imx29x_stop_streaming(imx29x);
	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	imx29x->streaming = false;

	return 0;
}

static int imx29x_set_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;

	if (enable)
		ret = v4l2_subdev_enable_streams(sd, IMAGE_PAD, BIT_ULL(0));
	else
		ret = v4l2_subdev_disable_streams(sd, IMAGE_PAD, BIT_ULL(0));

	return ret == -EALREADY ? 0 : ret;
}

/* Power/clock management functions */
static int imx29x_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx29x *imx29x = to_imx29x(sd);
	int ret;

	ret = regulator_bulk_enable(IMX29X_NUM_SUPPLIES,
				    imx29x->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx29x->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx29x->reset_gpio, IMX29X_XCLR_DEASSERTED);
	usleep_range(IMX29X_XCLR_DEASSERT_DELAY_US,
		     IMX29X_XCLR_DEASSERT_DELAY_US +
		     IMX29X_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(IMX29X_NUM_SUPPLIES, imx29x->supplies);
	return ret;
}

static int imx29x_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx29x *imx29x = to_imx29x(sd);

	gpiod_set_value_cansleep(imx29x->reset_gpio, IMX29X_XCLR_ASSERTED);
	regulator_bulk_disable(IMX29X_NUM_SUPPLIES, imx29x->supplies);
	clk_disable_unprepare(imx29x->xclk);
	return 0;
}

static int imx29x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx29x *imx29x = to_imx29x(sd);

	if (imx29x->streaming)
		imx29x_stop_streaming(imx29x);

	return 0;
}

static int imx29x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx29x *imx29x = to_imx29x(sd);
	int ret;

	if (imx29x->streaming) {
		ret = imx29x_start_streaming(imx29x);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx29x_stop_streaming(imx29x);
	imx29x->streaming = false;
	return ret;
}

static int imx29x_get_regulators(struct imx29x *imx29x)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	unsigned int i;

	for (i = 0; i < IMX29X_NUM_SUPPLIES; i++)
		imx29x->supplies[i].supply = imx29x_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX29X_NUM_SUPPLIES,
				       imx29x->supplies);
}

static int imx29x_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret = -EINVAL;
	int i;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	if (ret) {
		dev_err(dev, "could not parse endpoint\n");
		goto out_put;
	}

	if (ep.bus.mipi_csi2.num_data_lanes != IMX29X_NUM_DATA_LANES) {
		dev_err(dev, "only 4 data lanes are supported\n");
		ret = -EINVAL;
		goto out_free;
	}

	if (!ep.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property missing\n");
		ret = -EINVAL;
		goto out_free;
	}

	ret = -EINVAL;
	for (i = 0; i < ep.nr_of_link_frequencies; i++) {
		if (ep.link_frequencies[i] == IMX29X_LINK_FREQ) {
			ret = 0;
			break;
		}
	}

	if (ret) {
		dev_err(dev, "unsupported link frequency; expected %llu Hz\n",
			(unsigned long long)IMX29X_LINK_FREQ);
		ret = -EINVAL;
	}

out_free:
	v4l2_fwnode_endpoint_free(&ep);
out_put:
	fwnode_handle_put(endpoint);

	return ret;
}

/*
 * No documented readable chip-ID register; read a documented register to verify
 * I2C register access without claiming sensor identity.
 */
static int imx29x_check_i2c_readable(struct imx29x *imx29x)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	int ret;
	u32 val;

	ret = imx29x_read_reg(imx29x, IMX29X_REG_BLKLEVEL, 1, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read BLKLEVEL register (%d)\n",
			ret);
		return ret;
	}

	dev_dbg(&client->dev, "I2C register read OK (BLKLEVEL=0x%02x)\n",
		val);

	return 0;
}

static int imx29x_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	if (sel->pad != IMAGE_PAD)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx29x *imx29x = to_imx29x(sd);
		const struct v4l2_rect *crop;

		crop = __imx29x_get_pad_crop(imx29x, sd_state, sel->pad,
					     sel->which);
		if (!crop)
			return -EINVAL;

		sel->r = *crop;

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX29X_NATIVE_WIDTH;
		sel->r.height = IMX29X_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX29X_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX29X_PIXEL_ARRAY_TOP;
		sel->r.width = IMX29X_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX29X_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx29x_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct imx29x *imx29x = to_imx29x(sd);
	const struct imx29x_mode *mode = imx29x->mode;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = IMAGE_PAD,
		.format = {
			.code = imx29x->fmt_code,
			.width = mode->width,
			.height = mode->height,
		},
	};
	struct v4l2_mbus_framefmt *meta_fmt;
	struct v4l2_rect *crop;

	imx29x_set_pad_format(sd, state, &fmt);

	meta_fmt = v4l2_subdev_state_get_format(state, METADATA_PAD);
	meta_fmt->width = IMX29X_EMBEDDED_LINE_WIDTH;
	meta_fmt->height = IMX29X_NUM_EMBEDDED_LINES;
	meta_fmt->code = MEDIA_BUS_FMT_SENSOR_DATA;
	meta_fmt->field = V4L2_FIELD_NONE;

	crop = v4l2_subdev_state_get_crop(state, IMAGE_PAD);
	*crop = mode->crop;

	return 0;
}

static const struct v4l2_subdev_core_ops imx29x_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx29x_video_ops = {
	.s_stream = imx29x_set_stream,
};

static const struct v4l2_subdev_pad_ops imx29x_pad_ops = {
	.enum_mbus_code = imx29x_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx29x_set_pad_format,
	.get_selection = imx29x_get_selection,
	.enum_frame_size = imx29x_enum_frame_size,
	.enable_streams = imx29x_enable_streams,
	.disable_streams = imx29x_disable_streams,
};

static const struct v4l2_subdev_ops imx29x_subdev_ops = {
	.core = &imx29x_core_ops,
	.video = &imx29x_video_ops,
	.pad = &imx29x_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx29x_internal_ops = {
	.init_state = imx29x_init_state,
};

/* Initialize control handlers */
static int imx29x_init_controls(struct imx29x *imx29x)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx29x->sd);
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &imx29x->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 20);
	if (ret)
		return ret;

	mutex_init(&imx29x->mutex);
	ctrl_hdlr->lock = &imx29x->mutex;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx29x_set_framing_limits() call below.
	 */
	imx29x->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx29x_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(imx29x_link_freq_menu) - 1,
						   0, imx29x_link_freq_menu);
	/* By default, PIXEL_RATE is read only */
	imx29x->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx29x_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0xffff,
					       0xffff, 1,
					       0xffff);
	imx29x->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx29x_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xfffff, 1, 0);
	imx29x->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx29x_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);
	imx29x->test_pattern =
		v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx29x_ctrl_ops,
					     V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(imx29x_test_pattern_menu) - 1,
					     0, 0, imx29x_test_pattern_menu);

	imx29x->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx29x_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX29X_EXPOSURE_MIN,
					     IMX29X_EXPOSURE_MAX,
					     IMX29X_EXPOSURE_STEP,
					     IMX29X_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx29x_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX29X_ANA_GAIN_MIN, IMX29X_ANA_GAIN_MAX,
			  IMX29X_ANA_GAIN_STEP, IMX29X_ANA_GAIN_DEFAULT);

	if (imx29x->link_freq)
		imx29x->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (imx29x->pixel_rate)
		imx29x->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx29x_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx29x->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx29x_set_framing_limits(imx29x, imx29x->mode);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx29x->mutex);

	return ret;
}

static void imx29x_free_controls(struct imx29x *imx29x)
{
	v4l2_ctrl_handler_free(imx29x->sd.ctrl_handler);
	mutex_destroy(&imx29x->mutex);
}

static const struct imx29x_compatible_data imx294_compatible = {
	.model = IMX29X_MODEL_IMX294,
	.supports_quad_bayer_modes = true,
	.binned_common_regs = imx294_common_regs,
	.num_binned_common_regs = ARRAY_SIZE(imx294_common_regs),
};

static const struct imx29x_compatible_data imx492_compatible = {
	.model = IMX29X_MODEL_IMX492,
	.supports_mono = true,
	.supports_color_binned_modes = true,
	.binned_common_regs = imx492_binned_common_regs,
	.num_binned_common_regs = ARRAY_SIZE(imx492_binned_common_regs),
};

static const struct of_device_id imx294_dt_ids[] = {
	{ .compatible = "sony,imx294", .data = &imx294_compatible },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx294_dt_ids);

static const struct of_device_id imx492_dt_ids[] = {
	{ .compatible = "sony,imx492", .data = &imx492_compatible },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx492_dt_ids);

static int imx29x_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx29x *imx29x;
	unsigned int i;
	int ret;

	imx29x = devm_kzalloc(&client->dev, sizeof(*imx29x), GFP_KERNEL);
	if (!imx29x)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx29x->sd, client, &imx29x_subdev_ops);

	imx29x->compatible_data = device_get_match_data(dev);
	if (!imx29x->compatible_data)
		return -ENODEV;
	imx29x->mono = imx29x->compatible_data->supports_mono &&
		       (imx29x->compatible_data->mono ||
		       of_property_read_bool(dev->of_node, "mono-mode"));
	imx29x->quad_bayer_modes =
		imx29x->compatible_data->supports_quad_bayer_modes &&
		of_property_read_bool(dev->of_node, "quad-bayer-modes");
	imx29x->color_binned_modes =
		imx29x->compatible_data->supports_color_binned_modes &&
		of_property_read_bool(dev->of_node, "color-binned-modes");

	if (imx29x->mono)
		dev_info(dev, "mono mode enabled; expose Y10/Y12 formats\n");
	if (imx29x->quad_bayer_modes)
		dev_warn(dev,
			 "experimental quad Bayer modes enabled; no standard 4x4 CFA media-bus code\n");
	if (imx29x->color_binned_modes)
		dev_info(dev, "color binned mode enabled; expose 12-bit binned Bayer mode\n");

	ret = imx29x_check_hwcfg(dev);
	if (ret)
		return ret;

	/* Get system clock (xclk) */
	imx29x->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx29x->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx29x->xclk);
	}

	imx29x->xclk_freq = clk_get_rate(imx29x->xclk);
	for (i = 0; i < ARRAY_SIZE(imx29x_plrd_setups); i++) {
		if (imx29x_plrd_setups[i].xclk_freq == imx29x->xclk_freq) {
			imx29x->plrd_setup = &imx29x_plrd_setups[i];
			break;
		}
	}
	if (!imx29x->plrd_setup)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported XCLK %u Hz\n",
				     imx29x->xclk_freq);

	dev_info(dev, "XCLK %u Hz selected\n", imx29x->xclk_freq);

	ret = imx29x_get_regulators(imx29x);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx29x->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx29x->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx29x->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = imx29x_power_on(dev);
	if (ret)
		return ret;

	ret = imx29x_check_i2c_readable(imx29x);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx29x_set_default_format(imx29x);

	/* Enable runtime PM and let autosuspend turn the device off when idle. */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, IMX29X_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx29x_init_controls(imx29x);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx29x->sd.internal_ops = &imx29x_internal_ops;
	imx29x->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx29x->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx29x->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx29x->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx29x->sd.entity, NUM_PADS, imx29x->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	imx29x->sd.state_lock = imx29x->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx29x->sd);
	if (ret) {
		dev_err(dev, "failed to initialize subdev state: %d\n", ret);
		goto error_media_entity;
	}
	imx29x->active_state_initialized = true;

	ret = v4l2_async_register_subdev_sensor(&imx29x->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_subdev_cleanup;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

error_subdev_cleanup:
	imx29x->active_state_initialized = false;
	v4l2_subdev_cleanup(&imx29x->sd);

error_media_entity:
	media_entity_cleanup(&imx29x->sd.entity);

error_handler_free:
	imx29x_free_controls(imx29x);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx29x_power_off(&client->dev);

	return ret;
}

static void imx29x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx29x *imx29x = to_imx29x(sd);

	v4l2_async_unregister_subdev(sd);
	imx29x->active_state_initialized = false;
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	imx29x_free_controls(imx29x);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx29x_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops imx29x_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(imx29x_suspend, imx29x_resume)
	RUNTIME_PM_OPS(imx29x_power_off, imx29x_power_on, NULL)
};

static struct i2c_driver imx294_i2c_driver = {
	.driver = {
		.name = "imx294",
		.of_match_table	= imx294_dt_ids,
		.pm = pm_ptr(&imx29x_pm_ops),
	},
	.probe = imx29x_probe,
	.remove = imx29x_remove,
};

static struct i2c_driver imx492_i2c_driver = {
	.driver = {
		.name = "imx492",
		.of_match_table	= imx492_dt_ids,
		.pm = pm_ptr(&imx29x_pm_ops),
	},
	.probe = imx29x_probe,
	.remove = imx29x_remove,
};

static int __init imx29x_init(void)
{
	int ret;

	ret = i2c_add_driver(&imx294_i2c_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&imx492_i2c_driver);
	if (ret)
		i2c_del_driver(&imx294_i2c_driver);

	return ret;
}
module_init(imx29x_init);

static void __exit imx29x_exit(void)
{
	i2c_del_driver(&imx492_i2c_driver);
	i2c_del_driver(&imx294_i2c_driver);
}
module_exit(imx29x_exit);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_DESCRIPTION("Sony IMX294/IMX492 sensor driver");
MODULE_LICENSE("GPL");
