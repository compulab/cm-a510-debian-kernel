/*
 *  linux/arch/arm/mach-dove/idt5v49ee503.c
 *  EEPROM programmable clock generator (IDT5V49EE503) driver
 */

/* TODO: move to Dove's clock.c ? */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <asm/clkdev.h>
#include <asm/string.h>
#include <asm/div64.h>
#include "idt5v49ee503.h"
#include "clock.h"

#define IDT5V49EE503_BUS_ADDR			0x6A /* 7'b1101010 */

#define IDT5V49EE503_SW_MODE_CTL_REG		0x0
#define IDT5V49EE503_CFG_SEL_REG		0x1
#define IDT5V49EE503_OUT_ENA_REG		0x2
#define IDT5V49EE503_OUT_SUSPEND_REG		0x3
#define IDT5V49EE503_PLL_SUSPEND_REG		0x4
#define IDT5V49EE503_XTCLKSEL_REG		0x5
#define IDT5V49EE503_XTAL_REG			0x7

/********************/
/* LOCLA DATA TYPES */
/********************/
/* Clock configuration ID */
typedef enum
{
	IDT_CLK_CFG_0 = 0,
	IDT_CLK_CFG_1 = 1,
	IDT_CLK_CFG_2 = 2,
	IDT_CLK_CFG_3 = 3,
	IDT_CLK_CFG_4 = 4,
	IDT_CLK_CFG_5 = 5,
	IDT_CLK_CFG_NUM

} idt_clock_cfg_id_t;

/* Clock output divider */
typedef enum
{
	IDT_OUT_DIV1 = 0,
	IDT_OUT_DIV2,
	IDT_OUT_DIV3,
	IDT_OUT_DIV6,
	IDT_OUT_DIV_NUM,
	IDT_OUT_DIV_INV = IDT_OUT_DIV_NUM

} idt_out_div_t;


typedef struct _idt_clock_cfg_t
{
	int			bus_id;		/* i2c bus/adapter ID to use with i2c_get_adapter() */\
	int			clk_src_clkin;  /* clock source is CLKIN(0) or XTAL/REFIN(0) */
	idt_clock_id_t		clock_id;	/* clock ID to configure (PLL0-PLL3) */
	idt_output_id_t		out_id;		/* output pin where PLL + output divider are connected */
	idt_clock_cfg_id_t	cfg_id;		/* clock internal configuration ID */
	unsigned int		cfg_act;	/* when !=0 this configuration is not just stored */
						/* in the clock EEPROM, but also set as active  and */
						/* PLL + output are enabled */
} idt_clock_cfg_t;

/* Clock source IDs (MUX names) */
typedef enum
{
	IDT_CLK_SRC_ID_PRIM = 0,
	IDT_CLK_SRC_ID_0,
	IDT_CLK_SRC_ID_1,
	IDT_CLK_SRC_ID_2,
	IDT_CLK_SRC_ID_3,
	IDT_CLK_SRC_ID_6,
	IDT_CLK_SRC_ID_NUM

} idt_clock_src_id_t;

/* Clock sources (MUX selection) */
typedef enum
{
	IDT_CLK_SRC_DIV1 = 0x0,
	IDT_CLK_SRC_DIV3 = 0x1,
	IDT_CLK_SRC_REFIN= 0x2,
	IDT_CLK_SRC_RSRVD= 0x3,
	IDT_CLK_SRC_PLL0 = 0x4,
	IDT_CLK_SRC_PLL1 = 0x5,
	IDT_CLK_SRC_PLL2 = 0x6,
	IDT_CLK_SRC_PLL3 = 0x7

} idt_clock_src_t;

/* Clock sources (MUX selection) */
typedef enum
{
	IDT_PRM_CLK_CRYSTAL = 0,
	IDT_PRM_CLK_CLKIN   = 1
} idt_prim_clock_src_t;

/* Clock registers for variour PLL parameters */
typedef struct _idt_pll_reg_set_t
{
	unsigned char N_lreg;	/* feedback divider LSB N[7:0]  */
	unsigned char N_mreg;	/* feedback divider MSB N[11:8] */
	unsigned char N_mlshft;	/* N[11:8] nibble left-shift value */
	unsigned char D_reg;	/* reference divider D[6:0]  */
	unsigned char IP_reg;	/* Loop Filter  */
} idt_pll_reg_set_t;

/* Clock registers for MUX source configuration */
typedef struct _idt_src_reg_set_t
{
	unsigned char	lreg;		/* SRCx[1:0] or SRCx[2:0] register address (LSB) */
	unsigned char	lshift;		/* SRCx[1:0] or SRCx[2:0] data shift */
	unsigned char   lmask;		/* SRCx[1:0] or SRCx[2:0] data mask (width) */
	unsigned char	mreg;		/* SRCx[2] register address (MSB, only is separated from [1:0]) */
	unsigned char	mshift;		/* SRCx[2] data shift (MSB, only is separated from [1:0]) */
} idt_src_reg_set_t;

/* Fixed frequencies table entry */
typedef struct _idt_freq_ten_t
{
	unsigned long	f_out;	/* output frequency Hz */
	unsigned long 	f_in;	/* input (reference) freqency Hz */
	unsigned int	n;	/* N (feedback divider) is 12 bits wide in HW */
	unsigned int	d;	/* D (reference divider) is 7 bits wide in HW */
	unsigned int	odiv;	/* ODIV, total output divider 7 and 1 bits in HW */
	
} idt_freq_ten_t;

typedef struct _idt_drv_info_t
{
	struct i2c_client 	*i2c_cl;
} idt_drv_info_t;

/***************/
/* STATIC DATA */
/***************/
idt_drv_info_t *idt_drv_g;

/* Registers addresses in clock programming tables */
static idt_pll_reg_set_t idt5v49ee503_pll_regs[IDT_CLK_CFG_NUM][IDT_PLL_NUM] = {
	/*  N[7:0]  N[11:8]  Nsh  D[6:0] IP */
	{ { 0x18,   0x1C,    0,   0x10    , 0xC}, /* PLL0 */
	  { 0x30,   0x34,    0,   0x28    , 0x24}, /* PLL1 */
	  { 0x48,   0x4C,    0,   0x40    , 0x3C}, /* PLL2 */
	  { 0x64,   0x34,    4,   0x5C    , 0x58}  /* PLL3 */
	}, /* CFG0 */
	{ { 0x19,   0x1D,    0,   0x11    , 0xD}, /* PLL0 */
	  { 0x31,   0x35,    0,   0x29    , 0x25}, /* PLL1 */
	  { 0x49,   0x4D,    0,   0x41    , 0x3D}, /* PLL2 */
	  { 0x65,   0x35,    4,   0x5D    , 0x59}  /* PLL3 */
	}, /* CFG1 */
	{ { 0x1A,   0x1E,    0,   0x12    , 0xE}, /* PLL0 */
	  { 0x32,   0x36,    0,   0x2A    , 0x26}, /* PLL1 */
	  { 0x4A,   0x4E,    0,   0x42    , 0x3E}, /* PLL2 */
	  { 0x66,   0x36,    4,   0x5E    , 0x5A }  /* PLL3 */
	}, /* CFG2 */
	{ { 0x1B,   0x1F,    0,   0x13    , 0xF}, /* PLL0 */
	  { 0x33,   0x37,    0,   0x2B    , 0x27}, /* PLL1 */
	  { 0x4B,   0x4F,    0,   0x43    , 0x3F }, /* PLL2 */
	  { 0x67,   0x37,    4,   0x5F    , 0x5B}  /* PLL3 */
	}, /* CFG3 */
	{ { 0x16,   0x20,    0,   0x14    , 0xA}, /* PLL0 */
	  { 0x2E,   0x38,    0,   0x2C    , 0x22}, /* PLL1 */
	  { 0x46,   0x50,    0,   0x44    , 0x3A}, /* PLL2 */
	  { 0x62,   0x38,    4,   0x60    , 0x56}  /* PLL3 */
	}, /* CFG4 */
	{ { 0x17,   0x21,    0,   0x15    , 0xB}, /* PLL0 */
	  { 0x2F,   0x39,    0,   0x2D    , 0x23}, /* PLL1 */
	  { 0x47,   0x51,    0,   0x45    , 0x3B}, /* PLL2 */
	  { 0x63,   0x39,    4,   0x61    , 0x57}  /* PLL3 */
	}  /* CFG5 */
};

/* register addresses for output dividers */
static unsigned int idt5v49ee503_div_regs[IDT_CLK_CFG_NUM][IDT_OUT_DIV_NUM] = {
	/* DIV1  DIV2  DIV3  DIV6 */
	{  0x88, 0x90, 0x94, 0xA8 }, /* CFG0 */
	{  0x89, 0x91, 0x95, 0xA9 }, /* CFG1 */
	{  0x8A, 0x92, 0x96, 0xAA }, /* CFG2 */
	{  0x8B, 0x93, 0x97, 0xAB }, /* CFG3 */
	{  0x8C, 0x8E, 0x98, 0xA6 }, /* CFG4 */
	{  0x8D, 0x8F, 0x99, 0xA7 }  /* CFG5 */
};

/* Registers addresses for source selectors (0xFF - not valid) */
static idt_src_reg_set_t idt5v49ee503_src_regs[IDT_CLK_CFG_NUM][IDT_CLK_SRC_ID_NUM] = {
	/*  lreg  lshift  lmask mreg  mshift */
	{ { 0xC0, 0x0,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_PRIM */
	  { 0xC0, 0x4,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_0 */
	  { 0xC0, 0x6,    0x3,  0xC4, 0x0  }, /* IDT_CLK_SRC_ID_1 */
	  { 0xC4, 0x1,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_2 */
	  { 0xC4, 0x4,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_3 */
	  { 0xCC, 0x5,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_6 */
	}, /* CFG0 */
	{ { 0xC1, 0x0,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_PRIM */
	  { 0xC1, 0x4,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_0 */
	  { 0xC1, 0x6,    0x3,  0xC5, 0x0  }, /* IDT_CLK_SRC_ID_1 */
	  { 0xC5, 0x1,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_2 */
	  { 0xC5, 0x4,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_3 */
	  { 0xCD, 0x5,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_6 */
	}, /* CFG1 */
	{ { 0xC2, 0x0,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_PRIM */
	  { 0xC2, 0x4,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_0 */
	  { 0xC2, 0x6,    0x3,  0xC6, 0x0  }, /* IDT_CLK_SRC_ID_1 */
	  { 0xC6, 0x1,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_2 */
	  { 0xC6, 0x4,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_3 */
	  { 0xCE, 0x5,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_6 */
	}, /* CFG2 */
	{ { 0xC3, 0x0,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_PRIM */
	  { 0xC3, 0x4,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_0 */
	  { 0xC3, 0x6,    0x3,  0xC7, 0x0  }, /* IDT_CLK_SRC_ID_1 */
	  { 0xC7, 0x1,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_2 */
	  { 0xC7, 0x4,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_3 */
	  { 0xCF, 0x5,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_6 */
	}, /* CFG3 */
	{ { 0xBE, 0x0,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_PRIM */
	  { 0xBE, 0x4,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_0 */
	  { 0xBE, 0x6,    0x3,  0xC8, 0x0  }, /* IDT_CLK_SRC_ID_1 */
	  { 0xC8, 0x1,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_2 */
	  { 0xC8, 0x4,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_3 */
	  { 0xCA, 0x5,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_6 */
	}, /* CFG4 */
	{ { 0xBF, 0x0,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_PRIM */
	  { 0xBF, 0x4,    0x3,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_0 */
	  { 0xBF, 0x6,    0x3,  0xC9, 0x0  }, /* IDT_CLK_SRC_ID_1 */
	  { 0xC9, 0x1,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_2 */
	  { 0xC9, 0x4,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_3 */
	  { 0xCB, 0x5,    0x7,  0xFF, 0x0  }, /* IDT_CLK_SRC_ID_6 */
	}  /* CFG5 */
};

/* The following equation is used for parameters computation:
   Fout = (Fin * (M/D))/ODIV
   Where for PLL0:
	M = 2 * N + A + 1 (for A > 0)
	M = 2 * N (for A = 0)
   For PLL1, PLL2, PLL3:
	M = N
*/
/* Supported fixed frequencies table */
static idt_freq_ten_t idt5v49ee503_freq_tbl[] = {
	/*f_out      f_in      n    d   odiv */
	{ 148500000, 25000000, 891, 25,  6},
	{  64109000, 25000000, 877, 19, 18},
	{ 108883000, 25000000, 662, 19, 8},
	{ 160963000, 25000000, 734, 19,  6},
	{ 136358000, 25000000, 480, 11,  8},
	{         0,        0,   0,  0,  0},
};

static idt_output_id_t clock0_out_id = IDT_OUT_ID_2;
static idt_clock_id_t clock0_pll_id = IDT_PLL_1;
static idt_output_id_t clock1_out_id = IDT_OUT_ID_3;
static idt_clock_id_t clock1_pll_id = IDT_PLL_2;

#define FIN 25000000ULL
static int calc_freq_div(unsigned long long freq, idt_freq_ten_t *entry)
{
	unsigned int n, d, odiv;
	unsigned long long max_diff, diff;
	uint64_t tmp;
	uint32_t factor;

	max_diff = freq * 1;
	do_div(max_diff, 10000);
	for (odiv = 4; odiv < 256; odiv+=2)
		for (d = odiv/*1*/; d < 127; d++) {
			tmp = freq * odiv * d;
			do_div(tmp, FIN);
			if (tmp < 1 || tmp > 4095)
				continue;
			n = (unsigned int) tmp;
			/* check if this n gives accurate Fout */
			tmp = FIN * n;
			factor = d * odiv;
			do_div(tmp, factor);
			if (tmp > freq)
				diff = tmp - freq;
			else
				diff = freq - tmp;
			
			if (diff < max_diff) {
				printk(KERN_DEBUG "dividers found for Fout = %lld. n=%d d=%d odiv %d\n",
				       freq, n, d, odiv);
				printk(KERN_DEBUG "diff = %lld. max diff %lld freq %lld tmp %lld\n",
				       diff, max_diff, freq, tmp);
				entry->odiv = odiv;
				entry->d = d;
				entry->n = n;
				return 0;
			}
		}

	printk("error: can't find dividers for Fout = %lld\n", freq);

	return -1;
}

/****************************************************************************** 
   Write I2C register
   Input:
 	driver - driver control structure
 	addr   - register address
 	val    - register value
   Output:
      	NONE
 
   Returns:
	0 on success or error value
*******************************************************************************/
static int idt5v49ee503_write_reg (idt_drv_info_t *idt_drv,
				   unsigned char  addr,
				   unsigned char  val)
{
 	unsigned char buf[3] = {0, addr, val}; /* fist byte is command (0) */
	int rc = i2c_master_send(idt_drv->i2c_cl, buf, 3);
//	printk("%s: addr %x val %x\n", __func__, addr, val);
	if (rc != 3) {
		printk(KERN_ERR "idt: failed to write 0x%x to register 0x%x (rc %d)\n", 
		       val, addr, rc);
		return -EIO;
	}
	return 0;

} /* end of idt5v49ee503_read_reg */

/****************************************************************************** 
   Read I2C register
   Input:
 	idt_drv - driver control structure
 	addr   - register address
 	val    - register value
   Output:
      	NONE
 
   Returns:
	0 on success or error value
*******************************************************************************/
static int idt5v49ee503_read_reg (idt_drv_info_t *idt_drv,
				  unsigned char  addr,
				  unsigned char  *val)
{

	int           rval;
	unsigned char buf[3] = {0, addr, 0}; /* fist byte for WRITE is command (0)
	                                        or ID byte for READ (ignored) */
	struct i2c_msg msg[2];

	msg[0].addr = idt_drv->i2c_cl->addr;
	msg[0].flags = 0;
	msg[0].buf = buf;
	msg[0].len = 2;

	msg[1].addr = idt_drv->i2c_cl->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 2;

	rval = i2c_transfer(idt_drv->i2c_cl->adapter, &msg[0], 2);
	*val = buf[1];
	return (rval != 2);

} /* end of idt5v49ee503_read_reg */

/****************************************************************************** 
   Read, modify and write back I2C register
   Input:
 	idt_drv - driver control structure
 	addr   - register address
 	fval   - register's field value
 	shift  - number of bits to shift the field
 	mask   - the field's mask (before shifting)
   Output:
      	NONE
 
   Returns:
	0 on success or error value
*******************************************************************************/
static int idt5v49ee503_mod_reg (idt_drv_info_t *idt_drv,
				 unsigned char  addr,
				 unsigned char  fval,
				 unsigned char  shift,
				 unsigned char	mask)
{
	int           rval;
	unsigned char val = 0;

	if ((rval = idt5v49ee503_read_reg(idt_drv, addr, &val)) != 0)
		return rval;

	val &= ~(mask << shift); 
	val |= (fval & mask) <<	shift;

	return idt5v49ee503_write_reg(idt_drv, addr, val);

} /* end of idt5v49ee503_mod_reg */


/****************************************************************************** 
   Connect PLL to specific output pin and output divider
   Input:
 	idt_drv - driver control structure
 	clock_cfg - clock configuration
   Output:
      	NONE
 
   Returns:
	Output divider ID
*******************************************************************************/
static idt_out_div_t idt5v49ee503_setup_output (idt_drv_info_t	*idt_drv,
						idt_clock_cfg_t	*clock_cfg)
{
	idt_clock_src_id_t	clk_src_id[2]; /* maximum 2 clock sources to config */
	idt_clock_src_t		clk_src[2];
	idt_out_div_t		divider; /* currently only one level division support */
	idt_src_reg_set_t	*reg_set = idt5v49ee503_src_regs[clock_cfg->cfg_id];
	int			i;

	if (idt_drv == NULL)
		return IDT_OUT_DIV_INV;

	/* second source MUX will usually not be used (excepting OUT0) */
	clk_src_id[1] = IDT_OUT_ID_INV;
	
	/* find out MUX ID and divider based on output pin - see datasheet page 2 */
	switch (clock_cfg->out_id)
	{
	case IDT_OUT_ID_0:
		clk_src_id[0] = IDT_CLK_SRC_ID_1; 
		clk_src_id[1] = IDT_CLK_SRC_ID_0;
		clk_src[1] = IDT_CLK_SRC_DIV1; /* always use DIV1 for OUT0 */
		divider = IDT_OUT_DIV1;
		break;

	case IDT_OUT_ID_1:
		clk_src_id[0] = IDT_CLK_SRC_ID_1;
		divider = IDT_OUT_DIV1;
		break;

	case IDT_OUT_ID_2:
		clk_src_id[0] = IDT_CLK_SRC_ID_2;
		divider = IDT_OUT_DIV2;
		break;

	case IDT_OUT_ID_3:
		clk_src_id[0] = IDT_CLK_SRC_ID_3;
		divider = IDT_OUT_DIV3;
		break;

	case IDT_OUT_ID_6:
		if (clock_cfg->clock_id != IDT_PLL_1)
			return IDT_OUT_DIV_INV;
		clk_src_id[0] = IDT_CLK_SRC_ID_6;
		divider = IDT_OUT_DIV6;
		break;
	default:
		return IDT_OUT_DIV_INV;

	} /* switch(clock_cfg->out_id) */

	/* find out MUX input selection based on PLL ID */
	switch (clock_cfg->clock_id)
	{
	case IDT_PLL_0:
		clk_src[0] = IDT_CLK_SRC_PLL0;
		break;

	case IDT_PLL_1:
		clk_src[0] = IDT_CLK_SRC_PLL1;
		break;

	case IDT_PLL_2:
		clk_src[0] = IDT_CLK_SRC_PLL2;
		break;

	case IDT_PLL_3:
		clk_src[0] = IDT_CLK_SRC_PLL3;
		break;

	default:
		return IDT_OUT_DIV_INV;

	} /* switch (clock_cfg->clock_id) */

	/* Write SRCx MUX configuration to clock registers */
	for (i = 0; i < 2; i++)
	{		
		if (clk_src_id[i] == IDT_OUT_ID_INV)
			continue;

		if (idt5v49ee503_mod_reg(idt_drv,
				         reg_set[clk_src_id[i]].lreg,
				         clk_src[i],
				         reg_set[clk_src_id[i]].lshift,
				         reg_set[clk_src_id[i]].lmask) != 0)
			return IDT_OUT_DIV_INV;

		/* MSB bit is located in other than LSB register? */
		if (reg_set[clk_src_id[i]].mreg != 0xFF)
		{
			if (idt5v49ee503_mod_reg(idt_drv,
						 reg_set[clk_src_id[i]].mreg,
						 clk_src[i],
						 reg_set[clk_src_id[i]].mshift,
						 0x1) != 0)
				return IDT_OUT_DIV_INV;
		}
	}	

	/* Configure primary clock source */
	if (idt5v49ee503_mod_reg(idt_drv,
				 reg_set[IDT_CLK_SRC_ID_PRIM].lreg,
				 clock_cfg->clk_src_clkin != 0 ? 
					IDT_PRM_CLK_CLKIN : IDT_PRM_CLK_CRYSTAL,
				 reg_set[IDT_CLK_SRC_ID_PRIM].lshift,
				 reg_set[IDT_CLK_SRC_ID_PRIM].lmask) != 0)
		return IDT_OUT_DIV_INV;
	return divider;

} /* end of idt5v49ee503_setup_output */

/****************************************************************************** 
   Write PLL parameters to clock HW according to specific frequency index
   Input:
 	freq_idx  - index to fixed frequencies table
 	clock_cfg - clock configuration
   Output:
      	NONE
 
   Returns:
	0	- success,
*******************************************************************************/
static int idt5v49ee503_write_cfg (idt_drv_info_t *idt_drv,
				   idt_freq_ten_t *freq,
				   idt_clock_cfg_t *clock_cfg)
{
	idt_out_div_t		div_id; /* output divider */
	idt_pll_reg_set_t 	*pll_regs = idt5v49ee503_pll_regs[clock_cfg->cfg_id];
	unsigned int 		*div_regs = idt5v49ee503_div_regs[clock_cfg->cfg_id];
	unsigned int		div_val, n_lsb, n_msb = 0;


	/* Setup SRCx MUXes and primary clock source first */
	if ((div_id = idt5v49ee503_setup_output(idt_drv, clock_cfg)) == IDT_OUT_DIV_INV)
		return -EIO;
	/* Configure clock parameters */
	if (idt5v49ee503_write_reg(idt_drv,
				 pll_regs[clock_cfg->clock_id].D_reg,
				 freq->d) != 0)
		return -EIO;

	if (clock_cfg->clock_id == IDT_PLL_0)
	{ /* M = 2 * N (A=0) */
		n_lsb = (freq->n >> 1) & 0xFF;
		n_msb = (freq->n >> 9) & 0xF;
	} /* M = 2 * N (A=0) */
	else
	{ /* M =  N */
		n_lsb = freq->n & 0xFF;
		n_msb = (freq->n >> 8) & 0xF;
	} /* M =  N */

	if (idt5v49ee503_write_reg(idt_drv,
				   pll_regs[clock_cfg->clock_id].N_lreg,
				   n_lsb) != 0)
		return -EIO;

	/* MSP part of N parameter shares register space with others */
	if (idt5v49ee503_mod_reg(idt_drv,
				 pll_regs[clock_cfg->clock_id].N_mreg,
				 n_msb,
				 pll_regs[clock_cfg->clock_id].N_mlshft,
				 0xF) != 0)
		return -EIO;

	/* Configure output divider */
	if (freq->odiv == 1)
		div_val = 0xFF;
	else if (freq->odiv == 2)
		div_val = 0;
	else /* ODIV = (Q[6:0] + 2) * 2 */
		div_val = ((freq->odiv >> 1) - 2) | 0x80; 

	if (idt5v49ee503_write_reg(idt_drv,
				   div_regs[div_id],
				   div_val) != 0)
		return -EIO;

	/* Configure loop filter */
	if (idt5v49ee503_write_reg(idt_drv,
				   pll_regs[clock_cfg->clock_id].IP_reg,
				   0x10) != 0)
		return -EIO;

	return 0;

} /* end of idt5v49ee503_write_cfg */

/****************************************************************************** 
   activate specific clock configuration
   Input:
	idt_drv	    - driver data
	clock_cfg   - clock configuration
   Output:
      	NONE
 
   Returns:
 	0      - success,
 	-EPERM - the clock is not initialized
 	-EINVAL - bad configuration ID
*******************************************************************************/
int idt5v49ee503_set_act_cfg (idt_drv_info_t *idt_drv, idt_clock_cfg_t *clock_cfg)
{
	if (clock_cfg->cfg_id >= IDT_CLK_CFG_NUM)
		return -EINVAL;

	/* Activate configuration */
	return idt5v49ee503_write_reg(idt_drv,
				      IDT5V49EE503_CFG_SEL_REG,
				      clock_cfg->cfg_id);

} /* end of idt5v49ee503_act_cfg */

/****************************************************************************** 
   Save all clock registers to EEPROM
   Input:
 	idt_drv - driver data
   Output:
      	NONE
 
   Returns:
 	0      - success,
*******************************************************************************/
int idt5v49ee503_save_regs (idt_drv_info_t *idt_drv)
{
	/* PROGSAVE command (1) */
	char		cmd = 0x1;

	return i2c_master_send(idt_drv->i2c_cl, &cmd, 1);

} /* end of idt5v49ee503_save_regs */

/****************************************************************************** 
   Save all clock registers to EEPROM
   Input:
 	idt_drv - driver data
   Output:
      	NONE
 
   Returns:
 	0      - success,
*******************************************************************************/
int idt5v49ee503_restore_regs (idt_drv_info_t *idt_drv)
{
	/* PROGRESTORE command (2) */
	char		cmd = 0x2;

	return i2c_master_send(idt_drv->i2c_cl, &cmd, 1);

} /* end of idt5v49ee503_save_regs */

/****************************************************************************** 
   Enable or disable PLL
   Input:
 	idt_drv  - driver data
 	pll_id  - PLL ID
 	enable  - 1 for enable, 0 for disable
   Output:
      	NONE
 
   Returns:
 	0      - success,
*******************************************************************************/
int idt5v49ee503_pll_enable (idt_drv_info_t *idt_drv,
			     idt_clock_id_t pll_id,
			     int            enable)
{

	/* Enable specific PLL */
	return idt5v49ee503_mod_reg(idt_drv,
				    IDT5V49EE503_PLL_SUSPEND_REG,
				    enable != 0 ? 1 : 0,
				    pll_id,
				    0x1);

} /* end of idt5v49ee503_pll_enable */

/****************************************************************************** 
   Enable or disable clock Output
   Input:
 	idt_drv  - driver data
 	out_id  - output ID
 	enable  - 1 for enable, 0 for disable
   Output:
      	NONE
 
   Returns:
 	0      - success,
*******************************************************************************/
int idt5v49ee503_out_enable (idt_drv_info_t *idt_drv,
			     idt_output_id_t out_id,
			     int             enable)
{
	/* Enable specific PLL */
	return idt5v49ee503_mod_reg(idt_drv,
				    IDT5V49EE503_OUT_SUSPEND_REG,
				    enable != 0 ? 1 : 0,
				    out_id,
				    0x1);

} /* end of idt5v49ee503_out_enable */

/****************************************************************************** 
   Enable or disable clock Output
   Input:
 	idr_drv - driver data
 	sw_ctl  - 1 - SW control, 0 - HW control
   Output:
      	NONE
 
   Returns:
 	0      - success,
*******************************************************************************/
int idt5v49ee503_sw_ctrl (idt_drv_info_t *idt_drv,
			  int  sw_ctl)
{
	/* Enable specific PLL */
	return idt5v49ee503_write_reg(idt_drv,
				      IDT5V49EE503_SW_MODE_CTL_REG,
				      sw_ctl != 0 ? 1 : 0);

} /* end of idt5v49ee503_sw_ctrl */

/****************************************************************************** 
   Program clock with specific frequency from known fixed freqiencies
   Input:
 	freq_set - frequency to be programmed
 		(only a number of fixed freqiencies is supported)
 	clock_cfg - clock configuration
   Output:
      	NONE
 
   Returns:
	0	- success,
	-EINVAL	- unsupported frequency
*******************************************************************************/
int idt5v49ee503_set (idt_drv_info_t *idt_drv,
		      unsigned long	freq_set, 
		      idt_clock_cfg_t	*clock_cfg)
{
	int	i;
	int	rval;

	for (i = 0; idt5v49ee503_freq_tbl[i].f_out != 0; i++)
	{
		if (idt5v49ee503_freq_tbl[i].f_out == freq_set)
		{

			rval = idt5v49ee503_write_cfg(idt_drv, &idt5v49ee503_freq_tbl[i], clock_cfg);

			if ((rval == 0) && (clock_cfg->cfg_act))
			{ /* activate clock and config */

				rval = idt5v49ee503_set_act_cfg(idt_drv, clock_cfg);

				/* enable PLL core */
				if (rval == 0)
				{	
					rval = idt5v49ee503_pll_enable(idt_drv,
								       clock_cfg->clock_id,
								       1);
				}
				/* enable clock output pin */
				if (rval == 0)
				{	
					rval = idt5v49ee503_out_enable(idt_drv,
								       clock_cfg->out_id,
								       1);
				}
				/* use SW control (i.e. values stored in clock registers) */
				return idt5v49ee503_sw_ctrl(idt_drv, 1);

			} /* activate clock and config */
		}
	}

	return -EINVAL;

} /* end of idt5v49ee503_set */
#if 1
/****************************************************************************** 
   Program clock with specific frequency
   Input:
 	freq - frequency to be programmed
 	clock_cfg - clock configuration
   Output:
      	NONE
 
   Returns:
	0	- success,
	-EINVAL	- unsupported frequency
*******************************************************************************/
int idt5v49ee503_set_freq (idt_drv_info_t *idt_drv,
			   unsigned long long	freq, 
			   idt_clock_cfg_t	*clock_cfg)
{
	int	rval;
	idt_freq_ten_t entry;
	
	if (calc_freq_div(freq, &entry) < -1)
		return -1;

	rval = idt5v49ee503_write_cfg(idt_drv, &entry, clock_cfg);

	if ((rval == 0) && (clock_cfg->cfg_act))
	{ /* activate clock and config */
		
		rval = idt5v49ee503_set_act_cfg(idt_drv, clock_cfg);
		/* enable PLL core */
		if (rval == 0)
		{	
			rval = idt5v49ee503_pll_enable(idt_drv,
						       clock_cfg->clock_id,
						       1);
		}
				/* enable clock output pin */
		if (rval == 0)
		{	
			rval = idt5v49ee503_out_enable(idt_drv,
						       clock_cfg->out_id,
						       1);
		}
		/* use SW control (i.e. values stored in clock registers) */
		return idt5v49ee503_sw_ctrl(idt_drv, 1);
		
	} /* activate clock and config */

	return -EINVAL;

} /* end of idt5v49ee503_set */
#endif
/*
 * Generic counter attributes
 */
static ssize_t idt5v49ee503_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	dev_dbg(dev, "idt5v49ee503_show() called on %s\n", attr->attr.name);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "%d\n", 0);
}

static ssize_t idt5v49ee503_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	idt_drv_info_t *idt_drv = i2c_get_clientdata(client);
	idt_clock_cfg_t *cfg;
	char *endp;
	u64 val;

	dev_dbg(dev, "idt5v49ee503_store() called on %s\n", attr->attr.name);

	/* Decode input */
	val = simple_strtoull(buf, &endp, 0);
	if (buf == endp) {
		dev_dbg(dev, "input string not a number\n");
		return -EINVAL;
	}
	cfg = kzalloc(sizeof(idt_clock_cfg_t), GFP_KERNEL);
	if (cfg == NULL) {
		dev_dbg(dev, "failed to allocate memory\n");
		return -EINVAL;
	}
	cfg->clk_src_clkin = IDT_PRM_CLK_CRYSTAL;
	cfg->clock_id = clock0_pll_id;
	cfg->out_id = clock0_out_id;
	cfg->cfg_id = IDT_CLK_CFG_0;
	cfg->cfg_act = 1;
		
	if (idt5v49ee503_set_freq(idt_drv, val, cfg)) {
		dev_err(dev, "failed to set clock to %llu\n", val);
		count = -EIO;
		goto exit;
	}
	return count;
exit:
	kfree(cfg);
	return count;
}

static ssize_t idt5v49ee503_store2(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	idt_drv_info_t *idt_drv = i2c_get_clientdata(client);
	idt_clock_cfg_t *cfg;
	char *endp;
	u64 val;

	dev_dbg(dev, "idt5v49ee503_store() called on %s\n", attr->attr.name);

	/* Decode input */
	val = simple_strtoull(buf, &endp, 0);
	if (buf == endp) {
		dev_dbg(dev, "input string not a number\n");
		return -EINVAL;
	}
	cfg = kzalloc(sizeof(idt_clock_cfg_t), GFP_KERNEL);
	if (cfg == NULL) {
		dev_dbg(dev, "failed to allocate memory\n");
		return -EINVAL;
	}
	cfg->clk_src_clkin = IDT_PRM_CLK_CRYSTAL;
	cfg->clock_id = clock1_pll_id;
	cfg->out_id = clock1_out_id;
	cfg->cfg_id = IDT_CLK_CFG_0;
	cfg->cfg_act = 1;
		
	if (idt5v49ee503_set_freq(idt_drv, val, cfg)) {
		dev_err(dev, "failed to set clock to %llu\n", val);
		count = -EIO;
		goto exit;
	}
	return count;
exit:
	kfree(cfg);
	return count;
}

static ssize_t idt5v49ee503_storef(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	idt_drv_info_t *idt_drv = i2c_get_clientdata(client);
	idt_clock_cfg_t *cfg;
	char *endp;
	u64 val;

	dev_dbg(dev, "idt5v49ee503_store() called on %s\n", attr->attr.name);

	/* Decode input */
	val = simple_strtoull(buf, &endp, 0);
	if (buf == endp) {
		dev_dbg(dev, "input string not a number\n");
		return -EINVAL;
	}
	cfg = kzalloc(sizeof(idt_clock_cfg_t), GFP_KERNEL);
	if (cfg == NULL) {
		dev_dbg(dev, "failed to allocate memory\n");
		return -EINVAL;
	}
	cfg->clk_src_clkin = IDT_PRM_CLK_CRYSTAL;
	cfg->clock_id = clock0_pll_id;
	cfg->out_id = clock0_out_id;
	cfg->cfg_id = IDT_CLK_CFG_0;
	cfg->cfg_act = 1;
		
	if (idt5v49ee503_set(idt_drv, val, cfg)) {
		dev_err(dev, "failed to set clock to %llu\n", val);
		return -EIO;
	}

	return count;
}
unsigned char  reg_addr;
static ssize_t idt5v49ee503_store_reg(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	char *endp;
	u64 val;

	dev_dbg(dev, "idt5v49ee503_store_reg() called on %s\n", attr->attr.name);

	/* Decode input */
	val = simple_strtoull(buf, &endp, 0);
	if (buf == endp) {
		dev_dbg(dev, "input string not a number\n");
		return -EINVAL;
	}
	reg_addr = val;

	return count;
}
static ssize_t idt5v49ee503_show_reg(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	dev_dbg(dev, "idt5v49ee503_show_reg() called on %s\n", attr->attr.name);

	/* Format the output string and return # of bytes */
	return sprintf(buf, "%x\n", reg_addr);
}

static ssize_t idt5v49ee503_store_reg_val(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	idt_drv_info_t *idt_drv = i2c_get_clientdata(client);
	char *endp;
	unsigned int val;

	dev_dbg(dev, "idt5v49ee503_store_reg_val() called on %s\n", attr->attr.name);

	/* Decode input */
	val = simple_strtoul(buf, &endp, 0);
	if (buf == endp) {
		dev_dbg(dev, "input string not a number\n");
		return -EINVAL;
	}
	dev_printk(KERN_INFO, dev, "idt5v49ee503 write 0x%x to register 0x%x \n", val, (unsigned int)reg_addr);
	idt5v49ee503_write_reg(idt_drv,
			       reg_addr,
			       val);

	return count;
}
static ssize_t idt5v49ee503_show_reg_val(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	idt_drv_info_t *idt_drv = i2c_get_clientdata(client);
	int           rval;
	unsigned char val = 0;

	dev_dbg(dev, "idt5v49ee503_show_reg_val() called on %s\n", attr->attr.name);
	dev_printk(KERN_INFO, dev, "idt5v49ee503 read from register 0x%x \n", (unsigned int)reg_addr);
	if ((rval = idt5v49ee503_read_reg(idt_drv, reg_addr, &val)) != 0)
                return rval;

	/* Format the output string and return # of bytes */
	return sprintf(buf, "%x\n", val);
}

/*
 * Simple register attributes
 */

static DEVICE_ATTR(clk0, S_IRUGO | S_IWUSR, idt5v49ee503_show, idt5v49ee503_store);
static DEVICE_ATTR(clk1, S_IRUGO | S_IWUSR, idt5v49ee503_show, idt5v49ee503_store2);
static DEVICE_ATTR(clkf, S_IRUGO | S_IWUSR, idt5v49ee503_show, idt5v49ee503_storef);
static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR, idt5v49ee503_show_reg, idt5v49ee503_store_reg);
static DEVICE_ATTR(val, S_IRUGO | S_IWUSR, idt5v49ee503_show_reg_val, idt5v49ee503_store_reg_val);

static const struct attribute_group idt5v49ee503_group = {
	.attrs = (struct attribute *[]) {
		&dev_attr_clk0.attr,
		&dev_attr_clk1.attr,
		&dev_attr_clkf.attr,
		&dev_attr_reg.attr,
		&dev_attr_val.attr,
		NULL,
	},
};

static void  idt_clk_enable(struct clk *clk)
{
	idt_clock_id_t pll_id;
	if (clk->flags == 0)
		pll_id = clock0_pll_id;
	else
		pll_id = clock1_pll_id;
		
	idt5v49ee503_pll_enable(idt_drv_g, pll_id , 1);

	return;
}

static void  idt_clk_disable(struct clk *clk)
{
	idt_clock_id_t pll_id;

	if (clk->flags == 0)
		pll_id = clock0_pll_id;
	else
		pll_id = clock1_pll_id;
		
	idt5v49ee503_pll_enable(idt_drv_g, pll_id , 0);

	return;
}
static unsigned long idt_clk_getrate(struct clk *clk)
{
	return *(clk->rate);
}

static int idt_clk_setrate(struct clk *clk, unsigned long rate)
{
	idt_clock_cfg_t *cfg;
	idt_clock_id_t pll_id;
	idt_output_id_t out_id;
	int rc = 0;

	if (clk->flags == 0) {
		pll_id = clock0_pll_id;
		out_id = clock0_out_id;
	} else {
		pll_id = clock1_pll_id;
		out_id = clock1_out_id;
	}

	cfg = kzalloc(sizeof(idt_clock_cfg_t), GFP_KERNEL);
	if (cfg == NULL) {
		printk("idt clk: failed to allocate memory\n");
		return -EINVAL;
	}
	cfg->clk_src_clkin = IDT_PRM_CLK_CRYSTAL;
	cfg->clock_id = pll_id;
	cfg->out_id = out_id;
	cfg->cfg_id = IDT_CLK_CFG_0;
	cfg->cfg_act = 1;
	printk("set clk %d to %lu\n", clk->flags, rate);	
	if (idt5v49ee503_set_freq(idt_drv_g, rate, cfg)) {
		printk("idt clk: failed to set clock to %lu\n", rate);
		rc = -EIO;
	} else
		*(clk->rate) = rate;
	kfree(cfg);
	return rc;
}

const struct clkops idt_clk_ops = {
	.enable		= idt_clk_enable,
	.disable	= idt_clk_disable,
	.setrate	= idt_clk_setrate,
	.getrate	= idt_clk_getrate,
};
static unsigned long idt_clk0_rate = 0;
static unsigned long idt_clk1_rate = 0;

static struct clk idt_clk0 = {
	.ops	= &idt_clk_ops,
	.flags	= 0,
	.rate	= &idt_clk0_rate,
};

static struct clk idt_clk1 = {
	.ops	= &idt_clk_ops,
	.flags	= 1,
	.rate	= &idt_clk1_rate,
};

static struct clk_lookup idt_clocks[] = {
	{
		.con_id = "IDT_CLK0",
		.clk	= &idt_clk0,
	},
	{
		.con_id = "IDT_CLK1",
		.clk	= &idt_clk1,
	},
};

/*
 * Called when a idt5v49ee503 device is matched with this driver
 */
static int idt5v49ee503_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	idt_drv_info_t *idt_drv;
	struct idt_data *data;
	int i;
	int rc;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "i2c bus does not support the idt5v49ee503\n");
		rc = -ENODEV;
		goto exit;
	}
	idt_drv = kzalloc(sizeof(idt_drv_info_t), GFP_KERNEL);
	
	if (idt_drv == NULL)
		return -ENOMEM;
	idt_drv_g = idt_drv;
	idt_drv->i2c_cl = client;
	i2c_set_clientdata(client, idt_drv);

	/* disable all pll's by default */
	idt5v49ee503_write_reg(idt_drv,
			       IDT5V49EE503_PLL_SUSPEND_REG,
			       0);
	idt5v49ee503_write_reg(idt_drv,
			       IDT5V49EE503_OUT_ENA_REG,
			       0);
	idt5v49ee503_write_reg(idt_drv,
			       IDT5V49EE503_OUT_SUSPEND_REG,
			       0);


	rc = sysfs_create_group(&client->dev.kobj, &idt5v49ee503_group);
	if (rc)
		goto free;

	data = client->dev.platform_data;
	if (data) {
		if (data->clock0_enable) {
			clock0_out_id = data->clock0_out_id;
			clock0_pll_id = data->clock0_pll_id;
			clkdev_add(&idt_clocks[0]);
		}
		if (data->clock1_enable) {
			clock1_out_id = data->clock1_out_id;
			clock1_pll_id = data->clock1_pll_id;
			clkdev_add(&idt_clocks[1]);
		}
	} else /* if no platform data provided, enable two clocks using the default configuration */
		for (i = 0; i < ARRAY_SIZE(idt_clocks); i++)
			clkdev_add(&idt_clocks[i]);

	return rc;
 free:
	kfree(idt_drv);
 exit:
	return rc;
}

static int idt5v49ee503_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &idt5v49ee503_group);
	return 0;
}

static const struct i2c_device_id idt5v49ee503_id[] = {
	{ "idt5v49ee503", IDT5V49EE503_BUS_ADDR },
	{ }
};
MODULE_DEVICE_TABLE(i2c, idt5v49ee503_id);

static struct i2c_driver idt5v49ee503_driver = {
	.driver = {
		.name = "idt5v49ee503",
	},
	.probe = idt5v49ee503_probe,
	.remove = idt5v49ee503_remove,
	.id_table = idt5v49ee503_id,
};

static int __init idt5v49ee503_i2c_init(void)
{
	return i2c_add_driver(&idt5v49ee503_driver);
}

static void __exit idt5v49ee503_i2c_exit(void)
{
	i2c_del_driver(&idt5v49ee503_driver);
}

MODULE_AUTHOR("kostap <kostap@gandalf602.il.marvell.com>");
MODULE_DESCRIPTION("IDT5V49EE503 I2C clock generator driver");
MODULE_LICENSE("GPL");

module_init(idt5v49ee503_i2c_init);
module_exit(idt5v49ee503_i2c_exit);
