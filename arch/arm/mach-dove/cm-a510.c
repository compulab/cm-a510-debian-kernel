/*
 * arch/arm/mach-dove/cm-a510.c
 *
 * Copyright (C) 2010 CompuLab, Ltd.
 * Konstantin Sinyuk <kostyas@compulab.co.il>
 *
 * Based on Marvell DB-MV88AP510-BP Development Board Setup
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand.h>
#include <linux/timer.h>
#include <linux/ata_platform.h>
#include <linux/mv643xx_eth.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/pci.h>
#include <linux/gpio_mouse.h>
#include <linux/gpio_keys.h>
#include <linux/spi/spi.h>
#include <linux/spi/orion_spi.h>
#include <linux/spi/flash.h>
#include <linux/spi/ads7846.h>
#include <linux/input.h>
#include <video/dovefb.h>
#include <video/dovefbreg.h>
#include <mach/dove_bl.h>
#include <plat/i2s-orion.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/mach/arch.h>
#include <mach/dove.h>
#include <asm/hardware/pxa-dma.h>
#include <plat/orion_nfc.h>
#include <mach/dove_ssp.h>
#include <mach/pm.h>
#include "common.h"
#include "clock.h"
#include "mpp.h"
#include "pmu/mvPmu.h"
#include "pmu/mvPmuRegs.h"
#include "pdma/mvPdma.h"

#define CM_A510_WAKEUP_GPIO	(3)
#define CM_A510_POWER_OFF_GPIO	(8)

static unsigned int use_hal_giga = 0;
#ifdef CONFIG_MV643XX_ETH
module_param(use_hal_giga, uint, 0);
MODULE_PARM_DESC(use_hal_giga, "Use the HAL giga driver");
#endif

static int dvs_enable = 0;
module_param(dvs_enable, int, 0);
MODULE_PARM_DESC(dvs_enable, "if 1 then enable DVS");

extern unsigned int useHalDrivers;
extern char *useNandHal;


extern unsigned int lcd0_enable;
extern unsigned int lcd1_enable;
/*
 * set lcd clock source in dovefb_mach_info, notice the sequence is not
 * same to bootargs.
 * = MRVL_AXI_CLK, choose AXI, 333Mhz.
         name is "AXICLK"
 * = MRVL_PLL_CLK, choose PLL, auto use accurate mode if only one LCD refer to it.
         name is "LCDCLK" or "accurate_LCDCLK"
 * = MRVL_EXT_CLK0, choose external clk#0, (available REV A0)
         name is "IDT_CLK0"
 * = MRVL_EXT_CLK1, choose external clk#1, (available REV A0)
         name is "IDT_CLK1"
 */
/*
 * LCD HW output Red[0] to LDD[0] when set bit [19:16] of reg 0x190
 * to 0x0. Which means HW outputs BGR format default. All platforms
 * uses this controller should enable .panel_rbswap. Unless layout
 * design connects Blue[0] to LDD[0] instead.
 */
static struct dovefb_mach_info cm_a510_lcd0_dmi = {
	.id_gfx			= "GFX Layer 0",
	.id_ovly		= "Video Layer 0",
	.clk_src		= MRVL_PLL_CLK,
	.clk_name		= "accurate_LCDCLK0",
//	.clk_src		= MRVL_EXT_CLK1,
//	.clk_name		= "IDT_CLK1",
//	.num_modes		= ARRAY_SIZE(video_modes),
//	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB888PACK,
	.io_pin_allocation	= IOPAD_DUMB24,
	.panel_rgb_type		= DUMB24_RGB888_0,
	.panel_rgb_reverse_lanes= 0,
	.gpio_output_data	= 0,
	.gpio_output_mask	= 0,
#ifndef CONFIG_FB_DOVE_CLCD0_I2C_DEFAULT_SETTING
	.ddc_i2c_adapter	= CONFIG_FB_DOVE_CLCD0_I2C_CHANNEL,
	.ddc_i2c_address	= CONFIG_FB_DOVE_CLCD0_I2C_ADDRESS,
#else
	.ddc_i2c_adapter	= 0,
	.ddc_i2c_address        = 0x3f,
#endif
	.invert_composite_blank	= 0,
	.invert_pix_val_ena	= 0,
	.invert_pixclock	= 0,
	.invert_vsync		= 0,
	.invert_hsync		= 0,
	.panel_rbswap		= 1,
	.active			= 1,
};

static struct dovefb_mach_info cm_a510_lcd0_vid_dmi = {
	.id_ovly		= "Video Layer 0",
//	.num_modes		= ARRAY_SIZE(video_modes),
//	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB888PACK,
	.io_pin_allocation	= IOPAD_DUMB24,
	.panel_rgb_type		= DUMB24_RGB888_0,
	.panel_rgb_reverse_lanes= 0,
	.gpio_output_data	= 0,
	.gpio_output_mask	= 0,
	.ddc_i2c_adapter	= -1,
	.invert_composite_blank	= 0,
	.invert_pix_val_ena	= 0,
	.invert_pixclock	= 0,
	.invert_vsync		= 0,
	.invert_hsync		= 0,
	.panel_rbswap		= 1,
	.active			= 0,
	.enable_lcd0		= 0,
};

static struct dovefb_mach_info cm_a510_lcd1_dmi = {
	.id_gfx			= "GFX Layer 1",
	.id_ovly		= "Video Layer 1",
	.clk_src		= MRVL_PLL_CLK,
	.clk_name		= "accurate_LCDCLK",
//	.num_modes		= ARRAY_SIZE(video_modes),
//	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB565,
	.io_pin_allocation	= IOPAD_DUMB24,
	.panel_rgb_type		= DUMB24_RGB888_0,
	.panel_rgb_reverse_lanes= 0,
	.gpio_output_data	= 0,
	.gpio_output_mask	= 0,
#ifndef CONFIG_FB_DOVE_CLCD1_I2C_DEFAULT_SETTING
        .ddc_i2c_adapter        = CONFIG_FB_DOVE_CLCD1_I2C_CHANNEL,
        .ddc_i2c_address        = CONFIG_FB_DOVE_CLCD1_I2C_ADDRESS,
#else
        .ddc_i2c_adapter        = 1,
        .ddc_i2c_address        = 0x50,
#endif
	.invert_composite_blank	= 0,
	.invert_pix_val_ena	= 0,
	.invert_pixclock	= 0,
	.invert_vsync		= 0,
	.invert_hsync		= 0,
	.panel_rbswap		= 1,
	.active			= 1,
#ifndef CONFIG_FB_DOVE_CLCD
	.enable_lcd0		= 1,
#else
	.enable_lcd0		= 0,
#endif
};

static struct dovefb_mach_info cm_a510_lcd1_vid_dmi = {
	.id_ovly		= "Video Layer 1",
//	.num_modes		= ARRAY_SIZE(video_modes),
//	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB888PACK,
	.io_pin_allocation	= IOPAD_DUMB24,
	.panel_rgb_type		= DUMB24_RGB888_0,
	.panel_rgb_reverse_lanes= 0,
	.gpio_output_data	= 0,
	.gpio_output_mask	= 0,
	.ddc_i2c_adapter	= -1,
	.invert_composite_blank	= 0,
	.invert_pix_val_ena	= 0,
	.invert_pixclock	= 0,
	.invert_vsync		= 0,
	.invert_hsync		= 0,
	.panel_rbswap		= 1,
	.active			= 0,
};



static struct dove_ssp_platform_data dove_ssp_platform_data = {
	.use_dma = 0,
	.use_loopback = 1,
	.dss = 32,
	.scr = 2,
	.frf = 1,
	.rft = 8,
	.tft = 8
};


/*****************************************************************************
 * BACKLIGHT
 ****************************************************************************/
static struct dovebl_platform_data cm_a510_backlight_data = {
	.default_intensity = 0xa,
	.gpio_pm_control = 1,

	.lcd_start = DOVE_LCD1_PHYS_BASE,	/* lcd power control reg base. */
	.lcd_end = DOVE_LCD1_PHYS_BASE+0x1C8,	/* end of reg map. */
	.lcd_offset = LCD_SPU_DUMB_CTRL,	/* register offset */
	.lcd_mapped = 0,		/* va = 0, pa = 1 */
	.lcd_mask = 0x200000,		/* mask, bit[21] */
	.lcd_on = 0x200000,			/* value to enable lcd power */
	.lcd_off = 0x0,			/* value to disable lcd power */

	.blpwr_start = DOVE_LCD1_PHYS_BASE, /* bl pwr ctrl reg base. */
	.blpwr_end = DOVE_LCD1_PHYS_BASE+0x1C8,	/* end of reg map. */
	.blpwr_offset = LCD_SPU_DUMB_CTRL,	/* register offset */
	.blpwr_mapped = 0,		/* pa = 0, va = 1 */
	.blpwr_mask = 0x100000,		/* mask */
	.blpwr_on = 0x100000,		/* value to enable bl power */
	.blpwr_off = 0x0,		/* value to disable bl power */

	.btn_start = DOVE_LCD1_PHYS_BASE, /* brightness control reg base. */
	.btn_end = DOVE_LCD1_PHYS_BASE+0x1C8,	/* end of reg map. */
	.btn_offset = LCD_CFG_GRA_PITCH,	/* register offset */
	.btn_mapped = 0,		/* pa = 0, va = 1 */
	.btn_mask = 0xF0000000,	/* mask */
	.btn_level = 15,	/* how many level can be configured. */
	.btn_min = 0x1,	/* min value */
	.btn_max = 0xF,	/* max value */
	.btn_inc = 0x1,	/* increment */
};

void __init cm_a510_clcd_init(void) {
#ifdef CONFIG_FB_DOVE
	struct dovefb_mach_info *lcd0_dmi, *lcd0_vid_dmi;
	u32 dev, rev;

	dove_pcie_id(&dev, &rev);

	lcd0_dmi = &cm_a510_lcd0_dmi;
	lcd0_vid_dmi = &cm_a510_lcd0_vid_dmi;

	clcd_platform_init(lcd0_dmi, lcd0_vid_dmi,
			   &cm_a510_lcd1_dmi, &cm_a510_lcd1_vid_dmi,
			   &cm_a510_backlight_data);

#endif /* CONFIG_FB_DOVE */
}

#define CM_A510_PHY_RST_GPIO		(1)
#define CM_A510_TS_PEN_GPIO			(13)
#define CM_A510_TS_PEN_IRQ			(64 + CM_A510_TS_PEN_GPIO)
#define CM_A510_LED_GPIO         	(65)
#define CM_A510_USB_HUB_nRST		(69)
#define CM_A510_WLAN_nRST			(70)
#define CM_A510_WLAN_REG_ON			(71)


/*
 * Touchscreen - TSC2046 connected to SSP2
 */

static const struct ads7846_platform_data tsc2046_info = {
	.model            = 7846,
	.vref_delay_usecs = 100,
	.pressure_max     = 1024,
	.debounce_max     = 10,
	.debounce_tol     = 3,
	.debounce_rep     = 1,
	.gpio_pendown     = CM_A510_TS_PEN_GPIO,
};

static struct spi_board_info __initdata tsc2046_board_info[] = {
	{
		.modalias        = "ads7846",
		.bus_num         = 1,
		.max_speed_hz    = 2600000, /* 100 kHz sample rate */
		.irq             = CM_A510_TS_PEN_IRQ,
		.platform_data   = &tsc2046_info,
		.chip_select	 = 0,
	},
};

int __init cm_a510_ts_gpio_setup(void)
{

	orion_gpio_set_valid(CM_A510_TS_PEN_GPIO, 1);
	/*
	if (gpio_request(CM_A510_TS_PEN_GPIO, "DOVE_TS_PEN_IRQ") != 0)
		pr_err("Dove: failed to setup TS IRQ GPIO\n");
	if (gpio_direction_input(CM_A510_TS_PEN_GPIO) != 0) {
		printk(KERN_ERR "%s failed "
		       "to set output pin %d\n", __func__,
		       CM_A510_TS_PEN_GPIO);
		gpio_free(CM_A510_TS_PEN_GPIO);
		return -1;
	}
	*/
	/* IRQ */
	set_irq_chip(CM_A510_TS_PEN_IRQ, &orion_gpio_irq_chip);
	set_irq_handler(CM_A510_TS_PEN_IRQ, handle_level_irq);
	set_irq_type(CM_A510_TS_PEN_IRQ, IRQ_TYPE_LEVEL_LOW);

	return 0;
}

int __init cm_a510_giga_phy_gpio_setup(void)
{
	orion_gpio_set_valid(CM_A510_PHY_RST_GPIO, 1);
	if (gpio_request(CM_A510_PHY_RST_GPIO, "Giga Phy reset gpio") != 0)
		pr_err("Dove: failed to setup Giga phy reset GPIO\n");
	if (gpio_direction_output(CM_A510_PHY_RST_GPIO, 1) != 0) {
		printk(KERN_ERR "%s failed to set output pin %d\n", __func__,
		       CM_A510_PHY_RST_GPIO);
		gpio_free(CM_A510_PHY_RST_GPIO);
		return -1;
	}

	return 0;
}

int __init cm_a510_wlan_init(void)
{
	orion_gpio_set_valid(CM_A510_WLAN_nRST, 2);

	if (gpio_request(CM_A510_WLAN_nRST, "CM-A510 WLAN nRESET ") != 0)
		pr_err("Dove: failed to setup system WLAN nRESET GPIO\n");

	if (gpio_direction_output(CM_A510_WLAN_nRST, 1) != 0)
		printk(KERN_ERR "%s failed to set output pin %d\n", __func__,
		       CM_A510_WLAN_nRST);
	gpio_set_value(CM_A510_WLAN_nRST,1);

	orion_gpio_set_valid(CM_A510_WLAN_REG_ON, 2);

	if (gpio_request(CM_A510_WLAN_REG_ON, "CM-A510 WLAN REG ON ") != 0)
		pr_err("Dove: failed to setup system WLAN REG ON GPIO\n");

	if (gpio_direction_output(CM_A510_WLAN_REG_ON, 1) != 0)
		printk(KERN_ERR "%s failed to set output pin %d\n", __func__,
		       CM_A510_WLAN_REG_ON);
	gpio_set_value(CM_A510_WLAN_REG_ON,1);

	return 0;
}

int __init cm_a510_usb_hub_init()
{

	orion_gpio_set_valid(CM_A510_USB_HUB_nRST, 2);

	if (gpio_request(CM_A510_USB_HUB_nRST, "CM-A510 USB Hub Reset ") != 0)
		pr_err("Dove: failed to setup system USB hub \n");

	if (gpio_direction_output(CM_A510_USB_HUB_nRST, 1) != 0)
		printk(KERN_ERR "%s failed to set output pin %d\n", __func__,
		       CM_A510_USB_HUB_nRST);

	gpio_set_value(CM_A510_USB_HUB_nRST,0);
	msleep(500);
	gpio_set_value(CM_A510_USB_HUB_nRST,1);

	return 0;
}

#define SB_A510_DVI_PD_GPIO     (72 + 3)
#define SB_A510_LVDS_PD_GPIO    (72 + 4)
#define SB_A510_PCI_RST_GPIO    (72 + 6)
#define SB_A510_MMC_PWEN        (72 + 11)

int __init cm_a510_led_setup(int level)
{

        orion_gpio_set_valid(CM_A510_LED_GPIO, 2);

	if (gpio_request(CM_A510_LED_GPIO, "CM-A510 System LED") != 0)
		pr_err("Dove: failed to setup system LED GPIO\n");

	if (gpio_direction_output(CM_A510_LED_GPIO, 1) != 0)
		printk(KERN_ERR "%s failed to set output pin %d\n", __func__,
		       CM_A510_LED_GPIO);
	gpio_set_value(CM_A510_LED_GPIO,level);
	return 0;
}


static int __init cm_a510_lateinit(void)
{
	int ret = 0;
	/* If LCD0 interface is active enable DVI transeiver on SB-A510 */
	if (lcd0_enable)
	{
		ret = gpio_request(SB_A510_DVI_PD_GPIO,"SB-A510 DVI PD ");
		if (ret == 0) {
			gpio_direction_output(SB_A510_DVI_PD_GPIO, 1);
			gpio_set_value(SB_A510_DVI_PD_GPIO, 1);
		}
	}
/*
	ret = gpio_request(SB_A510_PCI_RST_GPIO,"SB-A510 PCI RST");
	if (ret == 0) {
		gpio_direction_output(SB_A510_PCI_RST_GPIO, 1);
		gpio_set_value(SB_A510_PCI_RST_GPIO, 1);
       	}
*/


        ret = gpio_request(SB_A510_MMC_PWEN,"SB-A510 MMC Power Enable");
        if (ret == 0) {
		gpio_direction_output(SB_A510_MMC_PWEN, 0);
               gpio_set_value(SB_A510_MMC_PWEN, 0);
        }

	ret = gpio_request(SB_A510_LVDS_PD_GPIO,"SB-A510 LVDS PD#");
        if (ret == 0) {
		gpio_direction_output(SB_A510_LVDS_PD_GPIO, 1);
               gpio_set_value(SB_A510_LVDS_PD_GPIO, 1);
        }

	/* Proceed with MMC init when the MMC power is ready */
	dove_sdio0_init();
	dove_sdio1_init();

	return 0;
}

late_initcall(cm_a510_lateinit);

extern int __init pxa_init_dma_wins(struct mbus_dram_target_info * dram);

static struct orion_i2s_platform_data i2s1_data = {
	.i2s_play	= 1,
	.i2s_rec	= 1,
	.spdif_play	= 1,
};

static struct orion_i2s_platform_data i2s0_data = {
	.i2s_play	= 1,
	.i2s_rec	= 1,
};

static struct mv643xx_eth_platform_data cm_a510_ge00_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR_DEFAULT,
};

static struct mv_sata_platform_data cm_a510_sata_data = {
        .n_ports        = 1,
};


/*****************************************************************************
 * SPI Devices:
 * 	SPI0: 1M Flash Winbond w25q32bv
 ****************************************************************************/
static const struct flash_platform_data cm_a510_spi_flash_data = {
	.type		= "w25q32bv",
};

static struct spi_board_info __initdata cm_a510_spi_flash_info[] = {
	{
		.modalias       = "m25p80",
	/*	.platform_data  = &cm_a510_spi_flash_data,    */
		.irq            = -1,
		.max_speed_hz   = 20000000,
		.bus_num        = 0,
		.chip_select    = 0,
	},
};

/*****************************************************************************
 * PCI
 ****************************************************************************/
static int __init cm_a510_pci_init(void)
{
	if (machine_is_cm_a510()) {
		dove_pcie_init(1, 1);
	}

	return 0;
}

subsys_initcall(cm_a510_pci_init);


/* PCA9555 */
static struct pca953x_platform_data cm_a510_gpio_ext_pdata = {
	.gpio_base = 72,
};

static struct i2c_board_info cm_a510_gpio_ext_info[] = {
	[0] = {
		I2C_BOARD_INFO("pca9555", 0x20),
		.platform_data = &cm_a510_gpio_ext_pdata,
	},
};

/*****************************************************************************
 * Sound on I2C bus
 ****************************************************************************/
static struct i2c_board_info __initdata sound1_i2c_info = {
	I2C_BOARD_INFO("tlv320aic23", 0x1A),
};


/*****************************************************************************
 * NAND
 ****************************************************************************/
static struct mtd_partition partition_dove[] = {
	{ .name		= "UImage",
	  .offset	= 0,
	  .size		= 4 * SZ_1M },
	{ .name		= "Root",
	  .offset	= MTDPART_OFS_APPEND,
	  .size         = MTDPART_SIZ_FULL},
};
static u64 nfc_dmamask = DMA_BIT_MASK(32);
static struct nfc_platform_data cm_a510_nfc_hal_data = {
	.nfc_width      = 8,			/* Assume non-ganged by default */
	.num_devs       = 1,			/* Assume non-ganged by default */
	.num_cs		= 1,			/* MT 29F64G908CBAAA */
	.use_dma	= 1,
	.ecc_type	= MV_NFC_ECC_BCH_2K,	/* 4bit ECC required by K9HBG08U1A */
	.parts = partition_dove,
	.nr_parts = ARRAY_SIZE(partition_dove)
};

static struct nfc_platform_data cm_a510_nfc_data = {
	.nfc_width      = 8,
	.num_devs       = 1,
	.num_cs         = 1,
	.use_dma        = 0,
	.ecc_type	= MV_NFC_ECC_BCH_2K,
	.parts = partition_dove,
	.nr_parts = ARRAY_SIZE(partition_dove)
 };

static struct resource dove_nfc_resources[]  = {
	[0] = {
		.start	= (DOVE_NFC_PHYS_BASE),
		.end	= (DOVE_NFC_PHYS_BASE + 0xFF),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_NAND,
		.end	= IRQ_NAND,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* DATA DMA */
		.start	= 97,
		.end	= 97,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		/* COMMAND DMA */
		.start	= 99,
		.end	= 99,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device dove_nfc = {
	.name		= "orion-nfc",
	.id		= -1,
	.dev		= {
		.dma_mask		= &nfc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &cm_a510_nfc_data,
	},
	.resource	= dove_nfc_resources,
	.num_resources	= ARRAY_SIZE(dove_nfc_resources),
};

static void __init cm_a510_nfc_init(void)
{
	/* Check if HAL driver is intended */
	if(useHalDrivers || useNandHal) {
		dove_nfc.name = "orion-nfc-hal";

		cm_a510_nfc_hal_data.tclk = dove_tclk_get();
		dove_nfc.dev.platform_data = &cm_a510_nfc_hal_data;

	} else {  /* NON HAL driver is used */
		cm_a510_nfc_data.tclk = dove_tclk_get();
	}

	platform_device_register(&dove_nfc);
}

static struct dove_mpp_mode cm_a510_mpp_modes[] __initdata = {
	{ 0, MPP_GPIO },           	/* PMU - External wakup 0 */
	{ 1, MPP_GPIO },           	/* DDR termination control */
	{ 2, MPP_PMU },			/* Standby power control */
	{ 3, MPP_GPIO },		/* power off */
	{ 4, MPP_PMU },		        /* DDR_CKE_MASK - must be controled by PMU*/
	{ 5, MPP_PMU },			/* DeepIdle power control */
	{ 6, MPP_GPIO },		/* DRR nReset */
	{ 7, MPP_GPIO },		/* PMU - External wakup 1 */
	{ 8, MPP_GPIO },		/* MCU_MISC0 */
	{ 9, MPP_PCIE },		/* PCIE1_nCLKREQ */
	{ 10, MPP_PMU },		/* DVS SDI control */
	{ 11, MPP_PCIE },               /* PCIE0_nCLKREQ */
	{ 12, MPP_GPIO },           	/* LAN0 interrupt */
	{ 13, MPP_GPIO },           	/* TS PEN IRQ */
        { 14, MPP_UART2 },              /* UART2 TXD */
        { 15, MPP_UART2},               /* UART2 RXD */
	{ 16, MPP_SDIO0 },		/* SD0_CD */
	{ 17, MPP_TWSI },               /* TW_SDA - I2C2 */
	{ 18, MPP_GPIO },               /* GPIO18 */
	{ 19, MPP_TWSI },               /* TW_SCL - I2C2 */
	{ 20, MPP_SPI1 },               /* Touch screen - SPI1 */
	{ 21, MPP_SPI1 },               /* Touch screen - SPI1 */
	{ 22, MPP_SPI1 },               /* Touch screen - SPI1 */
	{ 23, MPP_SPI1 },               /* Touch screen - SPI1 */

	{ 24, MPP_CAM }, /* will configure MPPs 24-39*/

	{ 40, MPP_SDIO0 }, /* SD0 Group MPPs 40-45 */

	{ 46, MPP_SDIO1 }, /* SD1 Group MPPs 46-51 */

        { 52, MPP_AUDIO1 }, /* AU1 Group */

	{ 58, MPP_SPI0 }, /* will configure MPPs 58-61 */

        { 62, MPP_UART1 }, /* UART1 RXD */
        { 63, MPP_UART1 }, /* UART2 TXD */
	{ 64, MPP_GPIO }, /*Set NFC8_15 as output GPIOs */
        { -1 },
};


static void cm_a510_power_off(void)
{
	if (gpio_request(CM_A510_POWER_OFF_GPIO, "CM_A510_POWER_OFF") != 0) {
		pr_err("Dove: failed to setup power off GPIO\n");
		return;
	}

	if (gpio_direction_output(CM_A510_POWER_OFF_GPIO, 0) != 0) {
 		printk(KERN_ERR "%s failed to set power off output pin %d\n",
		       __func__, CM_A510_POWER_OFF_GPIO);
		return;
	}
}

#ifdef CONFIG_PM
/*****************************************************************************
 * POWER MANAGEMENT
 ****************************************************************************/
extern int global_dvs_enable;
static int __init cm_a510_pm_init(void)
{
	MV_PMU_INFO pmuInitInfo;
	u32 dev, rev;

	if (!machine_is_cm_a510())
		return 0;

	global_dvs_enable = dvs_enable;

	dove_pcie_id(&dev, &rev);

	pmuInitInfo.batFltMngDis = MV_FALSE;				/* Keep battery fault enabled */
	pmuInitInfo.exitOnBatFltDis = MV_FALSE;				/* Keep exit from STANDBY on battery fail enabled */

	pmuInitInfo.sigSelctor[0] = PMU_SIGNAL_EXT0_WKUP; /* EXT0_WU input */
	pmuInitInfo.sigSelctor[1] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[2] = PMU_SIGNAL_SLP_PWRDWN;	/* STANDBY => 0: I/O off, 1: I/O on */
	pmuInitInfo.sigSelctor[3] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[4] = PMU_SIGNAL_CKE_OVRID;	/* CKE controlled by Dove */
	pmuInitInfo.sigSelctor[5] = PMU_SIGNAL_CPU_PWRDWN;	/* DEEP-IdLE => 0: CPU off, 1: CPU on */
	pmuInitInfo.sigSelctor[6] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[7] = PMU_SIGNAL_EXT1_WKUP;  /* EXT1_WU input */
	pmuInitInfo.sigSelctor[8] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[9] = PMU_SIGNAL_NC;			/* CPU power good  - not used */
	pmuInitInfo.sigSelctor[10] = PMU_SIGNAL_SDI;			/* Voltage regulator control */
	pmuInitInfo.sigSelctor[11] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[12] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[13] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[14] = PMU_SIGNAL_NC;
	pmuInitInfo.sigSelctor[15] = PMU_SIGNAL_NC;
	pmuInitInfo.dvsDelay = 0x4200;				/* ~100us in 166MHz cc - delay for DVS change */\
                pmuInitInfo.ddrTermGpioNum = 1;			/* GPIO 1 used to disable terminations */

	/* For X0 and higher wait at least 150ms + spare */
	pmuInitInfo.standbyPwrDelay = 0x2000;		/* 250ms delay to wait for complete powerup */

	/* Initialize the PMU HAL */
	if (mvPmuInit(&pmuInitInfo) != MV_OK)
	{
		printk(KERN_ERR "ERROR: Failed to initialise the PMU!\n");
		return 0;
	}

	/* Configure wakeup events */
	mvPmuWakeupEventSet(PMU_STBY_WKUP_CTRL_EXT0_FALL | PMU_STBY_WKUP_CTRL_RTC_MASK);

	/* Register the PM operation in the Linux stack */
	dove_pm_register();

	return 0;
}

__initcall(cm_a510_pm_init);
#endif

/*****************************************************************************
 * Board Init
 ****************************************************************************/
static void __init cm_a510_init(void)
{
	u32 dev, rev;

	/*
	 * Basic Dove setup. Needs to be called early.
	 */
	dove_init();

	dove_pcie_id(&dev, &rev);
	dove_mpp_conf(cm_a510_mpp_modes);
	pm_power_off = cm_a510_power_off;

        /* the (SW1) button is for use as a "wakeup" button */
	dove_wakeup_button_setup(CM_A510_WAKEUP_GPIO);

	/* sdio card interrupt workaround using GPIOs */
	dove_sd_card_int_wa_setup(0);
	dove_sd_card_int_wa_setup(1);

	cm_a510_ts_gpio_setup();

	dove_rtc_init();

	pxa_init_dma_wins(&dove_mbus_dram_info);
	pxa_init_dma(16);
#ifdef CONFIG_MV_HAL_DRIVERS_SUPPORT
	if(useHalDrivers || useNandHal) {
		if (mvPdmaHalInit(MV_PDMA_MAX_CHANNELS_NUM) != MV_OK) {
			printk(KERN_ERR "mvPdmaHalInit() failed.\n");
			BUG();
		}
		/* reserve channels for NAND Data and command PDMA */
		pxa_reserve_dma_channel(MV_PDMA_NAND_DATA);
		pxa_reserve_dma_channel(MV_PDMA_NAND_COMMAND);
	}
#endif
	dove_xor0_init();
	dove_xor1_init();
#ifdef CONFIG_MV_ETHERNET
	if(use_hal_giga || useHalDrivers)
		dove_mv_eth_init();
	else
#endif
	dove_ge00_init(&cm_a510_ge00_data);
	dove_ehci0_init();
	dove_ehci1_init();

	/* ehci init functions access the usb port, only now it's safe to disable
	 * all clocks
	 */
	ds_clks_disable_all(0, 0);
	dove_sata_init(&cm_a510_sata_data);
	dove_spi0_init(0);
	dove_spi1_init(1);
	dove_uart0_init();
	dove_uart1_init();
	dove_i2c_init();
	dove_i2c_exp_init(0);
	dove_i2c_exp_init(1);
	dove_sdhci_cam_mbus_init();
	cm_a510_nfc_init();
	cm_a510_clcd_init();
	dove_vmeta_init();
	dove_gpu_init();
	dove_ssp_init(&dove_ssp_platform_data);
	dove_cesa_init();
	dove_hwmon_init();

	dove_i2s_init(0, &i2s0_data);
	/*dove_i2s_init(1, &i2s1_data);*/

        i2c_register_board_info(0, &sound1_i2c_info, 1);

	/*NOTE: On CM-A510 the TWSI bus 2 routed to MPP17,19 is refered by
	        option 1 in MV64XX I2C expander driver*/
	i2c_register_board_info(1, cm_a510_gpio_ext_info, 1);

	spi_register_board_info(cm_a510_spi_flash_info,
				ARRAY_SIZE(cm_a510_spi_flash_info));
	spi_register_board_info(tsc2046_board_info,
				ARRAY_SIZE(tsc2046_board_info));

	/*Turn on CM-A510 LED */
	cm_a510_led_setup(1);
	/*Turn on WLAN */
	cm_a510_wlan_init();
	/* Reset USB hub */
	cm_a510_usb_hub_init();
}

MACHINE_START(CM_A510, "Compulab CM-A510 Board")
	.phys_io	= DOVE_SB_REGS_PHYS_BASE,
	.io_pg_offst	= ((DOVE_SB_REGS_VIRT_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.init_machine	= cm_a510_init,
	.map_io		= dove_map_io,
	.init_irq	= dove_init_irq,
	.timer		= &dove_timer,
/* reserve memory for VMETA and GPU */
	.fixup		= dove_tag_fixup_mem32,
MACHINE_END
