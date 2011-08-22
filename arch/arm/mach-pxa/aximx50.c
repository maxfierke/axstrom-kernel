/*
 * Hardware definitions for Dell Axim X50/51(v)
 *
 * Copyright (c) 2009 Ertan Deniz
 * Copyright (c) 2010-2011 Paul Burton <paulburton89@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/plat-ram.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <mach/pxa2xx_spi.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/i2c.h>

#include <mach/aximx50.h>
#include <mach/pxa2xx-regs.h>
#include <mach/mfp-pxa27x.h>
#include <mach/pxa27x_keypad.h>
#include <mach/pxafb.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/udc.h>
#include <mach/regs-ost.h>
#include <mach/regs-lcd.h>

#include <video/mbxfb.h>

#include "generic.h"
#include "devices.h"

/****************
 * Init Machine *
 ****************/

static unsigned long aximx50_pin_config[] __initdata = {

	/* BTUART */
	GPIO42_BTUART_RXD,
	GPIO43_BTUART_TXD,
	GPIO44_BTUART_CTS,
	GPIO45_BTUART_RTS,

	/* STUART */
	GPIO46_STUART_RXD,
	GPIO47_STUART_TXD,

	/* MCI controller */
	GPIO32_MMC_CLK,
	GPIO112_MMC_CMD,
	GPIO92_MMC_DAT_0,
	GPIO109_MMC_DAT_1,
	GPIO110_MMC_DAT_2,
	GPIO111_MMC_DAT_3,

	/* LCD */
	GPIO58_LCD_LDD_0,
	GPIO59_LCD_LDD_1,
	GPIO60_LCD_LDD_2,
	GPIO61_LCD_LDD_3,
	GPIO62_LCD_LDD_4,
	GPIO63_LCD_LDD_5,
	GPIO64_LCD_LDD_6,
	GPIO65_LCD_LDD_7,
	GPIO66_LCD_LDD_8,
	GPIO67_LCD_LDD_9,
	GPIO68_LCD_LDD_10,
	GPIO69_LCD_LDD_11,
	GPIO70_LCD_LDD_12,
	GPIO71_LCD_LDD_13,
	GPIO72_LCD_LDD_14,
	GPIO73_LCD_LDD_15,
	GPIO74_LCD_FCLK,
	GPIO75_LCD_LCLK,
	GPIO76_LCD_PCLK,
	GPIO77_LCD_BIAS,

	/* I2C */
	GPIO117_I2C_SCL,
	GPIO118_I2C_SDA,

	/* SSP1 */
	GPIO23_SSP1_SCLK,
	GPIO24_GPIO, /* GPIO_NR_X50_TSC2046_CS - SFRM as chip select */
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,

	/* I2S */
	GPIO28_I2S_BITCLK_OUT,
	GPIO29_I2S_SDATA_IN,
	GPIO30_I2S_SDATA_OUT,
	GPIO31_I2S_SYNC,
	GPIO113_I2S_SYSCLK,

	/* PC Card */
	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO50_nPIOR,
	GPIO51_nPIOW,
	GPIO102_nPCE_1,
	GPIO54_nPCE_2,
	GPIO55_nPREG,
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,
	GPIO89_GPIO,	/* WiFi Reset */
	GPIO107_GPIO,   /* WiFi Ready */

	/* SDRAM and local bus */
	GPIO15_nCS_1,
	GPIO78_nCS_2,
	GPIO79_nCS_3,
	GPIO80_nCS_4,
	GPIO33_nCS_5,
	GPIO18_RDY,
	
	/* Keys */
	GPIO93_KP_DKIN_0,
	GPIO100_KP_MKIN_0,
	GPIO101_KP_MKIN_1,
	GPIO97_KP_MKIN_3,
	GPIO98_KP_MKIN_4,
	GPIO90_KP_MKIN_5,
	GPIO91_KP_MKIN_6,
	GPIO103_KP_MKOUT_0,
	GPIO105_KP_MKOUT_2,
};

#define MACHINE_X50  0x00
#define MACHINE_X51  0x10
#define MACHINE_QVGA 0x00
#define MACHINE_VGA  0x01

static u32 machine_type;
static u32 lcd_type;

/*
 * FPGA
 */

static u16 *aximx50_fpga;
static u16 aximx50_fpga_cache[16] = { 0 };

void aximx50_fpga_set(uint offset, u16 val)
{
	aximx50_fpga_cache[offset / sizeof(u16)] |= val;
	aximx50_fpga[offset / sizeof(u16)] = aximx50_fpga_cache[offset / sizeof(u16)];
}
EXPORT_SYMBOL(aximx50_fpga_set);

void aximx50_fpga_clear(uint offset, u16 val)
{
	aximx50_fpga_cache[offset / sizeof(u16)] |= val;
	aximx50_fpga[offset / sizeof(u16)] = aximx50_fpga_cache[offset / sizeof(u16)];
}
EXPORT_SYMBOL(aximx50_fpga_clear);

u16 aximx50_fpga_read(uint offset)
{
	return aximx50_fpga[offset / sizeof(u16)];
}

static void aximx50_init_fpga(void)
{
	if (!(aximx50_fpga = ioremap_nocache(PXA_CS4_PHYS, 0x20))) {
		printk(KERN_ERR "Unable to map FPGA!\n");
		return;
	}

	aximx50_fpga_set(0x14, 0x8);
	aximx50_fpga_set(0x16, 0x0);
	aximx50_fpga_set(0x1c, 0x6);
}

/*
 * Machine Detection
 */

void aximx50_detect_machine(void)
{
	void __iomem *docregs_iobase;
	__u16 id0, id1;

	machine_type = 0;

	/* First we'll see if we have a DOC G3 */
	docregs_iobase = ioremap_nocache(0x00201000, 0x7FF);
	if (docregs_iobase) {
		readb(docregs_iobase + 0x0c);
		writeb(0x5, docregs_iobase + 0x0c);
		wmb();
		writeb(0xA, docregs_iobase + 0x72);
		wmb();

		writew(0x1000, docregs_iobase + 0x1a);
		wmb();
		id0 = readw(docregs_iobase + 0x00);

		writew(0x1074, docregs_iobase + 0x1a);
		wmb();
		id1 = readw(docregs_iobase + 0x74);

		printk(KERN_DEBUG "DOC ID0 = 0x%04x\n", id0);
		printk(KERN_DEBUG "DOC ID1 = 0x%04x\n", id1);

		if (id0 == 0x0200 && id1 == 0xfdff) {
			/* The device contains a DOC G3 flash chip */
			/* Thus, it's an X51(v) */

			machine_type |= MACHINE_X51;
		}

		iounmap(docregs_iobase);
	}
	else {
		printk(KERN_ERR "Unable to map DOC register IO space! Assuming X50\n");
	}

	/* Now check the LCD type */
	aximx50_fpga_set(0x1E, 0x8);
	udelay(1000);
	lcd_type = aximx50_fpga_read(0x1E) & 3;
	udelay(1000);
	aximx50_fpga_clear(0x1E, 0x8);
	
	/*
		From AximSDK.dll, it seems:

		0 = 3.7" Sharp
		1 = 3.5" Sharp
		2 = 3.7" Samsung
		3 = 3.5" Sharp
	*/

	printk(KERN_DEBUG "Detected display: ");
	printk(lcd_type % 2 ? "3.5\" " : "3.7\" ");
	printk(lcd_type == 2 ? "Samsung\n" : "Sharp\n");

	if ((lcd_type % 2) == 0)
		machine_type |= MACHINE_VGA;

	printk(KERN_DEBUG "Detected machine: X5%d%s (0x%02x)\n",
	       (machine_type & MACHINE_X51) ? 1 : 0,
	       (machine_type & MACHINE_VGA) ? "v" : "",
	       machine_type);
}

/******************************************************************************
 * SD/MMC card controller
 ******************************************************************************/
static void mci_setpower(struct device *dev, unsigned int vdd)
{
	struct pxamci_platform_data *p_d = dev->platform_data;

	if ((1 << vdd) & p_d->ocr_mask)
		aximx50_fpga_set(0x16, 0x01);
	else
		aximx50_fpga_clear(0x16, 0x01);
}

static struct pxamci_platform_data aximx50_mci_platform_data = {
	.detect_delay_ms    = 250,
	.ocr_mask           = MMC_VDD_32_33 | MMC_VDD_33_34,
	.gpio_card_detect   = GPIO_NR_AXIMX50_SD_DETECT,
	.gpio_card_ro       = GPIO_NR_AXIMX50_SD_READONLY,
	.gpio_power         = -1,
	.setpower           = mci_setpower,
};

/****************************************************************
 * Keyboard
 ****************************************************************/
static unsigned int x50_key_map[] = {
	KEY(0, 0, KEY_WLAN),
	KEY(0, 2, KEY_UP),

	KEY(1, 0, KEY_RECORD),
	KEY(1, 2, KEY_DOWN),

	KEY(3, 0, KEY_CALENDAR),
	KEY(3, 2, KEY_RIGHT),

	KEY(4, 0, KEY_ADDRESSBOOK), // contacts
	KEY(4, 2, KEY_LEFT),

	KEY(5, 0, KEY_MENU), // mail
	KEY(5, 2, KEY_ENTER),

	KEY(6, 0, KEY_BACK), // home
};

static struct pxa27x_keypad_platform_data x50_kbd = {
	.matrix_key_rows = 7,
	.matrix_key_cols = 3,
	.matrix_key_map = x50_key_map,
	.matrix_key_map_size = ARRAY_SIZE(x50_key_map),

	.direct_key_num = 1,
	//.direct_key_map = { KEY_POWER, 0, 0, 0, 0, 0, 0, 0 },
	.direct_key_map = { 0, 0, 0, 0, 0, 0, 0, 0 },
};

/*
  GPIO Keys
*/

static struct gpio_keys_button aximx50_gpio_buttons[] = {
	{
		.desc = "Power",
		.gpio = GPIO_NR_AXIMX50_BTN_POWER,
		.code = KEY_SUSPEND,
		.type = EV_PWR,
		.wakeup = 1,
	},
};

static struct gpio_keys_platform_data aximx50_gpio_keys_platform_data = {
	.buttons = aximx50_gpio_buttons,
	.nbuttons = ARRAY_SIZE(aximx50_gpio_buttons),
};

static struct platform_device aximx50_gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &aximx50_gpio_keys_platform_data,
	},
};

/****************************************************************
 * USB Gadget
 ****************************************************************/

static struct pxa2xx_udc_mach_info x50_udc_info = {
	.gpio_vbus_inverted = false,
	.gpio_vbus = GPIO_NR_X50_USB_CABLE_DETECT,
//  .gpio_pullup =
};

/*
 * Touchscreen
 */

/* ADS7846 is connected through SSP ... */
static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect = 3, /* Matches the number of chips attached to NSSP */
//  .clock_enable = CKEN_SSP1,
	.enable_dma = 1,
};

static void ads7846_cs(u32 command)
{
	gpio_set_value(GPIO_NR_X50_TSC2046_CS, !(command == PXA2XX_CS_ASSERT));
}

static struct pxa2xx_spi_chip ads_hw = {
//  .tx_threshold	   = 12,
//  .rx_threshold	   = 12,
//  .dma_burst_size	 = 8,
	.cs_control     = ads7846_cs,
};

static struct ads7846_platform_data aximx50_ts_info = {
	.model              = 7846,
	.vref_delay_usecs   = 100,  /* internal, no capacitor */
	.settle_delay_usecs = 150,
	.x_min              = 1,
	.x_max              = 480,
	.y_min              = 1,
	.y_max              = 640,
	.swap_xy	        = 1,
	.pressure_min       = 1,
	.pressure_max       = 512,
	.debounce_max       = 20,
	.debounce_tol       = 10,
	.debounce_rep       = 1,
	.gpio_pendown       = GPIO_NR_X50_PEN_IRQ_N,
};

static struct spi_board_info __initdata aximx50_boardinfo[] = { {
	.modalias         = "ads7846",
	.platform_data    = &aximx50_ts_info,
	.controller_data  = &ads_hw,
	.irq              = X50_IRQ(PEN_IRQ_N),
	.max_speed_hz     = 100000 /* max sample rate at 3V */
	                    * 26 /* command + data + overhead */,
	.bus_num          = 1,
	.chip_select      = 0,
} };

/*
 * PXA Framebuffer
 */

static struct pxafb_mode_info aximx50_pxafb_modes_vga[] = {
	{
		.pixclock	= 19230,
		.bpp		= 16,
		.xres		= 480,
		.yres		= 640,
		.hsync_len	= 64,
		.vsync_len	= 2,
		.left_margin	= 17,
		.upper_margin	= 1,
		.right_margin	= 87,
		.lower_margin   = 4,
	},
};

static struct pxafb_mach_info aximx50_fb_info_vga = {
	.modes		= aximx50_pxafb_modes_vga,
	.num_modes	= ARRAY_SIZE(aximx50_pxafb_modes_vga),
	.lccr0		= LCCR0_Act | LCCR0_RDSTM | LCCR0_CMDIM,
	.lccr3		= 0x04f00001,
};

static struct pxafb_mode_info aximx50_pxafb_modes_qvga[] = {
	{
		.pixclock	= 134615,
		.bpp		= 16,
		.xres		= 240,
		.yres		= 320,
		.hsync_len	= 20,
		.vsync_len	= 1,
		.left_margin	= 21,
		.upper_margin	= 8,
		.right_margin	= 51,
		.lower_margin	= 1,
	},
};

static struct pxafb_mach_info aximx50_fb_info_qvga = {
	.modes		= aximx50_pxafb_modes_qvga,
	.num_modes	= ARRAY_SIZE(aximx50_pxafb_modes_qvga),
	.lccr0		= LCCR0_Act | LCCR0_RDSTM | LCCR0_CMDIM,
	.lccr3		= 0x04f00007,
};

/*
 * Intel 2700g (Marathon)
 */

#ifdef CONFIG_AXIMX50_VIDEOMTD
static struct resource aximx50_vram_resources = {
	.start = PXA_CS3_PHYS,
	.end = PXA_CS3_PHYS + SZ_16M-1,
	.flags = IORESOURCE_MEM,
};

static struct platdata_mtd_ram aximx50_vram_pdata = {
	.mapname = "2700g Video SDRAM",
	.bankwidth = 2,
};

static struct platform_device aximx50_vram = {
	.name = "mtd-ram",
	.id = 0,
	.resource = &aximx50_vram_resources,
	.num_resources = 1,
	.dev = {
		.platform_data = &aximx50_vram_pdata,
	},
};
#endif

#if defined(CONFIG_FB_MBX) || defined(CONFIG_FB_MBX_MODULE)
static u64 fb_dma_mask = ~(u64)0;

static struct resource aximx50_2700G_resource[] = {
	/* frame buffer memory including ODFB and External SDRAM */
	[0] = {
		.start = PXA_CS3_PHYS,
		.end   = PXA_CS3_PHYS + 0x01ffffff,
		.flags = IORESOURCE_MEM,
	},
	/* Marathon registers */
	[1] = {
		.start = PXA_CS3_PHYS + 0x03fe0000,
		.end   = PXA_CS3_PHYS + 0x03ffffff,
		.flags = IORESOURCE_MEM,
	},
};

static int aximx50_marathon_probe(struct fb_info *fb)
{
	return 0;
}

static int aximx50_marathon_remove(struct fb_info *fb)
{
	return 0;
}

static struct mbxfb_platform_data aximx50_2700G_data = {
	.xres = {
		.min = 240,
		.max = 1200,
		.defval = 480,
	},
	.yres = {
		.min = 240,
		.max = 1200,
		.defval = 640,
	},
	.bpp = {
		.min = 16,
		.max = 32,
		.defval = 16,
	},
	.memsize = 16*1024*1024,
	.probe = aximx50_marathon_probe,
	.remove = aximx50_marathon_remove,
};

static struct platform_device aximx50_2700G = {
	.name		= "mbx-fb",
	.dev		= {
		.platform_data	= &aximx50_2700G_data,
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(aximx50_2700G_resource),
	.resource	= aximx50_2700G_resource,
	.id		= -1,
};
#endif

#define lcd_readl(off) __raw_readl(lcd_iobase + off)
#define lcd_writel(off,val) __raw_writel(val, lcd_iobase + off)

#define MBX_SYSRST    (marathon_iobase + 0x10)
#define MBX_LCDCFG    (marathon_iobase + 0x60)
#define MBX_GPIOCFG   (marathon_iobase + 0x6c)

#define MBX_LCDCFG_LCD1ISIN 0x00010000
#define MBX_LCDCFG_IN565    0x20000000

static void __init aximx50_init_display(void)
{
	void __iomem *marathon_iobase;

#if defined(CONFIG_FB_MBX) || defined(CONFIG_FB_MBX_MODULE)
	if (lcd_type % 2 == 0) {
		printk(KERN_DEBUG "Using Intel 2700g (Marathon)\n");
		
		platform_device_register(&aximx50_2700G);
	}
	else {
#endif

	if (lcd_type % 2) {
		printk(KERN_DEBUG "Using PXA Framebuffer (QVGA)\n");
		set_pxa_fb_info(&aximx50_fb_info_qvga);
	}
	else {
		printk(KERN_DEBUG "Using PXA Framebuffer (VGA)\n");

		/* Setup the 2700g to pass LCD_IN (PXA LCD controller)
		   through to LCD1 (built in LCD) */

		if ((marathon_iobase = ioremap(0x0FFE0000, 0x1FFFF))) {
			printk(KERN_DEBUG "Mapped 2700G Registers to 0x%08lx\n", (ulong)marathon_iobase);
			printk(KERN_DEBUG "  LCD_CONFIG=0x%08x\n", readl(MBX_LCDCFG));
				
			writel(1, MBX_SYSRST);
			mdelay(10);

			writel(readl(MBX_LCDCFG) | MBX_LCDCFG_LCD1ISIN | MBX_LCDCFG_IN565,
			       MBX_LCDCFG); /* LCD1 is LCDIN */

			writel(0x00000001, MBX_GPIOCFG);

			iounmap(marathon_iobase);
		}
		else {
			printk(KERN_ERR "Unable to map 2700G registers\n");
		}

		set_pxa_fb_info(&aximx50_fb_info_vga);
	}

#if defined(CONFIG_FB_MBX) || defined(CONFIG_FB_MBX_MODULE)
	}
#endif
}




/******************************************************/

static struct platform_device *devices[] __initdata = {
	//&aximx50_bt,
	&aximx50_gpio_keys_device,
};

/*
 * I2C
 */

static struct i2c_pxa_platform_data aximx50_i2c_info = {
	.fast_mode = 1,
	.use_pio = 0,
};

static void __init aximx50_map_io(void)
{
	pxa_map_io();
}

static void __init aximx50_init_irq(void)
{
	pxa27x_init_irq();
}

static void __init aximx50_init( void )
{
	printk(KERN_NOTICE "Dell Axim x50/x51v initialization\n");
	
	pxa2xx_mfp_config(ARRAY_AND_SIZE(aximx50_pin_config));
	
	if (gpio_request(GPIO_NR_X50_TSC2046_CS, "ADS7846_CS"))
		printk(KERN_ERR "Unable to get CS GPIO\n");
	else
	    gpio_direction_output(GPIO_NR_X50_TSC2046_CS, 1);
	    
	aximx50_init_fpga();
	aximx50_detect_machine();
	aximx50_init_display();
	
	pxa_set_i2c_info(&aximx50_i2c_info);

	/* Swap touchscreen co-ordinates on X51(v), but not X50(v) */
	aximx50_ts_info.swap_xy = !!(machine_type & MACHINE_X51);
	
	pxa2xx_set_spi_info(1, &pxa_ssp_master_info);
	spi_register_board_info(aximx50_boardinfo, ARRAY_SIZE(aximx50_boardinfo));
	
	pxa_set_mci_info(&aximx50_mci_platform_data);
	pxa_set_keypad_info(&x50_kbd);

	/* USB power? */
	aximx50_fpga_set(0x1c, 0x8);

	pxa_set_udc_info(&x50_udc_info);
	
	platform_add_devices(devices, ARRAY_SIZE(devices));
}


MACHINE_START(X50, "Axim x50/x51(v)")
	.phys_io = 0x40000000,
	.io_pg_offst = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params = 0xa8000100,
	.map_io = aximx50_map_io,
	.init_irq = aximx50_init_irq,
	.timer = &pxa_timer,
	.init_machine = aximx50_init,
MACHINE_END

