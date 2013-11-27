/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013 CompuLab, Ltd.
 * Copyright (C) 2013 Jasbir
 *
 * Configuration settings for the Compulab Utilite Pro.
 *
 * SPDX-License-Identifier:        GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <asm/sizes.h>

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART4_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR

#define CONFIG_BOOTDELAY		5

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			    0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

/* MMC Configuration */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* Ethernet Configuration */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC0"
#define CONFIG_FEC_MXC_PHYADDR		0
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* SPI NOR Flash **/
#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SST
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   (0|(IMX_GPIO_NR(2, 30)<<8))
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

/* SATA Config */
#define CONFIG_CMD_SATA
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE       1
#define CONFIG_DWC_AHSATA_PORT_ID        0
#define CONFIG_DWC_AHSATA_BASE_ADDR      SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/* I2C Config */
#define CONFIG_CMD_I2C
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE

/* Framebuffer */
#undef CONFIG_VIDEO /** Turn it on/off as required **/
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_CFB_CONSOLE
#define CONFIG_CFB_CONSOLE_ANSI
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_IMX_HDMI

#undef CONFIG_SPLASH_SCREEN
#undef CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_CONSOLE_MUX
#endif

/* USB Configs */
#define CONFIG_CMD_USB /** Turn it on/off as required **/
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_EHCI_IS_TDI
#define CONFIG_USB_STORAGE
#define CONFIG_MXC_USB_PORT     1
#define CONFIG_MXC_USB_PORTSC   (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS    0
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_PREBOOT "usb start"
#endif

#define	CONFIG_EXTRA_ENV_SETTINGS \
	"autoload=no\0" \
	"loadaddr=0x10800000\0" \
	"console=ttymxc3,115200\0" \
	"ethprime=FEC0\0" \
	"kernel=uImage\0" \
	"bootscr=boot.scr\0" \
	"video_hdmi=mxcfb0:dev=hdmi,1280x720-24M@60,if=RGB24\0" \
	"video_dvi=mxcfb0:dev=dvi,1280x800-24M@60,if=RGB24\0" \
	"mmcdev=0\0" \
	"mmcroot=/dev/mmcblk0p2 rw rootwait" \
	"nandroot=/dev/mtdblock4 rw\0" \
	"nandrootfstype=ubifs\0" \
	"mmcargs=setenv bootargs console=${console} " \
		"root=${mmcroot} " \
		"${video}\0" \
	"nandargs=setenv bootargs console=${console} " \
		"root=${nandroot} " \
		"rootfstype=${nandrootfstype} " \
		"${video}\0" \
	"loadbootscript=fatload mmc ${mmcdev} ${loadaddr} ${bootscr}\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source ${loadaddr}\0" \
	"loadkernel=fatload mmc ${mmcdev} ${loadaddr} ${kernel}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"bootm ${loadaddr}\0" \
	"nandboot=echo Booting from nand ...; " \
		"run nandargs; " \
		"nand read ${loadaddr} 0 400000; " \
		"bootm ${loadaddr}\0" \


#define CONFIG_BOOTCOMMAND \
        "mmc dev ${mmcdev}; " \
	"if mmc rescan; then " \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"if run loadkernel; then " \
				"run mmcboot; " \
			"else run nandboot; " \
			"fi; " \
		"fi; " \
	"else " \
		"run nandboot; " \
	"fi"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "U-Boot >"
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING

/** Need to put this here for now ... not the right place **/
#define CONFIG_NR_DRAM_BANKS    2

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define PHYS_SDRAM_1            MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_2            MMDC1_ARB_BASE_ADDR

#define iomem_valid_addr(addr, size) \
 ((addr >= PHYS_SDRAM_1 && addr <= (PHYS_SDRAM_1 + PHYS_SDRAM_1_SIZE)) || \
  (addr >= PHYS_SDRAM_2 && addr <= (PHYS_SDRAM_2 + PHYS_SDRAM_2_SIZE)))

#define PHYS_SDRAM_1_SIZE       (1 << 30)       /* 1GB */
#define PHYS_SDRAM_2_SIZE       (1 << 30)       /* 1GB */

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/** FLASH and environment organization **/
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SECT_SIZE	(8 * 1024)
#define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE

/* Default is to read uboot env from SPI Flash */
#define CONFIG_ENV_IS_IN_SPI_FLASH
/* Turn this on to read uboot env from SD card */
#undef CONFIG_ENV_IS_IN_MMC

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET                (6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV                2
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET                (768 * 1024)
#define CONFIG_ENV_SECT_SIZE             (8 * 1024)
#define CONFIG_ENV_SPI_BUS               CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS                CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE              CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ            CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif /* __CONFIG_H * */
