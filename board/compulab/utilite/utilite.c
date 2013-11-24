/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013 CompuLab, Ltd.
 * Copyright (C) 2013 Jasbir
 *
 * Support for Compulab Utilite Pro.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <ipu_pixfmt.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL (PAD_CTL_PUS_100K_UP |                        \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |                     \
        PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |                       \
       PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |                     \
       PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define ENET_PAD_CTRL (PAD_CTL_PUS_100K_UP |                        \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS |                                \
        PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |                \
        PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define USB_HUB_RST IMX_GPIO_NR(7, 8)


iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_KEY_COL0__UART4_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_KEY_ROW0__UART4_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t usdhc1_pads[] = {
	/** Not present but keep so that we match number of mmc devices in uboot environment 
	    on SPI flash **/
	MX6_PAD_SD1_CLK__USDHC1_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL), 
	MX6_PAD_SD1_DAT0__USDHC1_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__USDHC1_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__USDHC1_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__USDHC1_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t usdhc2_pads[] = {
	/** Not present **/
	MX6_PAD_SD2_CLK__USDHC2_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__USDHC2_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__USDHC2_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__USDHC2_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__USDHC2_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	/** No CD pin for micro sd slot **/
    MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads[] = {
    MX6_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_MDC__ENET_MDC   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TXC__ENET_RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD0__ENET_RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD1__ENET_RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD2__ENET_RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD3__ENET_RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RXC__ENET_RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD0__ENET_RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD1__ENET_RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD2__ENET_RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD3__ENET_RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_GPIO_0__CCM_CLKO | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_GPIO_3__CCM_CLKO2 | MUX_PAD_CTRL(NO_PAD_CTRL),
    /* AR8031 PHY Reset */
    MX6_PAD_KEY_ROW4__GPIO_4_15 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usb_pads[] = {
	MX6_PAD_SD3_RST__GPIO_7_8 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef CONFIG_I2C_MXC
struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO_5_27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode =  MX6_PAD_CSI0_DAT8__GPIO_5_26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 26)
	}
};

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO_4_12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO_4_13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO_3__GPIO_1_3 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO_6__GPIO_1_6 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 6)
	}
};
#endif

int dram_init(void)
{
    gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;
    return 0;
}

static void setup_iomux_uart(void)
{
    imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

static struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR},
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
    struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
    int ret = 0;

    switch (cfg->esdhc_base) {
		case USDHC1_BASE_ADDR:
			ret = 1;
			break;
		case USDHC2_BASE_ADDR:
			ret = 1;
			break;
    	case USDHC3_BASE_ADDR:
            ret = 1;
            break;
    }
    return ret;
}

int board_mmc_init(bd_t *bis)
{
    s32 status = 0;
    u32 index = 0;

    for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
        switch (index) {
            case 0:
                imx_iomux_v3_setup_multiple_pads(
                    usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
                usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
                usdhc_cfg[0].max_bus_width = 4;
                break;
            case 1:
                imx_iomux_v3_setup_multiple_pads(
                    usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
                usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
                usdhc_cfg[1].max_bus_width = 4;
                break;
            case 2:
                imx_iomux_v3_setup_multiple_pads(
                    usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
                usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
                usdhc_cfg[2].max_bus_width = 4;
                break;
            default:
                printf("Warning: you configured more USDHC controllers"
                       "(%d) then supported by the board (%d)\n",
                index + 1, CONFIG_SYS_FSL_USDHC_NUM);
                return status;
        }
    	status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}
    return status;
}

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

    /* To enable AR8031 ouput a 125MHz clk from CLK_25M */
    phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
    phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
    phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

    val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
    val &= 0xffe3;
    val |= 0x18;
    phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

    /* introduce tx clock delay */
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
    val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
    val |= 0x0100;
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

    return 0;
}

int board_phy_config(struct phy_device *phydev)
{
        mx6_rgmii_rework(phydev);

        if (phydev->drv->config)
                phydev->drv->config(phydev);

        return 0;
}

static void setup_iomux_enet(void)
{
    imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

    /* Reset AR8031 PHY */
    gpio_direction_output(IMX_GPIO_NR(4, 15) , 0);
    udelay(500);
    gpio_set_value(IMX_GPIO_NR(4, 15), 1);
}

int board_eth_init(bd_t *bis)
{
    int ret;
    setup_iomux_enet();
    ret = cpu_eth_init(bis);
    if (ret)
        printf("FEC MXC: %s:failed\n", __func__);
    return ret;
}

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t const ecspi1_pads[] = {
    /* ECSPI1 */
    MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
    /* SS0 - Serial flash SST25VF016B */
	MX6_PAD_EIM_EB2__GPIO_2_30   | MUX_PAD_CTRL(SPI_PAD_CTRL),  
};

void setup_spi(void)
{
	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
		ARRAY_SIZE(ecspi1_pads));
}
#endif

#ifdef CONFIG_VIDEO_IPUV3
static struct fb_videomode const hdmi = {
    .name           = "HDMI",
    .refresh        = 60,
    .xres           = 1024,
    .yres           = 768,
    .pixclock       = 15385,
    .left_margin    = 220,
    .right_margin   = 40,
    .upper_margin   = 21,
    .lower_margin   = 7,
    .hsync_len      = 60,
    .vsync_len      = 10,
    .sync           = FB_SYNC_EXT,
    .vmode          = FB_VMODE_NONINTERLACED
};

static int detect_hdmi(void)
{
	struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
    return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

int board_video_skip(void)
{
    int ret;
    ret = ipuv3_fb_init(&hdmi, 0, IPU_PIX_FMT_RGB24);

    if (ret)
		printf("HDMI cannot be configured: %d\n", ret);

	if (detect_hdmi())
		imx_enable_hdmi_phy();

	return ret;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0 
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);
}
#endif /* CONFIG_VIDEO_IPUV3 */

#ifdef CONFIG_CMD_SATA
int setup_sata(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;

	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
			IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
			|IOMUXC_GPR13_SATA_PHY_7_SATA2M
			|IOMUXC_GPR13_SATA_SPEED_3G
			|(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			|IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			|IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			|IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			|IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			|IOMUXC_GPR13_SATA_PHY_1_SLOW);

	return 0;
}
#endif

int board_early_init_f(void)
{
    setup_iomux_uart();

#ifdef CONFIG_VIDEO_IPUV3
        setup_display();
#endif

    return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	/* Reset usb hub **/
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));
	gpio_direction_output(USB_HUB_RST, 0);
	mdelay(2);
	gpio_set_value(USB_HUB_RST, 1);
	return 0;
}
#endif

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_init(void)
{
    /* address of boot parameters */
    gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
    setup_spi();
#endif

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

#ifdef CONFIG_CMD_I2C
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#endif

    return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
    /* 4 bit bus width */
    {"sd3",  MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
    /* 8 bit bus width */
//    {"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
    {NULL,         0},
};
#endif

 
int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
    add_board_boot_modes(board_boot_modes);
#endif
    return 0;
}

int checkboard(void)
{
    puts("Board: Utilite\n");
    return 0;
}
