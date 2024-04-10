// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2022 Wesion, Inc. All rights reserved.
 */
#include <common.h>
#include <asm/io.h>
#include <malloc.h>
#include <errno.h>
#include <environment.h>
#include <fdt_support.h>
#include <linux/libfdt.h>
#include <amlogic/cpu_id.h>
#include <asm/arch/secure_apb.h>
#include <asm/arch/pinctrl_init.h>
#include <linux/sizes.h>
#include <asm-generic/gpio.h>
#include <dm.h>
#include <asm/armv8/mmu.h>
#include <amlogic/aml_v3_burning.h>
#include <amlogic/aml_v2_burning.h>
#include <linux/mtd/partitions.h>
#include <asm/arch/bl31_apis.h>
#include <i2c.h>
#ifdef CONFIG_AML_VPU
#include <amlogic/media/vpu/vpu.h>
#endif
#ifdef CONFIG_AML_VPP
#include <amlogic/media/vpp/vpp.h>
#endif
#ifdef CONFIG_AML_VOUT
#include <amlogic/media/vout/aml_vout.h>
#endif
#ifdef CONFIG_AML_HDMITX21
#include <amlogic/media/vout/hdmitx21/hdmitx_module.h>
#endif
#ifdef CONFIG_AML_LCD
#include <amlogic/media/vout/lcd/lcd_vout.h>
#endif
#ifdef CONFIG_RX_RTERM
#include <amlogic/aml_hdmirx.h>
#endif
#include <amlogic/storage.h>
#ifdef CONFIG_POWER_FUSB302
#include <power/fusb302.h>
#endif
#include <amlogic/board.h>


DECLARE_GLOBAL_DATA_PTR;

void sys_led_init(void)
{
}

int serial_set_pin_port(unsigned long port_base)
{
    return 0;
}

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_1_SIZE;
	return 0;
}

/* secondary_boot_func
 * this function should be write with asm, here, is is only for compiling pass
 * */
void secondary_boot_func(void)
{
}

int board_eth_init(bd_t *bis)
{
	return 0;
}

int active_clk(void)
{
	struct udevice *clk = NULL;
	int err;

	err = uclass_get_device_by_name(UCLASS_CLK,
			"xtal-clk", &clk);
	if (err) {
		pr_err("Can't find xtal-clk clock (%d)\n", err);
		return err;
	}
	err = uclass_get_device_by_name(UCLASS_CLK,
			"clock-controller@0", &clk);
	if (err) {
		pr_err("Can't find clock-controller@0 clock (%d)\n", err);
		return err;
	}

	return 0;
}

#ifdef CONFIG_AML_LCD

void board_lcd_detect(void)
{
	int ret = 0;
	int res = 0;
	u8 value = 0;
	uchar linebuf[1];
	struct udevice *bus;
	struct udevice *dev;

	uclass_get_device_by_seq(UCLASS_I2C, 3, &bus);
	ret = i2c_get_chip(bus, 0x38, 1, &dev);
	if (!ret) {
		res = dm_i2c_read(dev, 0xA8, linebuf, 1);
		if (!res) {
			printf("TP05 id=0x%x\n", linebuf[0]);
			if (linebuf[0] == 0x51) {//old ts050
				env_set("panel_type", "mipi_0");
				value = 1;
				printf("panel_type : mipi_0\n");
			} else if (linebuf[0] == 0x79) {//new ts050
				env_set("panel_type", "mipi_2");
				value = 1;
				printf("panel_type : mipi_2\n");
			}
		}
	}
	if (ret || res) {
		ret = i2c_get_chip(bus, 0x14, 1, &dev);
		if (!ret) {
			res = dm_i2c_read(dev, 0x9e, linebuf, 1);
			if (!res) {
				printf("TP10 id=0x%x\n", linebuf[0]);
				if (linebuf[0] == 0x00) {//TS101
					env_set("panel_type", "mipi_1");
					value = 1;
					printf("panel_type : mipi_1\n");
				}
			}
		}
	}

	env_set_ulong("mipi_lcd_exist", value);
	printf("mipi_lcd_exist : %d\n", value);
}
#endif /* CONFIG_AML_LCD */

static void select_fdtfile(void)
{
	cpu_id_t cpu_id;

	cpu_id = get_cpu_id();

	if (cpu_id.chip_rev == 0xA || cpu_id.chip_rev == 0xb) {
		env_set("chip_recv", "b");
		env_set("fdtfile", "amlogic/" CONFIG_DEFAULT_DEVICE_TREE".dtb");

	} else if (cpu_id.chip_rev == 0xC) {
		env_set("chip_recv", "c");
		env_set("fdtfile", "amlogic/" CONFIG_DEFAULT_DEVICE_TREE"n.dtb");
	}
}


#ifdef CONFIG_AML_HDMITX21
static void hdmitx_set_hdmi_5v(void)
{
	/*Power on VCC_5V for HDMI_5V*/
	writel(readl(PADCTRL_GPIOH_OEN) & (~(1 << 1)), PADCTRL_GPIOH_OEN);
	writel(readl(PADCTRL_GPIOH_O) | (1 << 1), PADCTRL_GPIOH_O);
}
#endif
void board_init_mem(void) {
	#if 1
	/* config bootm low size, make sure whole dram/psram space can be used */
	phys_size_t ram_size;
	char *env_tmp;
	env_tmp = env_get("bootm_size");
	if (!env_tmp) {
		ram_size = ((0x100000000 <= ((readl(SYSCTRL_SEC_STATUS_REG4) &
				0xFFFFFFFF0000) << 4)) ? 0xe0000000 :
					(((readl(SYSCTRL_SEC_STATUS_REG4)) &
					0xFFFFFFFF0000) << 4));
		env_set_hex("bootm_low", 0);
		env_set_hex("bootm_size", ram_size);
	}
	#endif
}

int board_init(void)
{
	printf("board init\n");

	/* The non-secure watchdog is enabled in BL2 TEE, disable it */
	run_command("watchdog off", 0);
	printf("watchdog disable\n");

	run_command("gpio set GPIOT_15", 0);//5G reset

#ifdef CONFIG_POWER_FUSB302
	fusb302_init();
#endif

	aml_set_bootsequence(0);
	//Please keep try usb boot first in board_init, as other init before usb may cause burning failure
#if defined(CONFIG_AML_V3_FACTORY_BURN) && defined(CONFIG_AML_V3_USB_TOOl)
	if ((0x1b8ec003 != readl(SYSCTRL_SEC_STICKY_REG2)) && (0x1b8ec004 != readl(SYSCTRL_SEC_STICKY_REG2)))
	{ aml_v3_factory_usb_burning(0, gd->bd); }
#endif//#if defined(CONFIG_AML_V3_FACTORY_BURN) && defined(CONFIG_AML_V3_USB_TOOl)

#if 0 //bypass below operations for pxp
	active_clk();
#endif
	pinctrl_devices_active(PIN_CONTROLLER_NUM);
	/*set vcc5V*/
	run_command("gpio set GPIOH_4", 0);
	run_command("gpio clear GPIOT_15", 0);//5G reset
	run_command("gpio clear GPIOH_3", 0); //pcie reset-gpio

	// FAN testing
	run_command("i2c dev 6", 0);
	run_command("i2c mw 0x18 0x96 1", 0);

	return 0;
}


int board_late_init(void)
{
	printf("board late init\n");

	aml_board_late_init_front(NULL);

	// Set boot source
	board_set_boot_source();

	// Select fdtfile
	select_fdtfile();
#ifdef CONFIG_AML_VPU
	vpu_probe();
#endif
#ifdef CONFIG_AML_HDMITX21
	hdmitx_set_hdmi_5v();
	hdmitx21_chip_type_init(MESON_CPU_ID_T7);
	hdmitx21_init();
#endif
#ifdef CONFIG_AML_VPP
	vpp_init();
#endif
#ifdef CONFIG_RX_RTERM
	rx_set_phy_rterm();
#endif
#ifdef CONFIG_AML_VOUT
	vout_probe();
#endif
#ifdef CONFIG_AML_LCD
	board_lcd_detect();
	lcd_probe();
#endif
	/* The board id is used to determine if the NN needs to adjust voltage */
	switch (readl(SYSCTRL_SEC_STATUS_REG4) >> 8 & 0xff) {
	case 2:
		/* The NN needs to adjust the voltage */
		env_set_ulong("nn_adj_vol", 1);
		break;
	default:
		/* The NN does not need to adjust the voltage */
		env_set_ulong("nn_adj_vol", 0);
	}

	aml_board_late_init_tail(NULL);
	return 0;
}

phys_size_t get_effective_memsize(void)
{
	phys_size_t ddr_size;

	// >>16 -> MB, <<20 -> real size, so >>16<<20 = <<4
#if defined(CONFIG_SYS_MEM_TOP_HIDE)
	ddr_size = (readl(SYSCTRL_SEC_STATUS_REG4) & ~0xffffUL) << 4;
	return (ddr_size - CONFIG_SYS_MEM_TOP_HIDE);
#else
	ddr_size = (readl(SYSCTRL_SEC_STATUS_REG4) & ~0xffffUL) << 4;
	return ddr_size;
#endif /* CONFIG_SYS_MEM_TOP_HIDE */

}

phys_size_t get_ddr_info_size(void)
{
	phys_size_t ddr_size = (((readl(SYSCTRL_SEC_STATUS_REG4)) & ~0xffffUL) << 4);

	return ddr_size;
}

ulong board_get_usable_ram_top(ulong total_size)
{
	unsigned long top = gd->ram_top;

	if (top >= 0xE0000000UL) {
		return 0xE0000000UL;
	}
	return top;
}

static struct mm_region bd_mem_map[] = {
	{
		.virt = 0x00000000UL,
		.phys = 0x00000000UL,
		.size = 0x05100000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0x09000000UL,
		.phys = 0x09000000UL,
		.size = 0x77000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0x80000000UL,
		.phys = 0x80000000UL,
		.size = 0x80000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = bd_mem_map;

int mach_cpu_init(void) {
	//printf("\nmach_cpu_init\n");
#ifdef 	CONFIG_UPDATE_MMU_TABLE
	unsigned long nddrSize = ((0x100000000 <= ((readl(SYSCTRL_SEC_STATUS_REG4) &
				0xFFFFFFFF0000) << 4)) ? 0xe0000000 :
				(((readl(SYSCTRL_SEC_STATUS_REG4)) &
				0xFFFFFFFF0000) << 4));

	if (nddrSize >= 0xe0000000) {
		bd_mem_map[1].size = 0xe0000000UL - bd_mem_map[1].phys;
		bd_mem_map[2].virt = 0xe0000000UL;
		bd_mem_map[2].phys = 0xe0000000UL;
		bd_mem_map[2].size = 0x20000000UL;
	}

#endif
	return 0;
}

int ft_board_setup(void *blob, bd_t *bd)
{
	/* eg: bl31/32 rsv */
	return 0;
}

/* partition table for spinor flash */
#ifdef CONFIG_SPI_FLASH
static const struct mtd_partition spiflash_partitions[] = {
	{
		.name = "env",
		.offset = 0,
		.size = 1 * SZ_256K,
	},
	{
		.name = "dtb",
		.offset = 0,
		.size = 1 * SZ_256K,
	},
	{
		.name = "boot",
		.offset = 0,
		.size = 2 * SZ_1M,
	},
	/* last partition get the rest capacity */
	{
		.name = "user",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
	}
};

const struct mtd_partition *get_spiflash_partition_table(int *partitions)
{
	*partitions = ARRAY_SIZE(spiflash_partitions);
	return spiflash_partitions;
}
#endif /* CONFIG_SPI_FLASH */

#ifdef CONFIG_MESON_NFC
static struct mtd_partition normal_partition_info[] = {
{
	.name = BOOT_BL2E,
	.offset = 0,
	.size = 0,
},
{
	.name = BOOT_BL2X,
	.offset = 0,
	.size = 0,
},
{
	.name = BOOT_DDRFIP,
	.offset = 0,
	.size = 0,
},
{
	.name = BOOT_DEVFIP,
	.offset = 0,
	.size = 0,
},
{
	.name = "logo",
	.offset = 0,
	.size = 2*SZ_1M,
},
{
	.name = "recovery",
	.offset = 0,
	.size = 16*SZ_1M,
},
{
	.name = "boot",
	.offset = 0,
	.size = 16*SZ_1M,
},
{
	.name = "system",
	.offset = 0,
	.size = 64*SZ_1M,
},
/* last partition get the rest capacity */
{
	.name = "data",
	.offset = MTDPART_OFS_APPEND,
	.size = MTDPART_SIZ_FULL,
},
};

struct mtd_partition *get_aml_mtd_partition(void)
{
        return normal_partition_info;
}

int get_aml_partition_count(void)
{
        return ARRAY_SIZE(normal_partition_info);
}

#endif

/* partition table */
/* partition table for spinand flash */
#if (defined(CONFIG_SPI_NAND) || defined(CONFIG_MTD_SPI_NAND))
static const struct mtd_partition spinand_partitions[] = {
	{
		.name = "logo",
		.offset = 0,
		.size = 2 * SZ_1M,
	},
	{
		.name = "recovery",
		.offset = 0,
		.size = 16 * SZ_1M,
	},
	{
		.name = "boot",
		.offset = 0,
		.size = 16 * SZ_1M,
	},
	{
		.name = "system",
		.offset = 0,
		.size = 64 * SZ_1M,
	},
	/* last partition get the rest capacity */
	{
		.name = "data",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
	}
};
const struct mtd_partition *get_partition_table(int *partitions)
{
	*partitions = ARRAY_SIZE(spinand_partitions);
	return spinand_partitions;
}
#endif /* CONFIG_SPI_NAND */

#ifdef CONFIG_MULTI_DTB
phys_size_t get_ddr_memsize(void)
{
	phys_size_t ddr_size = (((readl(SYSCTRL_SEC_STATUS_REG4)) & ~0xffffUL) << 4);

	printf("init board ddr size  %llx\n", ddr_size);
	env_set_hex("board_ddr_size", ddr_size);
	return ddr_size;
}
#endif

int checkhw(char * name)
{
#ifdef CONFIG_MULTI_DTB
	cpu_id_t cpu_id;

	char loc_name[64] = {0};
	phys_size_t ddr_size = get_ddr_memsize();

	cpu_id = get_cpu_id();

	switch (ddr_size) {
	case CONFIG_T7_4G_SIZE:
		if (cpu_id.chip_rev == 0xA || cpu_id.chip_rev == 0xb) {
			#ifdef CONFIG_HDMITX_ONLY
			strcpy(loc_name, "t7_a311d2_an400-hdmitx-only\0");
			#else
			strcpy(loc_name, "t7_a311d2_an400\0");
			#endif
		} else if (cpu_id.chip_rev == 0xC) {
			#ifdef CONFIG_HDMITX_ONLY
			strcpy(loc_name, "t7c_a311d2_an400-hdmitx-only-4g\0");
			#else
			strcpy(loc_name, "t7c_a311d2_an400-4g\0");
			#endif
		}
		break;
	case CONFIG_T7_8G_SIZE:
		if (cpu_id.chip_rev == 0xA || cpu_id.chip_rev == 0xb) {
			strcpy(loc_name, "t7_a311d2_an400\0");
		} else if (cpu_id.chip_rev == 0xC) {
			strcpy(loc_name, "t7c_a311d2_an400-4g\0");
			//
		}
		break;
	default:
		printf("DDR size: 0x%llx, multi-dt doesn't support, ", ddr_size);
		printf("set default t7_a311d2_an400\n");
		if (cpu_id.chip_rev == 0xA || cpu_id.chip_rev == 0xb) {
			strcpy(loc_name, "t7_a311d2_an400\0");
		} else if (cpu_id.chip_rev == 0xC) {
			strcpy(loc_name, "t7c_a311d2_an400-4g\0");
			//
		}
		break;
	}
	printf("init aml_dt to %s\n", loc_name);
	strcpy(name, loc_name);
	env_set("aml_dt", loc_name);
#else
	env_set("aml_dt", "t7_a311d2_an400\0");
#endif
	return 0;
}

const char * const _env_args_reserve_[] =
{
	"lock",
	"upgrade_step",
	"bootloader_version",
	"dts_to_gpt",
	"fastboot_step",
	"reboot_status",
	"expect_index",

	NULL//Keep NULL be last to tell END
};

int __attribute__((weak)) mmc_initialize(bd_t *bis){ return 0;}

int __attribute__((weak)) do_bootm(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]){ return 0;}

void __attribute__((weak)) set_working_fdt_addr(ulong addr) {}

int __attribute__((weak)) ofnode_read_u32_default(ofnode node, const char *propname, u32 def) {return 0;}

void __attribute__((weak)) md5_wd (unsigned char *input, int len, unsigned char output[16],	unsigned int chunk_sz){}

#ifdef CONFIG_AML_PCIE
#include <asm/arch-t7/pci.h>

void amlogic_pcie_init_reset_pin(int pcie_dev)
{
	mdelay(5);
	run_command("gpio set GPIOH_3", 0); //pcie reset-gpio

}

void amlogic_pcie_disable(void)
{
	run_command("gpio clear GPIOH_3", 0); //pcie reset-gpio
}
#endif
