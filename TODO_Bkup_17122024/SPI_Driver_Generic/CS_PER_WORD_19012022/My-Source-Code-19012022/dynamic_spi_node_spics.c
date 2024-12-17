#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

static unsigned short mode;
module_param(mode, ushort, 0);
MODULE_PARM_DESC(mode, "i=3 for PNP Tuner chip, =2 for mercury spi node, =1 for DAB spi node, =0 for SWDL/HDR spi node");

static struct spi_board_info spi_device_info_spidev = {
	.max_speed_hz = 1500000, /* 1.5MHz */
	.bus_num = 1,
	.chip_select = 0,
	.mode = SPI_MODE_1,
	.modalias = "spidev",
};

static struct spi_board_info spi_device_info_dab = {
	.max_speed_hz = 20000000, /* 20MHz */
	.bus_num = 1,
	.chip_select = 0,
	.mode = SPI_MODE_1,
	.modalias = "dabplugin",
};

/* Dynamic SPI node for Mercury */
static struct spi_board_info spi_device_info_mercury = {
	.max_speed_hz = 1500000, /* 1.5MHz */
	.bus_num = 1,
	.chip_select = 0,
	.mode = SPI_MODE_1,
	.modalias = "spidev",
};

/* Dynamic SPI node for PNP */
static struct spi_board_info spi_device_info_pnp = {
	.max_speed_hz = 1500000, /* 1.5MHz */
	.bus_num = 1,
	.chip_select = 1,   /*MIR, changed 0 to 1 on 18-01-2022*/
	.mode = SPI_MODE_1,
	.modalias = "pnp_dabplugin",
};

static struct spi_device *spi_device;

static int spi_init(void)
{
	struct spi_master *master;
	int ret;
	struct spi_board_info spidev_info;

	switch (mode) {
	case 0:
		spidev_info = spi_device_info_spidev;
		break;

	case 1:
		spidev_info = spi_device_info_dab;
		break;

	case 2:
		spidev_info = spi_device_info_mercury;
		break;

	case 3:
		spidev_info = spi_device_info_pnp;
		break;
	default:
		pr_err("module param mode not correct. Insert driver with correct mode\n");
		return -EPERM;
	}
	pr_info("bus_num:%d, mode:%d, chip_select:%d, max_speed_hz:%d =>\n",
		spidev_info.bus_num, spidev_info.mode,
		spidev_info.chip_select,
		spidev_info.max_speed_hz);

	master = spi_busnum_to_master(spidev_info.bus_num);
	if (!master)
		return -ENODEV;

	spi_device = spi_new_device(master, &spidev_info);
	if (!spi_device)
		return -ENODEV;

	ret = spi_setup(spi_device);
	if (ret)
		spi_unregister_device(spi_device);

	return ret;
}
module_init(spi_init);

static void spi_exit(void)
{
	spi_unregister_device(spi_device);
}
module_exit(spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vikram N <vikram.narayanarao@harman.com>");
MODULE_DESCRIPTION("Driver for dynamic SPI device node switching");
