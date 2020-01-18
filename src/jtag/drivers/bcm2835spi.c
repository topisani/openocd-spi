#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"

static bb_value_t bcm2835spi_read(void);
static int bcm2835spi_write(int tck, int tms, int tdi);
static int bcm2835spi_reset(int trst, int srst);

static int bcm2835_swdio_read(void);
static void bcm2835_swdio_drive(bool is_output);

static int bcm2835spi_init(void);
static int bcm2835spi_quit(void);

static struct bitbang_interface bcm2835spi_bitbang = {
	.read = bcm2835spi_read,
	.write = bcm2835spi_write,
	.reset = bcm2835spi_reset,
	.swdio_read = bcm2835_swdio_read,
	.swdio_drive = bcm2835_swdio_drive,
	.blink = NULL
};

/* GPIO numbers for each signal. Negative values are invalid */
static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int trst_gpio = -1;
static int srst_gpio = -1;
static int swclk_gpio = -1;
static int swdio_gpio = -1;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static unsigned int jtag_delay;

static bb_value_t bcm2835spi_read(void)
{
	//  TODO
	return 0;
}

static int bcm2835spi_write(int tck, int tms, int tdi)
{
	//  TODO
	return ERROR_OK;
}

static int bcm2835spi_swd_write(int tck, int tms, int tdi)
{
	//  TODO
	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int bcm2835spi_reset(int trst, int srst)
{
	//  TODO
	return ERROR_OK;
}

static void bcm2835_swdio_drive(bool is_output)
{
	//  TODO
}

static int bcm2835_swdio_read(void)
{
	//  TODO
	return 0;
}

static int bcm2835spi_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = speed_coeff/khz - speed_offset;
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int bcm2835spi_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int bcm2835spi_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"BCM2835 GPIO config: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "BCM2835 GPIO config: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "BCM2835 GPIO config: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "BCM2835 GPIO config: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "BCM2835 GPIO config: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "BCM2835 GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "BCM2835 GPIO config: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"BCM2835 GPIO nums: swclk = %d, swdio = %d",
			swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "BCM2835 num: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "BCM2835 num: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "BCM2835 GPIO: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_peripheral_base)
{
	//  TODO
	return ERROR_OK;
}

static const struct command_registration bcm2835spi_command_handlers[] = {
	{
		.name = "bcm2835spi_jtag_nums",
		.handler = &bcm2835spi_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "bcm2835spi_tck_num",
		.handler = &bcm2835spi_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "bcm2835spi_tms_num",
		.handler = &bcm2835spi_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "bcm2835spi_tdo_num",
		.handler = &bcm2835spi_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "bcm2835spi_tdi_num",
		.handler = &bcm2835spi_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "bcm2835spi_swd_nums",
		.handler = &bcm2835spi_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "bcm2835spi_swclk_num",
		.handler = &bcm2835spi_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "bcm2835spi_swdio_num",
		.handler = &bcm2835spi_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	{
		.name = "bcm2835spi_srst_num",
		.handler = &bcm2835spi_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "bcm2835spi_trst_num",
		.handler = &bcm2835spi_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "bcm2835spi_speed_coeffs",
		.handler = &bcm2835spi_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "bcm2835spi_peripheral_base",
		.handler = &bcm2835spi_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (RPi1 0x20000000, RPi2 0x3F000000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const char * const bcm2835_transports[] = { "swd", NULL };

struct jtag_interface bcm2835spi_interface = {
	.name = "bcm2835spi",
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
	.transports = bcm2835_transports,
	.swd = &bitbang_swd,
	.speed = bcm2835spi_speed,
	.khz = bcm2835spi_khz,
	.speed_div = bcm2835spi_speed_div,
	.commands = bcm2835spi_command_handlers,
	.init = bcm2835spi_init,
	.quit = bcm2835spi_quit,
};

static bool bcm2835spi_swd_mode_possible(void)
{
	return 1;
}

static int bcm2835spi_init(void)
{
	bitbang_interface = &bcm2835spi_bitbang;

	LOG_INFO("BCM2835 SPI SWD driver");

	if (bcm2835spi_swd_mode_possible()) {
		LOG_INFO("SWD only mode enabled");
	} else {
		LOG_ERROR("NOT SUPPORTED: Require tck, tms, tdi and tdo gpios for JTAG mode and/or swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_DEBUG("saved pinmux settings");

	if (swd_mode) {
		bcm2835spi_bitbang.write = bcm2835spi_swd_write;
		bitbang_switch_to_swd();
	}

	return ERROR_OK;
}

static int bcm2835spi_quit(void)
{
	//  TODO
	return ERROR_OK;
}
