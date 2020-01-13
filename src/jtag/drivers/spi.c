/***************************************************************************
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/
//  Implement SWD protocol with Raspberry Pi Bidirectional SPI (luppy@appkaki.com)
//  Why implement SWD over Bidirectional SPI on Raspberry Pi?  Because SWD over Bit-Banging GPIO has timing issues that affect OpenOCD flashing...
//  https://gist.github.com/lupyuen/18e66c3e81e11050a10d1192c5b84bb0

//  Based on https://raw.githubusercontent.com/raspberrypi/linux/rpi-3.10.y/Documentation/spi/spidev_test.c
//  SWD Protocol: https://annals-csis.org/proceedings/2012/pliks/279.pdf
//  See also: https://github.com/MarkDing/swd_programing_sram
//  Pi SPI Hardware: https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md
//  Pi SPI Kernel Driver: https://github.com/raspberrypi/linux/blob/rpi-3.12.y/drivers/spi/spi-bcm2708.c
//  BCM2835 Peripherals Datasheet: https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf
//  SWD mapped to SPI bytes: https://docs.google.com/spreadsheets/d/12oXe1MTTEZVIbdmFXsOgOXVFHCQnYVvIw6fRpIQZybg/edit#gid=0

//  To test:
//  Connect SWDIO to MOSI (Pin P1-19, Yellow)
//  Connect SWDCLK to SCLK (Pin P1-23, Blue)
//  Connect 3.3V and GND
//  sudo raspi-config
//  Interfacing Options --> SPI --> Yes
//  Finish --> Yes
#define SWD_SPI  //  Transmit and receive SWD commands over SPI...
#ifdef SWD_SPI	

#include <jtag/swd.h>

static void spi_exchange_transmit(uint8_t buf[], unsigned int offset, unsigned int bit_cnt);
static void spi_exchange_receive(uint8_t buf[], unsigned int offset, unsigned int bit_cnt);

///  Transmit or receive bit_cnt number of bits from/into buf starting at the bit offset.
///  If rnw is false: Transmit from host to target.
///  If rnw is true:  Receive from target to host.
void spi_exchange(bool rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
    if (!buf) { pabort("spi_exchange: null buffer"); }
    if (bit_cnt == 0) { return; }
    //  If rnw is true, receive from target to host. Else transmit from host to target.
    if (rnw) {
        spi_exchange_receive(buf, offset, bit_cnt);
    } else {
        spi_exchange_transmit(buf, offset, bit_cnt);
    }
    //  If bit_cnt is a multiple of 8, then we have a round number of bytes sent/received, no problem. 
    //  Else we got trailing undefined bits that will confuse the target. Need to resync the target by transmitting JTAG-to-SWD sequence.
    if (bit_cnt % 8 != 0) {
        transmit_seq_jtag_to_swd();
    }
    static int count = 0;  if (count++ == 1) { pabort("Exit for testing"); } ////
}

///  Transmit bit_cnt number of bits from buf starting at the bit offset.
static void spi_exchange_transmit(uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
    //  Consolidate the bits into a buffer before transmitting.
    unsigned int byte_cnt = (bit_cnt + 7) / 8;  //  Round up to next byte count.

    //  Transmit the consolidated bits to target.

	int tdi;

	for (unsigned int i = offset; i < bit_cnt + offset; i++) {
		int bytec = i/8;
		int bcval = 1 << (i % 8);
		tdi = !rnw && (buf[bytec] & bcval);

		bitbang_interface->write(0, 0, tdi);

		if (rnw && buf) {
			if (bitbang_interface->swdio_read())
				buf[bytec] |= bcval;
			else
				buf[bytec] &= ~bcval;
		}

		bitbang_interface->write(1, 0, tdi);
	}
}

///  Receive bit_cnt number of bits into buf starting at the bit offset.
static void spi_exchange_receive(uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
    //  Receive the bits from target.

    //  Populate buf from the received bits.


	int tdi;

	for (unsigned int i = offset; i < bit_cnt + offset; i++) {
		int bytec = i/8;
		int bcval = 1 << (i % 8);
		tdi = !rnw && (buf[bytec] & bcval);

		bitbang_interface->write(0, 0, tdi);

		if (rnw && buf) {
			if (bitbang_interface->swdio_read())
				buf[bytec] |= bcval;
			else
				buf[bytec] &= ~bcval;
		}

		bitbang_interface->write(1, 0, tdi);
	}
}

#endif  //  SWD_SPI

#ifdef NOTUSED
cd ~/openocd-spi
./bootstrap
./configure --enable-sysfsgpio --enable-bcm2835gpio --enable-cmsis-dap

clear ; cd ~/openocd-spi ; git pull ; make

clear ; cd ~/pinetime-rust-mynewt ; scripts/nrf52-pi/flash-unprotect.sh 

clear ; cd ~/pinetime-rust-mynewt ; scripts/nrf52-pi/flash-unprotect.sh >flash-unprotect.log 2>&1

clear ; cd ~/pi-swd-spi ; pi-swd-spi

sync ; sudo shutdown now

#endif  //  NOTUSED