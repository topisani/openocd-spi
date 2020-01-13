//#define SWD_SPI
#ifdef SWD_SPI	

///  Transmit or receive bit_cnt number of bits from buf starting at the bit offset.
///  If rnw is false: Transmit from host to target
///  If rnw is true:  Receive from target to host
void spi_exchange(bool rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
    if (!buf) { pabort("spi_exchange: null buffer"); }
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

sync ; sudo shutdown now

#endif  //  NOTUSED