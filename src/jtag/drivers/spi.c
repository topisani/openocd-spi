void spi_exchange(bool rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
	{ ////
		printf("** %s offset %d bits %2d:", rnw ? "target -> host" : "host -> target", offset, bit_cnt);
		if (!rnw && buf) {
			for (unsigned int i = 0; i < (bit_cnt + 7) / 8; i++) {
				printf(" %02x", buf[i]);
			}
		}
		printf("\n");
	} ////
	////  LOG_DEBUG("bitbang_exchange");
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
