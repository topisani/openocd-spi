#!/usr/bin/env bash
#  Build and test OpenOCD

set -e  #  Exit when any command fails.
set -x  #  Echo all commands.

clear ; cd ~/openocd-spi ; git pull ; make

clear ; cd ~/pinetime-rust-mynewt ; scripts/nrf52-pi/flash-unprotect.sh 
