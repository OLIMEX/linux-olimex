#!/bin/sh

# EXTRA_VER=$(date +%Y%m%d-%H%M%S)  ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- DEFCONFIG="linux-sunxi-olinuxino_defconfig" ./build-package.sh

make distclean
make $DEFCONFIG
make -j$(nproc) bindeb-pkg LOCALVERSION=-olimex KDEB_PKGVERSION=$(make kernelversion)-$EXTRA_VER DTC_FLAGS=-@
