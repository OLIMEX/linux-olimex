# A20-OLINUXINO-LIME2 Linux-Olimex 5.10.60
# Support for Realtek RTL8811CU/RTL8821CU USB wifi adapter driver version 5.4.1

## Linux kernel
There are several guides for kernel developers and users. These guides can
be rendered in a number of formats, like HTML and PDF. Please read
Documentation/admin-guide/README.rst first.

In order to build the documentation, use ``make htmldocs`` or
``make pdfdocs``.  The formatted documentation can also be read online at:

    https://www.kernel.org/doc/html/latest/

There are various text files in the Documentation/ subdirectory,
several of them using the Restructured Text markup notation.

Please read the Documentation/process/changes.rst file, as it contains the
requirements for building and running the kernel, and information about
the problems which may result by upgrading your kernel.

# How to build Linux Kernel for Olimage
## 1. Get Olimage sources
    # cd 
    # mkdir olimex-A20
    # cd olimex-A20
    # git clone https://github.com/ValentinoRicci/linux-olimex.git

## 2. Prepare for building
### 2.1 Install additional packages and tools required
    # sudo apt install build-essential bc kmod flex bison cpio libncurses5-dev fakeroot libelf-dev libssl-dev

### 2.2 Install arm-linux-gnueabihf- or aarch64 toolchain
#### 2.2.1 For A20-OLinuXino-LIME2 do:
    # sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

## 3 Building the kernel
### 3.1 Configuring kernel options 
In this github, the .config file is already updated for

Device drivers ---&gt; 

&nbsp; &nbsp; Network device support ---&gt;

&nbsp; &nbsp; &nbsp; Wireless LAN ---&gt;

&nbsp; &nbsp; &nbsp; &nbsp; Realtek devices

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &lt;M&gt; Realtek 8821C USB WiFi 
		
    # cd linux-olimex
    # sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- menuconfig
        < Exit > and save...
  
### 3.2 Enable or disable packages and modules (OPTION)
Manually edit .config or use:

    # sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- menuconfig

### 3.3 Build kernel package
    # EXTRA_VER=$(date +%Y%m%d-%H%M%S)
    # sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- LOCALVERSION=-olimex KDEB_PKGVERSION=$(make kernelversion)-$EXTRA_VER DTC_FLAGS=-@
    # cd drivers/net/wireless/realtek/rtl8821CU
    # scp 8821cu.ko olimex@<ip address>:/tmp

### 3.4 Install module on board
Open the OLIMEX A20-OLINUXINO-LIME2 terminal:

    # cd /tmp
    # insmod 8821cu.ko
    # ifconfig

