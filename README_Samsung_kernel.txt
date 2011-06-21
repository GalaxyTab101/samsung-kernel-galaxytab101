HOW TO BUILD KERNEL FOR GT-P7510

1. How to Build

	- get Toolchain
			From Codesourcery site( http://www.codesourcery.com )
			Ex) Download : http://www.codesourcery.com/sgpp/lite/arm/portal/package7813/public/arm-none-eabi/arm-2010.09-51-arm-none-eabi-i686-pc-linux-gnu.tar.bz2

			recommand :
							Feature : ARM
							Target OS : "EABI"
							package : "IA32 GNU/Linux TAR"

	- edit Makefile
			edit "CROSS_COMPILE" to right toolchain path(You downloaded).
			Ex)  CROSS_COMPILE=/opt/toolchains/arm-eabi-4.4.3/bin/arm-eabi-                 // You have to check.

	- make
			$ cd kernel
			$ make samsung_p4wifi_defconfig
			$ make
	
2. Output files
	- Kernel : kernel/arch/arm/boot/zImage