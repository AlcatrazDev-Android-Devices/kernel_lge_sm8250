#!/bin/bash
args="-j$(nproc --all) \
	O=../out \
	ARCH=arm64 \
	CROSS_COMPILE=../aarch64-linux-android-4.9/bin/aarch64-linux-android- \
	CROSS_COMPILE_ARM32=../arm-linux-androideabi-4.9/bin/arm-linux-androideabi- \
	CC=../clang-r498229b/bin/clang \
	CLANG_TRIPLE=aarch64-linux-gnu- \
	LLVM_IAS=1"
	make ${args} vendor/timelm-perf_defconfig
	make ${args}
