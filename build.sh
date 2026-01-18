#!/bin/bash

## voxl-cross >=3.3 contains the following cross-compile toolchains
## GCC 4.9 for APQ8096 32 and 64-bit respectively
## GCC 7 for QRB5165 1.X system images based on Ubuntu 18.04
## GCC 9 for QRB5165 2.x system images based on Ubuntu 20.04
## APQ8096 is now EOL so this example only builds for qrb5165
TOOLCHAIN_APQ8096_32="/opt/cross_toolchain/arm-gnueabi-4.9.toolchain.cmake"
TOOLCHAIN_APQ8096_64="/opt/cross_toolchain/aarch64-gnu-4.9.toolchain.cmake"
TOOLCHAIN_QRB5165="/opt/cross_toolchain/aarch64-gnu-7.toolchain.cmake"
TOOLCHAIN_QRB5165_2="/opt/cross_toolchain/aarch64-gnu-9.toolchain.cmake"

# placeholder in case more cmake opts need to be added later
EXTRA_OPTS=""

## this list is just for tab-completion
AVAILABLE_PLATFORMS="qrb5165 qrb5165-2 native"


print_usage(){
	echo ""
	echo " Build the current project based on platform target."
	echo ""
	echo " Usage:"
	echo ""
	echo "  ./build.sh qrb5165"
	echo "        Build 64-bit binaries for qrb5165 1.X system images"
	echo ""
	echo "  ./build.sh qrb5165-2"
	echo "        Build 64-bit binaries for qrb5165 2.X system images"
	echo ""
	echo "  ./build.sh native"
	echo "        Build with the native gcc/g++ compilers for testing code"
	echo "        locally on a desktop computer."
	echo ""
	echo ""
}


check_docker() {
	local MIN_VERSION="$1"
	local FILE="/etc/modalai/image.name"

	if [[ ! -f "$FILE" ]]; then
		echo "$FILE does not exist, are you running in the voxl-cross docker?"
		exit 1
	fi

	local IMAGE_STRING
	IMAGE_STRING=$(<"$FILE")

	if [[ "$IMAGE_STRING" =~ ^voxl-cross\(([0-9]+\.[0-9]+)\)$ ]]; then
		local VERSION="${BASH_REMATCH[1]}"
		echo "Found voxl-cross version: $VERSION"

		if [[ "$(printf '%s\n' "$VERSION" "$MIN_VERSION" | sort -V | head -n1)" == "$MIN_VERSION" ]]; then
			return 0
		else
			echo "voxl-cross $VERSION does not meet minimum required $MIN_VERSION."
			exit 1
		fi
	else
		echo "voxl-cross not found in $FILE"
		echo "are you running in the voxl-cross docker?"
		exit 1
	fi
}


case "$1" in
	qrb5165)
		mkdir -p build
		cd build
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165} -DPLATFORM=QRB5165 ${EXTRA_OPTS} ../
		make -j$(nproc)
		cd ../
		;;
	qrb5165-2)
		mkdir -p build
		cd build
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165_2} -DPLATFORM=QRB5165_2 ${EXTRA_OPTS} ../
		make -j$(nproc)
		cd ../
		;;

	native)
		mkdir -p build
		cd build
		cmake ${EXTRA_OPTS} ../
		make -j$(nproc)
		cd ../
		;;

	*)
		print_usage
		exit 1
		;;
esac



