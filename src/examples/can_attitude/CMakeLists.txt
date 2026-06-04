############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Detect UAVCAN driver dynamically to match board target
set(UAVCAN_DRIVER "generic")
if(CONFIG_ARCH_CHIP)
	if(${CONFIG_NET_CAN} MATCHES "y")
		set(UAVCAN_DRIVER "socketcan")
	elseif(${CONFIG_ARCH_CHIP} MATCHES "kinetis")
		set(UAVCAN_DRIVER "kinetis")
	elseif(${CONFIG_ARCH_CHIP} MATCHES "stm32h7")
		set(UAVCAN_DRIVER "stm32h7")
	elseif(${CONFIG_ARCH_CHIP} MATCHES "stm32")
		set(UAVCAN_DRIVER "stm32")
	endif()
endif()

px4_add_module(
	MODULE examples__can_attitude
	MAIN can_attitude
	SRCS
		can_attitude_main.cpp
		can_attitude_start.cpp
		can_attitude_example.cpp
	COMPILE_FLAGS
		-Wno-format-security
		-Wno-deprecated-copy
		-Wno-cast-align
		-Wno-address-of-packed-member
	DEPENDS
		drivers__uavcan
		uavcan
		px4_uavcan_dsdlc
	)

target_include_directories(examples__can_attitude PRIVATE
	${PX4_SOURCE_DIR}/src/drivers
	${PX4_BINARY_DIR}/src/drivers/uavcan/include
	${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan/libuavcan/include
	${PX4_BINARY_DIR}/src/drivers/uavcan/include/dsdlc_generated
	${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers/posix/include
	${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers/${UAVCAN_DRIVER}/driver/include
)

if(NOT config_uavcan_num_ifaces)
	set(config_uavcan_num_ifaces 1)
endif()

string(TOUPPER "${PX4_PLATFORM}" OS_UPPER)
string(TOUPPER "${UAVCAN_DRIVER}" UAVCAN_DRIVER_UPPER)

target_compile_definitions(examples__can_attitude PRIVATE
	-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
	-DUAVCAN_NO_ASSERTIONS
	-DUAVCAN_IMPLEMENT_PLACEMENT_NEW=1
	-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
	-DUAVCAN_${UAVCAN_DRIVER_UPPER}_${OS_UPPER}=1
	-DUAVCAN_${UAVCAN_DRIVER_UPPER}_NUM_IFACES=${config_uavcan_num_ifaces}
	-DUAVCAN_NUM_IFACES=${config_uavcan_num_ifaces}
	-DUAVCAN_DRIVER=uavcan_${UAVCAN_DRIVER}
)
