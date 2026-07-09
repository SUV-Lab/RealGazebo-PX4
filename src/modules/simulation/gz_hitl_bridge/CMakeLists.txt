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

# HilCodec: pure conversion library (no gz dependency -- unit-tested)
add_library(gz_hitl_codec STATIC HilCodec.cpp)
add_dependencies(gz_hitl_codec mavlink_c_generate)
target_include_directories(gz_hitl_codec PUBLIC
	${CMAKE_BINARY_DIR}/mavlink
	${CMAKE_BINARY_DIR}/mavlink/${CONFIG_MAVLINK_DIALECT}
)
target_compile_options(gz_hitl_codec PUBLIC -Wno-address-of-packed-member -Wno-cast-align)

px4_add_unit_gtest(SRC HilCodecTest.cpp LINKLIBS gz_hitl_codec)

# FcLink: FC UDP/serial transport + framing + heartbeat (no gz dependency -- unit-tested)
add_library(gz_hitl_fclink STATIC FcLink.cpp)
add_dependencies(gz_hitl_fclink mavlink_c_generate)
target_include_directories(gz_hitl_fclink PUBLIC
	${CMAKE_BINARY_DIR}/mavlink
	${CMAKE_BINARY_DIR}/mavlink/${CONFIG_MAVLINK_DIALECT}
)
# Link codec as PUBLIC so its include paths/compile options are inherited and propagate to
# anything that links gz_hitl_fclink (e.g. unit tests, the bridge executable).
target_link_libraries(gz_hitl_fclink PUBLIC gz_hitl_codec pthread)

px4_add_unit_gtest(SRC FcLinkTest.cpp LINKLIBS gz_hitl_fclink)

# Bridge executable (requires gz-transport)
if(NOT DEFINED ENV{GZ_DISTRO} OR NOT "$ENV{GZ_DISTRO}" STREQUAL "harmonic")
	find_package(gz-transport NAMES gz-transport gz-transport15 gz-transport14 gz-transport13)
else()
	find_package(gz-transport NAMES gz-transport13)
endif()

if(gz-transport_FOUND)
	if(gz-transport_VERSION VERSION_LESS "15")
		set(GZ_TRANSPORT_TARGET "gz-transport${gz-transport_VERSION_MAJOR}::core")
	else()
		set(GZ_TRANSPORT_TARGET "gz-transport::core")
	endif()

	add_executable(gz-hitl-bridge
		main.cpp
		GzSource.cpp
		GcsRelay.cpp
	)
	target_link_libraries(gz-hitl-bridge PRIVATE gz_hitl_codec gz_hitl_fclink ${GZ_TRANSPORT_TARGET} pthread)
	set_target_properties(gz-hitl-bridge PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY ${PX4_BINARY_DIR}/bin)
endif()
