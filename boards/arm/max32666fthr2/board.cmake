# Copyright (c) 2023 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_MAX32666FTHR2)
board_runner_args(openocd --cmd-pre-init "source [find interface/cmsis-dap.cfg]")
board_runner_args(openocd --cmd-pre-init "source [find target/max32665.cfg]")
endif()

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
