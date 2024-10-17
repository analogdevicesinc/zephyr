# Copyright (c) 2023-2024 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(openocd --cmd-pre-init "source [find interface/cmsis-dap.cfg]")
board_runner_args(openocd --cmd-pre-init "source [find target/max32655.cfg]")
board_runner_args(openocd --cmd-pre-init "allow_low_pwr_dbg")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
