# Copyright (c) 2026 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

set(output ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME})

function(zephyr_runner_file type path)
  # Property magic which makes west flash choose the signed build
  # output of a given type.
  set_target_properties(runners_yaml_props_target PROPERTIES "${type}_file" "${path}")
endfunction()

zephyr_linker_sources(ROM_START SORT_KEY 0x0header sbt/sbt_sla_header.ld)
zephyr_linker_sources(SECTIONS sbt/sbt_sla_symbols.ld)
if(CONFIG_SOC_MAX32651)
  set(SECURE_SOC "MAX32651")
endif()

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL Windows)
  set(SIGNING_TOOL sign_app.exe)
else()
  set(SIGNING_TOOL sign_app)
endif()

find_path(SIGNING_TOOL_PATH ${SIGNING_TOOL})

if(SIGNING_TOOL_PATH STREQUAL SIGNING_TOOL_PATH-NOTFOUND)
  message(WARNING "
  Signing Image tool (${SIGNING_TOOL}) is not available.
  Signed image will not be generated.
  You won't be able to run application on the board.
  Refer to board documentation for more information")
else()
  message(STATUS "Signing Image tool (${SIGNING_TOOL}) is available at ${SIGNING_TOOL_PATH}.")

  if(CONFIG_MAX32_SECURE_SOC_KEY_PATH STREQUAL "")
    message(WARNING "
    CONFIG_MAX32_SECURE_SOC_KEY_PATH is not set.
    Signed image could not be generated.")
  else()
    message(STATUS "${CONFIG_MAX32_SECURE_SOC_KEY_PATH} will be used to sign the image.")
  endif()

  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    # Remove the .sig section from the ELF file before signing.
    COMMAND ${CMAKE_OBJCOPY} -O binary --remove-section=.sig
    ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} ${output}_removed_sig.bin

    # Sign the binary using the signing tool. The tool takes the binary without the .sig section, generates a signature.
    COMMAND ${SIGNING_TOOL} -c ${SECURE_SOC}
    ca=${output}_removed_sig.bin
    sca=${output}.sbin
    key_file=${CONFIG_MAX32_SECURE_SOC_KEY_PATH}

    # Add the .sig section back to the ELF file after signing.
    COMMAND ${CMAKE_OBJCOPY} -O elf32-littlearm ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME}
    --update-section .sig=${ZEPHYR_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.sig ${output}.signed.elf
  )

  if(CONFIG_BUILD_OUTPUT_HEX)
    zephyr_runner_file(hex ${output}.signed.hex)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      # --gap-fill=0x00 fills NOBITS sections (eg. tbss) to their full Memory Size.
      # This prevents signature mismatch between signed ELF and flashed HEX during secure boot.
      COMMAND ${CMAKE_OBJCOPY} -O ihex --gap-fill=0x00 ${output}.signed.elf ${output}.signed.hex
    )
  endif()

  if(CONFIG_BUILD_OUTPUT_BIN)
    zephyr_runner_file(bin ${output}.signed.bin)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      # --gap-fill=0x00 fills NOBITS sections (eg. tbss) to their full Memory Size.
      # This prevents signature mismatch between signed ELF and flashed BIN during secure boot.
      COMMAND ${CMAKE_OBJCOPY} -O binary --gap-fill=0x00 ${output}.signed.elf ${output}.signed.bin
    )
  endif()
endif()
