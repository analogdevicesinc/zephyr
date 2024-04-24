# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adin6310_unmanaged)

# FILE(GLOB adin6310_srcs ${CMAKE_CURRENT_SOURCE_DIR}/lib_adin6310/srcs/*.c)
target_sources(app PRIVATE src/main.c)
# target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib_adin6310/inc)
target_link_libraries(app PUBLIC libm.a)

zephyr_get_include_directories_for_lang_as_string(       C includes)
zephyr_get_system_include_directories_for_lang_as_string(C system_includes)
zephyr_get_compile_definitions_for_lang_as_string(       C definitions)
zephyr_get_compile_options_for_lang_as_string(           C options)

FILE(GLOB lib_adin6310_srcs ${LIB_ADIN6310_PATH}/src/ses-proxy-srv/src/*.c
          ${LIB_ADIN6310_PATH}/src/ses-route-srv/src/*.c
          ${LIB_ADIN6310_PATH}/src/ses-tsn-api-srv/src/*.c
          ${LIB_ADIN6310_PATH}/src/smp-stk/src/*.c
)
zephyr_library_sources(${lib_adin6310_srcs})
zephyr_include_directories(${LIB_ADIN6310_PATH}/src/ses-proxy-srv/inc
            ${LIB_ADIN6310_PATH}/src/ses-route-srv/inc
            ${LIB_ADIN6310_PATH}/src/ses-tsn-api-srv/inc
            ${LIB_ADIN6310_PATH}/src/smp-stk/inc
            ${LIB_ADIN6310_PATH}/src/tsn-model-srv/inc
)

set(external_project_cflags
  "${includes} ${definitions} ${options} ${system_includes} -DDISABLE_AUTOMATIC_FW_UPDATE"
  )