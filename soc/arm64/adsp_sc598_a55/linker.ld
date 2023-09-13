/*
 * Copyright (c) 2023 Analog Devices
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/* #include <zephyr/arch/arm64/scripts/linker.ld> */


/*
 * Copyright 2023 NXP
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/* This is mostly a copy of arch/arm64/scripts/linker.ld with
 * a few SOF-related changes. This is needed in order to avoid
 * introducing sections based on CONFIG_SOF in the main ARM64
 * linker script.
 */
#include <zephyr/linker/sections.h>
#include <zephyr/devicetree.h>

#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>

/* physical address of RAM */
#ifdef CONFIG_XIP
  #define ROMABLE_REGION FLASH
#else
  #define ROMABLE_REGION RAM
#endif
#define RAMABLE_REGION RAM

#if !defined(CONFIG_XIP) && (CONFIG_FLASH_SIZE == 0)
  #define ROM_ADDR RAM_ADDR
#else
  #define ROM_ADDR (CONFIG_FLASH_BASE_ADDRESS + CONFIG_FLASH_LOAD_OFFSET)
#endif

#if CONFIG_FLASH_LOAD_SIZE > 0
  #define ROM_SIZE CONFIG_FLASH_LOAD_SIZE
#else
  #define ROM_SIZE (CONFIG_FLASH_SIZE * 1K - CONFIG_FLASH_LOAD_OFFSET)
#endif

#define RAM_SIZE (CONFIG_SRAM_SIZE * 1K)
#define RAM_ADDR CONFIG_SRAM_BASE_ADDRESS

#if defined(CONFIG_ARM_MMU)
  _region_min_align = CONFIG_MMU_PAGE_SIZE;
#elif defined(CONFIG_ARM_MPU)
  _region_min_align = CONFIG_ARM_MPU_REGION_MIN_ALIGN_AND_SIZE;
  #define BSS_ALIGN ALIGN(_region_min_align)
#else
  /* If building without MMU support, use default 4-byte alignment. */
  _region_min_align = 4;
#endif

#ifndef BSS_ALIGN
#define BSS_ALIGN
#endif

#define MMU_ALIGN    . = ALIGN(_region_min_align)

MEMORY
{
  /* #### SHARED L2 Memory ############################################### */
  /* ICC Data, uncached */
  MEM_L2_ICC_DATA : ORIGIN = 0x20000000, LENGTH = 4K

  /* ICC Buffer ARM Core 0, uncached */
  MEM_L2_ICC_DATA_CORE0 : ORIGIN = 0x20001000, LENGTH = 16K

  /* ICC Buffer SHARC Core 2, uncached */
  MEM_L2_ICC_DATA_CORE2 (r) : ORIGIN = 0x20005000, LENGTH = 64K

  /* ICC Buffer SHARC Core 1, uncached */
  MEM_L2_ICC_DATA_CORE1 (r) : ORIGIN = 0x20015000, LENGTH = 64K

  /* #### ARM L2 Memory ################################################## */

  /* ARM Core 0 L2 RAM, uncached */
  MEM_L2_UNCACHED : ORIGIN = 0x20025000, LENGTH = 108K

  /* ARM Core 0 L2 RAM, cached */
  MEM_L2_CACHED : ORIGIN = 0x20040000, LENGTH = 768K

  /* #### SHARC L2 Memory ################################################ */
  /* SHARC Core 2 L2 RAM, cached */
  MEM_L2_CORE2 (r) : ORIGIN = 0x20100000, LENGTH = 512K

  /* SHARC Core 1 L2 RAM, cached */
  MEM_L2_CORE1 (r) : ORIGIN = 0x20180000, LENGTH = 512K

  /* #### SHARC L3 Memory ################################################ */

  /* SHARC Core 1 NW Code, DMC0 */
  MEM_L3_CORE1_NW_CODE (r) : ORIGIN = 0x80000000, LENGTH = 3M

  /* SHARC Core 2 NW Code, DMC0 */
  MEM_L3_CORE2_NW_CODE (r) : ORIGIN = 0x80300000, LENGTH = 3M

  /* SHARC Core 1 Data, DMC0 */
  MEM_L3_CORE1_DATA (r) : ORIGIN = 0x80600000, LENGTH = 4M

  /* SHARC Core 1 VISA Code, DMC0 */
  MEM_L3_CORE1_VISA_CODE (r) : ORIGIN = 0x80A00000, LENGTH = 3M

  /* SHARC Core 2 VISA Code, DMC0 */
  MEM_L3_CORE2_VISA_CODE (r) : ORIGIN = 0x80D00000, LENGTH = 3M

  /* SHARC Core 1 Data, DMC0 */
  MEM_L3_CORE1_DATA2 (r) : ORIGIN = 0x81000000, LENGTH = 118M

  /* SHARC Core 2 Data, DMC0 */
  MEM_L3_CORE2_DATA2 (r) : ORIGIN = 0x88600000, LENGTH = 122M

  /* #### ARM L3 Memory ################################################## */

  /* ARM Core 0 L3, DMC0 */
  RAM : ORIGIN = 0x90000000, LENGTH = 256M

   /* Used by and documented in include/linker/intlist.ld */
    IDT_LIST  (wx) : ORIGIN = 0xFFFFF7FF, LENGTH = 2K

}

SECTIONS {
  __ICC_common_start = ORIGIN(MEM_L2_ICC_DATA) ;
  __ICC_common_end   = ORIGIN(MEM_L2_ICC_DATA) + LENGTH(MEM_L2_ICC_DATA) - 1 ;
  __ICC_arm_start = ORIGIN(MEM_L2_ICC_DATA_CORE0) ;
  __ICC_arm_end   = ORIGIN(MEM_L2_ICC_DATA_CORE0) + LENGTH(MEM_L2_ICC_DATA_CORE0) - 1 ;
  __ICC_sharc0_start = ORIGIN(MEM_L2_ICC_DATA_CORE1) ;
  __ICC_sharc0_end   = ORIGIN(MEM_L2_ICC_DATA_CORE1) + LENGTH(MEM_L2_ICC_DATA_CORE1) - 1 ;
  __ICC_sharc1_start = ORIGIN(MEM_L2_ICC_DATA_CORE2) ;
  __ICC_sharc1_end   = ORIGIN(MEM_L2_ICC_DATA_CORE2) + LENGTH(MEM_L2_ICC_DATA_CORE2) - 1 ;
  __l2_start = 0x20000000 ;
  __l2_end = 0x201FFFFF ;
  __l2_uncached_start = ORIGIN(MEM_L2_ICC_DATA_CORE0) ;
  __l2_uncached_end = ORIGIN(MEM_L2_UNCACHED) + LENGTH(MEM_L2_UNCACHED) - 1 ;
  __l2_cached_start = ORIGIN(MEM_L2_CACHED) ;
  __l2_cached_end = ORIGIN(MEM_L2_CORE1) + LENGTH(MEM_L2_CORE1) - 1 ;
}

OUTPUT_ARCH(aarch64)
OUTPUT_FORMAT(elf64-littleaarch64)
ENTRY(_start)

ENTRY(CONFIG_KERNEL_ENTRY)

SECTIONS
{

/* Read-only sections, merged into text segment: */
  .interp     : { *(.interp) 	}
  .hash          : { *(.hash)		}
  .dynsym        : { *(.dynsym)		}
  .dynstr        : { *(.dynstr)		}
  .gnu.version   : { *(.gnu.version)	}
  .gnu.version_d   : { *(.gnu.version_d)	}
  .gnu.version_r   : { *(.gnu.version_r)	}
  .rel.init      : { *(.rel.init)	}
  .rela.init     : { *(.rela.init)	}
  .rel.text      :
    {
      *(.rel.text)
      *(.rel.text.*)
      *(.rel.gnu.linkonce.t*)
    }
  .rela.text     :
    {
      *(.rela.text)
      *(.rela.text.*)
      *(.rela.gnu.linkonce.t*)
    }
  .rel.fini      : { *(.rel.fini)	}
  .rela.fini     : { *(.rela.fini)	}
  .rel.rodata    :
    {
      *(.rel.rodata)
      *(.rel.rodata.*)
      *(.rel.gnu.linkonce.r*)
    }
  .rela.rodata   :
    {
      *(.rela.rodata)
      *(.rela.rodata.*)
      *(.rela.gnu.linkonce.r*)
    }
  .rel.data      :
    {
      *(.rel.data)
      *(.rel.data.*)
      *(.rel.gnu.linkonce.d*)
    }
  .rela.data     :
    {
      *(.rela.data)
      *(.rela.data.*)
      *(.rela.gnu.linkonce.d*)
    }
  .rel.ctors     : { *(.rel.ctors)	}
  .rela.ctors    : { *(.rela.ctors)	}
  .rel.dtors     : { *(.rel.dtors)	}
  .rela.dtors    : { *(.rela.dtors)	}
  .rel.got       : { *(.rel.got)		}
  .rela.got      : { *(.rela.got)		}
  .rel.sdata     :
    {
      *(.rel.sdata)
      *(.rel.sdata.*)
      *(.rel.gnu.linkonce.s*)
    }
  .rela.sdata     :
    {
      *(.rela.sdata)
      *(.rela.sdata.*)
      *(.rela.gnu.linkonce.s*)
    }
  .rel.sbss      : { *(.rel.sbss)		}
  .rela.sbss     : { *(.rela.sbss)	}
  .rel.bss       : { *(.rel.bss)		}
  .rela.bss      : { *(.rela.bss)		}
  .rel.plt       : { *(.rel.plt)		}
  .rela.plt      : { *(.rela.plt)		}
  .plt      : { *(.plt)	}

#include <zephyr/linker/rel-sections.ld>

    /*
     * .plt and .iplt are here according to 'arm-zephyr-elf-ld --verbose',
     * before text section.
     */
    /DISCARD/ :
    {
        *(.plt)
    }

    /DISCARD/ :
    {
        *(.iplt)
    }

    GROUP_START(ROMABLE_REGION)

    __rom_region_start = ROM_ADDR;

    SECTION_PROLOGUE(_TEXT_SECTION_NAME,,)
    {
        __text_region_start = .;
#ifndef CONFIG_XIP
        z_mapped_start = .;
#endif

#ifdef CONFIG_AARCH64_IMAGE_HEADER
        KEEP(*(.image_header))
        KEEP(*(".image_header.*"))
#endif

        _vector_start = .;
        KEEP(*(.exc_vector_table))
        KEEP(*(".exc_vector_table.*"))

        KEEP(*(.vectors))

        _vector_end = .;

        *(.text)
        *(".text.*")
        *(.gnu.linkonce.t.*)

        /*
         * These are here according to 'arm-zephyr-elf-ld --verbose',
         * after .gnu.linkonce.t.*
         */
        *(.glue_7t) *(.glue_7) *(.vfp11_veneer) *(.v4_bx)

#include <zephyr/linker/kobject-text.ld>

        MMU_ALIGN;
    } GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

    __text_region_end = .;
    __text_region_size = __text_region_end - __text_region_start;

#if defined (CONFIG_CPP)
    SECTION_PROLOGUE(.ARM.extab,,)
    {
        /*
         * .ARM.extab section containing exception unwinding information.
         */
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
#endif

    SECTION_PROLOGUE(.ARM.exidx,,)
    {
        /*
         * This section, related to stack and exception unwinding, is placed
         * explicitly to prevent it from being shared between multiple regions.
         * It  must be defined for gcc to support 64-bit math and avoid
         * section overlap.
         */
        __exidx_start = .;
#if defined (__GCC_LINKER_CMD__)
        *(.ARM.exidx* gnu.linkonce.armexidx.*)
#endif
        __exidx_end = .;
    } GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

        __rodata_region_start = .;

#include <zephyr/linker/common-rom.ld>
#include <zephyr/linker/thread-local-storage.ld>

    SECTION_PROLOGUE(_RODATA_SECTION_NAME,,)
    {
        *(.rodata)
        *(".rodata.*")
        *(.gnu.linkonce.r.*)

        /*
         * The following is a workaround to allow compiling with GCC 12 and
         * above, which may emit "GOT indirections" for the weak symbol
         * references (see the GitHub issue zephyrproject-rtos/sdk-ng#547).
         */
        *(.got)
        *(.got.plt)

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-rodata.ld>

#include <zephyr/linker/kobject-rom.ld>

    } GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

#include <zephyr/linker/cplusplus-rom.ld>
    MMU_ALIGN;

    __rodata_region_end = .;
    __rodata_region_size = __rodata_region_end - __rodata_region_start;
    __rom_region_end = .;

    /*
     * These are here according to 'arm-zephyr-elf-ld --verbose',
     * before data section.
     */
    /DISCARD/ :
    {
        *(.igot.plt)
        *(.igot)
    }

    GROUP_END(ROMABLE_REGION)

    GROUP_START(RAMABLE_REGION)

    . = RAM_ADDR;
    /* Align the start of image RAM with the
     * minimum granularity required by MMU.
     */
    . = ALIGN(_region_min_align);
    _image_ram_start = .;
#ifdef CONFIG_XIP
    z_mapped_start = .;
#endif

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-ram-sections.ld>

#if defined(CONFIG_USERSPACE)
#define APP_SHARED_ALIGN . = ALIGN(_region_min_align);
#define SMEM_PARTITION_ALIGN(size) MMU_ALIGN

#if defined(CONFIG_ARM_MPU)
/*
 * When _app_smem region is empty, alignment is also needed. If there
 * is no alignment, the _app_smem_start used by arm mpu can be lower
 * than __rodata_region_end, and this two regions can overlap.
 * The Armv8-R aarch64 MPU does not allow overlapped regions.
 */
#define EMPTY_APP_SHARED_ALIGN APP_SHARED_ALIGN
#endif

#include <app_smem.ld>

    _app_smem_size = _app_smem_end - _app_smem_start;
    _app_smem_rom_start = LOADADDR(_APP_SMEM_SECTION_NAME);
#endif  /* CONFIG_USERSPACE */

    SECTION_DATA_PROLOGUE(_BSS_SECTION_NAME,(NOLOAD), BSS_ALIGN)
    {
        /*
         * For performance, BSS section is assumed to be 4 byte aligned and
         * a multiple of 4 bytes
         */
        . = ALIGN(4);
        __bss_start = .;
        __kernel_ram_start = .;

        *(.bss)
        *(".bss.*")
        *(COMMON)
        *(".kernel_bss.*")

        /*
         * As memory is cleared in words only, it is simpler to ensure the BSS
         * section ends on a 4 byte boundary. This wastes a maximum of 3 bytes.
                 */
        __bss_end = ALIGN(4);
    } GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

#if CONFIG_SOF
    SECTION_PROLOGUE(.static_uuid_entries,,)
    {
        *(*.static_uuids)
    } GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

    SECTION_PROLOGUE(.static_log_entries,,)
    {
        *(*.static_log*)
    } GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
#endif /* CONFIG_SOF */

#include <zephyr/linker/common-noinit.ld>

    SECTION_DATA_PROLOGUE(_DATA_SECTION_NAME,,)
    {
	__data_region_start = .;
	__data_start = .;
        *(.data)
        *(".data.*")
        *(".kernel.*")
#if CONFIG_SOF
	_trace_ctx_start = ABSOLUTE(.);
	*(.trace_ctx)
	_trace_ctx_end = ABSOLUTE(.);
#endif /* CONFIG_SOF */

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-rwdata.ld>

        __data_end = .;

    } GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
    __data_size = __data_end - __data_start;
    __data_load_start = LOADADDR(_DATA_SECTION_NAME);

    __data_region_load_start = LOADADDR(_DATA_SECTION_NAME);

#include <zephyr/linker/common-ram.ld>
#include <zephyr/linker/kobject-data.ld>
#include <zephyr/linker/cplusplus-ram.ld>

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-data-sections.ld>

    __data_region_end = .;


    /* Define linker symbols */

    MMU_ALIGN;
    _image_ram_end = .;
    _end = .; /* end of image */
    z_mapped_end = .;

    __kernel_ram_end = RAM_ADDR + RAM_SIZE;
    __kernel_ram_size = __kernel_ram_end - __kernel_ram_start;

    GROUP_END(RAMABLE_REGION)

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-sections.ld>

#include <zephyr/linker/debug-sections.ld>

    SECTION_PROLOGUE(.ARM.attributes, 0,)
    {
        KEEP(*(.ARM.attributes))
        KEEP(*(.gnu.attributes))
    }

    /DISCARD/ : { *(.note.GNU-stack) }


    /* Must be last in romable region */
    SECTION_PROLOGUE(.last_section,,)
    {
#ifdef CONFIG_LINKER_LAST_SECTION_ID
      /* Fill last section with a word to ensure location counter and actual rom
       * region data usage match. */
      LONG(CONFIG_LINKER_LAST_SECTION_ID_PATTERN)
#endif
    } GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

    /* To provide the image size as a const expression,
     * calculate this value here. */
    _flash_used = LOADADDR(.last_section) + SIZEOF(.last_section) - __rom_region_start;

}

