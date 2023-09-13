/*
 * Copyright (c) 2023 Analog Devices
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>

void main(void)
{
	int ch;
	
	printk("Hello World! %s\n", CONFIG_BOARD);

	for( ch = 75 ; ch <= 100; ch++ ) {
      printk("ASCII value = %d, Character = %c\n", ch , ch );
   }

	printk("Bye %s\n", CONFIG_BOARD);

    exit(0);
}
