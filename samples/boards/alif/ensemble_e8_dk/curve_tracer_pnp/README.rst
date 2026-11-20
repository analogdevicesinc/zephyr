.. zephyr:code-sample:: ensemble_e8_dk_curve_tracer_pnp
   :name: ADALM-LSMSPG PNP curve tracer on Ensemble E8 DK
   :relevant-api: dac_interface adc_interface

   Trace the Ic-Vc curves of the PNP curve tracer on an ADALM-LSMSPG board
   wired to the Alif Ensemble E8 DK mikroBUS socket, from a plain serial
   terminal.

Overview
********

This is the PNP half of the ``adi_lsmspg`` curve tracer sample, using only the
AD5593R (I2C). The shield cannot be used because the E8 DK has no Feather
connector, so the ADALM-LSMSPG is connected with jumper wires and the AD5593R
is declared in the board overlay on the ``mikrobus_i2c`` bus (I2C0, 100 kHz).

- AD5593R CH3 (DAC) drives the emitter through jumper P7.
- AD5593R CH0 (DAC) drives the base through a 49.9 kohm resistor.
- AD5593R CH1 (ADC) senses the collector voltage.
- The collector returns to AD5593R CH2 (DAC + ADC) through a 49.9 ohm sense
  resistor.

Each sweep prints the per-point values and an ASCII plot (60 columns wide,
plain ASCII only). Nothing starts automatically: type ``pnp_curvetrace``.

The plot is drawn like a PNP on a CRT curve tracer: Ic is zero on the top row
and grows downward, and Vc runs from 0 V on the left to the emitter voltage on
the right, where the curves start.

Wiring
******

Only I2C and power are connected. No SPI, AN, RST, CS, INT or +5V lines.
The ADALM-LSMSPG has its own 5 kohm pull-ups on SDA and SCL. Check that SDA
and SCL land on the matching Feather pins: with them swapped or misplaced the
AD5593R does not respond and the shell reports ``DAC channel 0 not ready``.

========================  ====================
E8 DK mikroBUS header     ADALM-LSMSPG Feather
========================  ====================
GND                       GND (two jumpers)
+3.3V                     3V3
SCL                       SCL
SDA                       SDA
========================  ====================

Fit jumper **P7** on the ADALM-LSMSPG so that AD5593R CH3 drives the PNP
emitter.

Building
********

.. code-block:: console

   west build -b ensemble_e8_dk/ae822fa0e5597ls0/rtss_hp \
       samples/boards/alif/ensemble_e8_dk/curve_tracer_pnp -p

The console and shell are on UART4 (RTSS-HP), 115200 8N1.

Flashing
********

#. Put switch **SW4** on the SE-UART position and flash with SETOOLS
   (``west flash``, ``alif_flash`` runner), or use ``west flash -r jlink``.
#. The PRG USB bridge is shared between SE-UART and the console UART. Move
   **SW4** back to the console position and reset the board, otherwise there
   is no output.

Running
*******

Linux (use whichever port the PRG USB bridge enumerates as):

.. code-block:: console

   picocom -b 115200 --imap lfcrlf /dev/ttyUSB0

On Windows, use the equivalent COM port, for example
``picocom -b 115200 --imap lfcrlf COM5`` (or PuTTY/Tera Term set to 115200 8N1).
Exit picocom with ``Ctrl-A Ctrl-X``.

The banner is printed once at boot. If you connect afterwards, press Enter,
or reset the board, to see the prompt; ``help`` lists the commands.

Sample session
**************

Illustrative output, not captured from this board (values come from the
upstream sample's run and will differ):

.. code-block:: console

   ADALM-LSMSPG PNP curve tracer (AD5593R on mikroBUS I2C)
   Commands:
     pnp_curvetrace  run one PNP sweep and print the ASCII plot
     help            list all shell commands
   uart:~$ pnp_curvetrace

   ========== AD5593R (I2C) PNP Curve Tracer ==========
   Vref: 2500 mV, Scale: 0.6104 mV/LSB, Rsense: 49.9 ohm, Rbase: 49900 ohm
   Emitter Drive: 2.4988 V

   Starting sweep...
   Base Drive: 1.9995 V, -4.023 uA
     coll voltage: 2.4994 V  coll current: 0.0000 mA
     coll voltage: 2.4524 V  coll current: 0.0122 mA
     ...

   === AD5593R (I2C) - PNP Curve Tracer (Ic vs Vc) ===
   Y-axis: Ic (0 to 5.87 mA, increasing downward)
   X-axis: Vc (0 to 2.50 V)

   +------------------------------------------------------------+
   |          *                                                 |
   |          ****                                              |
   ...
   +------------------------------------------------------------+
   0.0       0.50       1.00       1.50       2.00       2.50 V

   ===== AD5593R (I2C) PNP Curve Trace Complete =====

   Sweep took 3136 ms
   uart:~$
