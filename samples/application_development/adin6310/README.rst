.. _adin6310:

ADIN6310_T1L
#################

Overview
********

This project helps you to enable the ADI ADIN6310 6 port TSN switch (EVAL-ADIN6310T1LEBZ), which has the following features:

- 2x ADIN1300 10/100/1000 BASE-T Ethernet PHY
- 4x ADIN1100 10BASE-T1L Ethernet PHY
- LTC4296-1 PSE Controller supporting power class 12 by default.
- MAX32690 microcontroller with external Flash memory and RAM.

This project supports multiple examples based upon hardware configuration from the switch (S4)
as explained in the next section.

The default configuration for this hardware is for the MAX32690 processor to run firmware
to configure the ADIN6310 ethernet switch over SPI host interface into a basic switching mode
with VLAN ID 1-10 enabled for learning & forwarding on all ports. The MAX32690 also enables
the LTC4296-1 PSE controller to enable SpoE power to the 10BASE-T1L pairs for all four ports.

Example multiplexing
********************

Based on the position of the S4 switch during board power up, the following functionality will be enabled:

- Set S4.1 ON to disable PSE
- Set S4.2 ON to enable IEEE 802.1AS Time Synchronization on all ports (single instance)
- Set S4.3 ON to enable LLDP (Link layer discovery protocol)
- Set S4.4 ON to enable IGMP Snooping

In case of the REV C of the board, the switch selection value will default to 0000 (since the S4 switch
was only added on REV D of the board).

+---------------------------------------------+-------------------------------------------------------+
| Switch positions (S4) - REV D hardware only | Functionality                                         |
+=============================================+=======================================================+
| 0000                                        | Basic Switch + PSE (S4 default)                       |
+---------------------------------------------+-------------------------------------------------------+
| 0001                                        | Basic Switch                                          |
+---------------------------------------------+-------------------------------------------------------+
| 0010                                        | Basic Switch + PSE + Time Sync (802.1AS 2020)         |
+---------------------------------------------+-------------------------------------------------------+
| 0011                                        | Basic Switch + Time Sync                              |
+---------------------------------------------+-------------------------------------------------------+
| 0100                                        | Basic Switch + PSE + LLDP                             |
+---------------------------------------------+-------------------------------------------------------+
| 0101                                        | Basic Switch + LLDP                                   |
+---------------------------------------------+-------------------------------------------------------+
| 0110                                        | Basic Switch + PSE + Time Sync + LLDP                 |
+---------------------------------------------+-------------------------------------------------------+
| 0111                                        | Basic Switch + Time Sync + LLDP                       |
+---------------------------------------------+-------------------------------------------------------+
| 1000                                        | Basic Switch + PSE + IGMP Snooping                    |
+---------------------------------------------+-------------------------------------------------------+
| 1001                                        | Basic Switch + IGMP Snooping                          |
+---------------------------------------------+-------------------------------------------------------+
| 1010                                        | Basic Switch + PSE + Time Sync + IGMP Snooping        |
+---------------------------------------------+-------------------------------------------------------+
| 1011                                        | Basic Switch + Time Sync + IGMP Snooping              |
+---------------------------------------------+-------------------------------------------------------+
| 1100                                        | Basic Switch + PSE +  LLDP + IGMP Snooping            |
+---------------------------------------------+-------------------------------------------------------+
| 1101                                        | Basic Switch + LLDP + IGMP Snooping                   |
+---------------------------------------------+-------------------------------------------------------+
| 1110                                        | Basic Switch + PSE + Time Sync + LLDP + IGMP Snooping |
+---------------------------------------------+-------------------------------------------------------+
| 1111                                        | Basic Switch + Time Sync + LLDP + IGMP Snooping       |
+---------------------------------------------+-------------------------------------------------------+

Since the switch positions is only sampled at power up, any change to the switch position will require a power cycle
in order for the new configuration to apply.

Configuring the LTC4296-1 PSE controller
****************************************

By default, the LTC4296 is configured to support SPOE power class 12, and will provide power to devices connected
to the 10BASE-T1L ports if the SCCP protocol negotiation between the PSE and PD is successful. The PSE maximum
supported power class and the value of the high side sense resistors (connected to the HSNSPx and HSNSMx pins
of the LTC4296) can be changed by the user. Their value is configured the `app.overlay` file (in the project directory).

Search for the `ltc4296` node in the devicetree file and modify the `adi,power-class` and `adi,hs-resistor` properties.

.. code-block:: console

   ltc4296: ltc4296@1 {
       port0 {
			adi,power-class = <LTC4296_PSE_SCCP_CLASS_12>;
			adi,hs-resistor = <250>;
		};
   };

The `adi,power-class` property applies on a per port basis, and can be set to one of the following values:

- LTC4296_PSE_DISABLED # PSE disabled for this port
- LTC4296_PSE_APL # PSE APL mode
- LTC4296_PSE_SCCP_CLASS_10 # PSE SCCP class 10
- LTC4296_PSE_SCCP_CLASS_11 # PSE SCCP class 11
- LTC4296_PSE_SCCP_CLASS_12 # PSE SCCP class 12
- LTC4296_PSE_SCCP_CLASS_13 # PSE SCCP class 13
- LTC4296_PSE_SCCP_CLASS_14 # PSE SCCP class 14
- LTC4296_PSE_SCCP_CLASS_15 # PSE SCCP class 15

The `adi,hs-resistor` is expressed in milliohms and can be set to any integer value.

Each LTC4296 port has its own specific node in the devicetree (and the dt overlay) file (port0 to port4), and they can be configured independently, using
the mentioned properties.

Building
********

Build the sample application like this:

.. code-block:: console

   west build -b adin6310t1l/max32690/m4 samples/application_development/adin6310 -DLIB_ADIN6310_PATH=... -p auto

Programming
***********

The board can be programmed using 2 different probes:

1. SEGGER J-Link

Please note that you will need to have the J-Link software toolchain (may be downloaded from this page https://www.segger.com/downloads/jlink/)
installed and accessible from your `PATH` variable (both for Windows and Linux).

There are 2 methods of using the J-Link for programming
- Using west, by running the following in your terminal (has to be the same from which you previously compiled the project):

.. code-block:: console

   west flash --runner=jlink

- Using the JFlash (or JFlashLite) utility:
Open JFlashLite and select the MAX32690 MCU as the target. Then, you can program the .hex file found at the `build/zephyr/zephyr.hex` path
(in the `zephyr` directory).

2. MAX32625PICO

This method requires the user to use a custom version of OpenOCD. The easiest method of getting it is to install the MaximSDK using the automatic
installer available here https://analogdevicesinc.github.io/msdk//USERGUIDE/#installation . You need to make sure that "Open On-Chip Debugger"
is enabled in the "Select components" window during installation (it is by default).

After MaximSDK is installed, OpenOCD is available at the `MaximSDK/Tools/OpenOCD` path.
The MAX32690 can now be programmed by using west. Run the following in your terminal (has to be the same from which you previously compiled the project):

.. code-block:: console

   west flash --openocd-search ~/MaximSDK/Tools/OpenOCD/scripts/ --openocd ~/MaximSDK/Tools/OpenOCD/openocd

The path to the MaximSDK base directory has to be changed based on where you have previously installed it.

Running the firmware
********************

After the programming step, the recently loaded firmware image will run automatically. The microcontroller will log the configuration status
over UART (115200/8N1, no parity). For example, the 1111 switch configuration will lead to the following output:

.. code-block:: console

   Reader thread start
   *** Booting Zephyr OS build v1.12.0-77498-gcfddddfe8a98 ***
   Configured MAC address: 00:18:80:0e:47:78
   PSE disabled
   Time Synchonization example
   SES_PtpInitCmlds :: 0
   SES_PtpSetDefaultDs - 0
   Set MAC Address for port0 :: 0
   Set MAC Address for port1 :: 0
   Set MAC Address for port2 :: 0
   Set MAC Address for port3 :: 0
   Set MAC Address for port4 :: 0
   Set MAC Address for port5 :: 0
   LLDP Protcol
   IGMP Snooping
   Configuration done