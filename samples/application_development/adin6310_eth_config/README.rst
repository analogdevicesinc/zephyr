.. _adin6310_eth_config:

ADIN6310 ethernet configuration
#################

Overview
********

This is an example of using the ADIN6310 kernel driver. The forwarding rules
are the same as in the unmanaged example, however the Zephyr's networking
stack may used in order to process frames received on a "configuration port"
(defined in the device tree by the "cpu-port" property).

Building and Running
********************

Build the sample application like this:

.. code-block:: console

   west build -b adin6310switch/max32690/m4 samples/application_development/adin6310_unmanaged -DLIB_ADIN6310_PATH=... -p auto

Flashing the application may be done by running:

.. code-block:: console

   west flash --runner=jlink

Testing
*******

The sample may be tested by connecting over an Ethernet cable to the "cpu-port"
of the switch and then running the ping command.

.. code-block:: console

   ping 192.168.1.5
   PING 192.168.1.5 (192.168.1.5) 56(84) bytes of data.
   64 bytes from 192.168.1.5: icmp_seq=1 ttl=64 time=1008 ms
   64 bytes from 192.168.1.5: icmp_seq=2 ttl=64 time=3.93 ms
   64 bytes from 192.168.1.5: icmp_seq=3 ttl=64 time=1.81 ms
