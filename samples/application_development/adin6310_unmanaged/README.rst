.. _adin6310_unmanaged:

ADIN6310 unmanaged configuration
#################

Overview
********

A simple example showcasing how to use the ADIN6310 SES driver in Zephyr's
userspace. The ADIN6310 kernel driver is not used in this case, however its
specific devicetree node is still required, since the interrupt configuration
depends on it.
The switch is configured in an unmanaged mode, meaning that it will forward
frames received on each port to every other port.

Building and Running
********************

Build the sample application like this:

.. code-block:: console

   west build -b adin6310switch/max32690/m4 samples/application_development/adin6310_unmanaged -DLIB_ADIN6310_PATH=... -p auto 

Flashing the application may be done by running:

.. code-block:: console

   west flash --runner=jlink