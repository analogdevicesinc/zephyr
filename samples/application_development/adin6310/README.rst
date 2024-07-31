.. _adin6310_unmanaged_vlan:

ADIN6310 VLAN unmanaged configuration
#################

Overview
********

A simple example showcasing how to use the ADIN6310 SES driver in Zephyr's
userspace. The ADIN6310 kernel driver is not used in this case, however its
specific devicetree node is still required, since the interrupt configuration
depends on it.
The switch will forward frames with a VLAN ID between 1 - 10 received on any
port to all the other ports. The traffic which is not tagged or has an ID ouside
of the 1 - 10 interval will be dropped.

Building and Running
********************

Build the sample application like this:

.. code-block:: console

   west build -b adin6310switch/max32690/m4 samples/application_development/adin6310_unmanaged_vlan -DLIB_ADIN6310_PATH=... -p auto 

Flashing the application may be done by running:

.. code-block:: console

   west flash --runner=jlink