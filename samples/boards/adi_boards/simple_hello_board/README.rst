.. _simple_hello_board:

Simple Hello Board
##################

Overview
********

A simple sample that can be used with any ADI part and
prints "Hello World <name board>" to the console.

Building and Running
********************

This application can be built and executed on OPENOCD as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/simple_hello_board
   :host-os: unix
   :board: ADI Part
   :goals: run
   :compact:


Sample Output
=============

.. code-block:: console

Hello World! <name board>
ASCII value = 75, Character = K
ASCII value = 76, Character = L
ASCII value = 77, Character = M
ASCII value = 78, Character = N
ASCII value = 79, Character = O
ASCII value = 80, Character = P
ASCII value = 81, Character = Q
ASCII value = 82, Character = R
ASCII value = 83, Character = S
ASCII value = 84, Character = T
ASCII value = 85, Character = U
ASCII value = 86, Character = V
ASCII value = 87, Character = W
ASCII value = 88, Character = X
ASCII value = 89, Character = Y
ASCII value = 90, Character = Z
ASCII value = 91, Character = [
ASCII value = 92, Character = \
ASCII value = 93, Character = ]
ASCII value = 94, Character = ^
ASCII value = 95, Character = _
ASCII value = 96, Character = `
ASCII value = 97, Character = a
ASCII value = 98, Character = b
ASCII value = 99, Character = c
ASCII value = 100, Character = d
Bye <name board>

Exit QEMU by pressing :kbd:`CTRL+A` :kbd:`x`.
