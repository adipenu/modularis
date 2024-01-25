Drivers
=========

This page documents all the drivers between the computer and the electronics. The drivers are written in C++ and are located in the ``drivers`` folder.

For movement, the AUV will operate
autonomously based either on camera data or a
predetermined script that will direct the movement of
the AUV. To control the movement of the AUV, a
thruster mapper along with the PCA96585PW PWM
Driver chip is used. The raspberry pi communicates
with the PCA96585PW via I2C communication to
control the pulse of the PWM signals output from the
chip. To control the current configuration of 6
thrusters, a total of 6 PWM signals is sent from the
chip with the possibility to control up to 8 thrusters
if desired. The PWM signals from the chip are
directly connected to the corresponding ESC
(electronic speed controller) for each thruster, which
is then responsible for controlling the speed and
direction of the thruster.

.. image:: pwm_driv_dia.png
   :width: 529px
   :height: 400px
   :align: center

.. toctree::
   :caption: Driver Tree
   :maxdepth: 1

   PWM Driver <pwmdriver/thruster_pwm_documentation>