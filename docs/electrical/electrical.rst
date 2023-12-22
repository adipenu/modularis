Electrical
===========

This page links to all the electrical designs, calculations, trade-offs etc. 


The core functionality of Modularis is found
on the main PCB. Here is where all other boards and
external devices interface to process data or provide
power. This board supports three on-board computer
options and was designed to interface easily with any
selection. The main on-board computer for this
project was the Raspberry Pi 4, which contains ROS2
and handles the processing and decision-making for
incoming data. It interfaces with the thrusters
through an external pulse width modulation (PWM)
driver. The power board regulates the thruster power
and 5V power to all devices while also providing
protection to the battery and all components from
various electrical faults. The inertial measurement
board, which is one of future sensor addons, provides
a live tracking of acceleration, angular velocity, and
magnetic field to assist in orientation and thruster
control. An external SONAR board is also interfaced
with the main board to provide location data of the
AUV to prevent collisions.
Modularis is also equipped with a camera,
operated by a PWM servo. This is meant to allow for
object detection during missions and will work
alongside the thruster map to control speed through
PWM, which will in turn keep an object in the center
of the camera’s field-of-view. This continuous
tracking will permit the construction of a neural
radiance field (NeRF), which is a three-dimensional
model made from a collection of two-dimensional
images. Algorithms would be implemented for the
AUV to select the best angle of image capture to
provide the most useful information for the model.
The system architecture has always focused
on the key factor of modularity and ease of use at the
forefront. These design choices offer a dependable
AUV option, fostering advancements in underwater
vehicle navigation and control research. Its
architecture adopts a modular design in both
hardware and software, enabling the AUV to
facilitate diverse research applications.

HARDWARE DESIGN
--------------------
The hardware for Modularis is broken down
into three PCBs. Each board went through various
revisions from the previous and current semester.

.. toctree::
   :caption: Electronics Tree
   :maxdepth: 1

   Main Printed Circuit Board <mainBoard/mainBoard>
   Power System <power/power.rst>