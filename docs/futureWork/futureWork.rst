Future work
==================

For development work currently. Describes most tasks for Modularis from most crucial to least (Red being most immediate, 3 being least)

Threat level Red
-----------------------

- Power Pi from battery (**complete**)
- Modularis backup part system up and running- controlled by joystick
- Pose and image data to linux laptop

Threat level 1
--------------------

- Fix sonar mounts such that the sonar fits correctly
- Integrate Sonar such that Raspberry Pi receives sonar data into a node from the LattePanda board
- Acquire documentation on all mechanical designs
- Have Isabel and Daniel write about their designs after making outline

Threat level 2
---------------------

- Containerizing the existing ROS2 system and testing portability between Jetson Nano and Pi - Senior Design task
- Battery merge board for connecting power from both batteries into one
- Excel sheet where you can select which items you want
- Thruster connector: make a connector that allows for easy connect and disconnect of all thruster wires from the electronics enclosure for easy removal
- Make our own 3D printed black board for mounting ESC's
- Make modifications to enclosure wheels to give space for everything to fit appropriately
    - USB ports
    - More space for thruster wires
    - Main board “bend” issue

Threat level 3
------------------

- Test Modularis with larger (8 inch) tube
    - This is a larger project, but would allow for larger onboard computers
- Test Modularis docker container on TI board
- Make necessary structural readjustments to account for fitting TI board


Challenges
--------------------

- Battery balance not accessible to power board
- Neither is charger header since both connected to battery
- Efuse doesn’t work not enough amps
- PWM wires are super annoying to plug in through the hole in the main board. Need a simpler way.

Future Work Ideas
--------------

What if we made Modularis completely printable? Replacing all the parts from bluerobotics (besides tubes, perhaps?) Other groups are trying to do this, such as Yonder deep at UCSD, Scripps


