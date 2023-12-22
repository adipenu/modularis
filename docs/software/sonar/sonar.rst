Starfish Side-Scan Sonar
==========================

One of the sensors the AUV supports, the
Starfish Side Scan Sonar, is not compatible for use
with Linux systems. To extract any data from thesensor, we need to use the provided software,
Scanline, which is only supported on Windows OS.
To still make it possible to use this sonar, we decided
to include another single board computer, the
LattePanda Delta 432, that can hold the Windows
OS. The LattePanda will run the Scanline software
and a python script to start data capture and export
the data to .CSV format. The python script will then
parse the file to extract only the side scan sonar data
to then send over serial connection to the raspberry
pi for use by the pi as desired.