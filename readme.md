# Introduction
This project serves my introduction to DSP in general and real time signal processing in particular. The project will follow the [labs from EPFL's COM303 course]
(https://lcav.gitbook.io/dsp-labs/) which is very similar to their Coursera offering.

# Differences
For these sets of projects, I have decided to use another microcontroller, namely the [Raspberry Pi Pico 2 W]
(https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html). Mainly because it has a built in 
DSP unit which can speed things up and pave the way to more elaborate projects(more on that soon). This of course comes with its own set of difficulties, since
all of the starter code/project setup info needs to be done from scratch. But It should be worth it since I am very interested in trying a programmable I/O,
multicore progeramming on an MCU, and built in DSP cores.

Another difference is that I will be using usb to transmit audio data because I lost the DAC I bought(little things get lost easily!).
I might add support to it again if I ever find it.

# Project checklist
- [ ] USB audio passthrough(currently work in progress)
- [ ] Alien voice effect
- [ ] Digital filter design
- [ ] Granular synthesis
- [ ] Linear prediction
- [ ] DFT pitch shifting

# Stretch goals
I would love to make some of the projects by Prof. [V. Hunter Adams](https://vanhunteradams.com), particularly the galton board project.

#Acknowledgements
Special thanks to Martin Vetterli and Paolo Prandoni for their great course materials and published labs. Also thanks to V. Hunter Adams for his lectures on
Rpi pico programming and interesting labs.
