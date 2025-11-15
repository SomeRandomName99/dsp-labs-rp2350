# Introduction
This project serves my introduction to DSP in general and real time signal processing in particular. The project will follow the [labs from EPFL's COM303 course](https://lcav.gitbook.io/dsp-labs/) which is very similar to their Coursera offering.

# Differences
For these sets of projects, I have decided to use another microcontroller, namely the [Raspberry Pi Pico 2 W](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html). Mainly because it has a built in DSP unit which can speed things up and pave the way to more elaborate projects(more on that soon). This of course comes with its own set of difficulties, since all of the starter code/project setup info needs to be done from scratch. But It should be worth it since I am very interested in trying a programmable I/O, multicore progeramming on an MCU, and built in DSP cores.

Another difference is that I will be using usb to transmit audio data because I lost the DAC I bought(little things get lost easily!).  I might add support to it again if I ever find it.

# Project checklist
- [X] USB audio passthrough (Also has DC Filtering and Mute and Volume controls) [Lab learnings](lcavDSPLabs/audioPassThrough/lab1_audioPassThrough/Lab1Learning.md)
- [x] Alien voice effect 
- [ ] Digital filter design (Currently taking a deep dive into filtering using 6.341x and EPFL DSP Specialization on coursera)
- [ ] Granular synthesis
- [ ] Linear prediction
- [ ] DFT pitch shifting

# Stretch goals
I would love to do some of the projects by Prof. [V. Hunter Adams](https://vanhunteradams.com), particularly the galton board project. But the rest are also very cool projects. I will have to decide on a subset of them, or just do them all. We will see.

# Acknowledgements
Special thanks to Martin Vetterli and Paolo Prandoni for their great course materials and published labs. Also thanks to V. Hunter Adams for his lectures on Rpi pico programming and interesting programming projects that have definitely piqued my interest.
