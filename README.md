# ARDUINO-DGPS
This is a proposal of low cost solution of differential GPS based on low cost reciever modules uBlox NEO6M. The work has been done as part of Master Thesis composed at Linkoping University and CVUT in Prague. 

# Abstract
In today’s technical dimensions, there are various automotive gadgets as drones or automotive lawnmowers, which are widely used by private person to facilitate everyday life. Drones become more and more popular due to mobility and autonomous air operations. The continuous development leads to finding of diverse application for drones such as delivery service or filming.
These autonomous devices are used to navigate themselves and operate without human intervention. It predominantly uses a GPS as a source of position information, which is provided by GPS receiver attached to the device. However, these measurements are not accurate enough to estimate the exact position, which is one of the major requirements for autonomous operation. The main objective of this thesis is to propose the solution for improving the positioning accuracy, which is not expensive and easy to implement.
The Arduino DGPS solution proposes the algorithms for position accuracy improvements in two ways. The first mode uses a range residual computed by the receiver to estimate pseudorange corrections (PRCs) and corresponding position correction. The second mode works as a SBAS correction repeater, which uses the SBAS correction to acquire the position correction.
Keywords: Differential GPS, Arduino, Low-cost implementation
