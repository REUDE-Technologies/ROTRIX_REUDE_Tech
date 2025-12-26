# REUDE_ROTRIX
Description
This report provides guidance on designing the hardware and firmware necessary to integrate with the ROTRIX application. ROTRIX is an online tool that connects to your motor thrust test bed, helping you select the optimal powertrain for your drone. It enables you to characterize and evaluate the performance of motor, propeller, and ESC combinations under various test conditions, allowing you to choose the best configuration. To use this tool, specific hardware components and custom firmware are needed to transmit the necessary data to the software. This report outlines the hardware requirements and firmware development process for seamless integration with ROTRIX.

## Introduction
The integration of a motor thrust test bed with the ROTRIX application requires a thorough understanding of both hardware and firmware. This manual guides users through selecting the appropriate components and developing firmware that ensures smooth data exchange between the test bed and the software. By following this guide, users can unlock ROTRIX's full potential to assess and optimize drone powertrain configurations. A complete documentation on the ROTRIX firmware can be found at this link.
## Hardware
This section outlines the hardware requirements essential for integrating the thrust test bed with the ROTRIX application. 
A list of potential hardware components that can be used to build the thrust bed and connect with the software can be viewed [here](https://github.com/REUDE-Technologies/REUDE_ROTRIX/blob/9da665a63edde41015f87db99a2621a0f7748934/Firmware/Hardware.pdf).

## Software
The integration process requires two primary software tools:

### Arduino IDE
An IDE that serves as the platform for developing and uploading the firmware to the microcontroller (eg. ArduinoIDE, STM32CubeIDE). It facilitates serial communication between the microcontroller and the ROTRIX application. 

### Python 3 
Python is essential for processing data on the software side, enabling interaction between the application and the hardware components.  
You will need to install version 3.10 or higher for this software and can be downloaded from the link Python Download link
Both hardware and software components play a critical role in ensuring accurate data collection and seamless communication between the firmware and the application.

## Firmware Development
The firmware acts as the bridge between the hardware and the software, processing sensor data and communicating with the ROTRIX application. This section outlines how to develop firmware that ensures reliable data transmission and compatibility with the application. 
The firmware (Arduino code) could be developed according to the sensors used by the user. However, the firmware must process sensor outputs into predefined formats and units that the application can interpret. Data must adhere to specific formats, as detailed in the subsequent sections, to ensure smooth communication. 
A reference source code for the firmware can be viewed here along with a circuit diagram illustrating the multi-sensor integration for sensors used in the source code in here.

## Reference Videos
Youtube Tutorials - https://www.youtube.com/playlist?list=PLsoiseyMdKCvdG4K64kRmZcRscF7sHM1e
