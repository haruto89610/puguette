## Software and Simulation

The robot has went through different versions with different architectures. The old version aimed to use a Jetson Nano as well as a Teensy to control the 6 ODrives. As a result, part of the code was written in C++ as well as Python. However, errors were encountered where the UART signals had incorrect values. This could possibly have originated from noise, or the large amount of information needed to be transmitted. Although lowering the baudrate resulted in cleaner results, completely removing the issue was a difficult problem to overcome. As a result, the Teensy was removed, and the Jetson Nano communicates directly with the ODrives via USB. As a result, the new program uses only Python.

Because testing the robot is not always easy, considering the sheer size of the robot, a PyBullet simulation was made to attempt to help with testing. Because the polygon count on the actual CAD is too high, rendering the model was an issue. To solve this issue, the simulation uses a simplified version of the CAD model.