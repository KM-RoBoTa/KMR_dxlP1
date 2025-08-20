# C++ wrapper around Dynamixel's library for protocol 1 motors

> [!warning]
> This library has been refactored to increase optimization and ease-of-use. <br /> 
> This version 2.0.0 is **not** compatible with code written for version 1.0.0. <br /> 
> The old library version can be found on the branch "v1.0.0", although it is recommended to use this new version

> [!note]
> Remaining refactor tasks:
> - comment code
> - do markdown documentation 
> - finish and add examples

Library for an easy use of Dynamixel motors.  
It abstracts the hardware (no need to concern yourself with memory addresses) and automates the creation of reading/writing handlers.

It works with Dynamixel's **protocol 1**.

Dependencies:
- dynamixel API

How to use: go to the KMR_dxlP1 folder, then: 
```bash
mkdir build
cd build
cmake ../
cmake --build .
```

If you have Doxygen and Graphviz installed, you can regenerate the documentation locally with
```bash
make docs
```
from the `build` folder after the cmake.  
The generated documentation can be found in "docs/generated_docs/html".

