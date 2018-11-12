# MCP library for ESP-IDF

* This C++ library covers MCP-Standard, MCP-Advanced and Roboclaw motor controllers. Some of the functions can be used only for Advanced model.
* Don't forget to enable C++ exceptions (menuconfig/compiler options)
* Read-Write buffer and maximum retry for serial communication can be changed in the header file.  
* Library uses standard ESP-IDF serial communication(which means it uses built-in locks/semaphores)
* Example uses a semaphore to overcome race condition/task-conflicts.
