import ctypes
libs = ctypes.CDLL('./Usb3020.dll')
a = libs.USB3020_CreateDevice(0)

print(libs.USB3020_InitDeviceDA(a, 65536, ))