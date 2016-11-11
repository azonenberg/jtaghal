# jtaghal
JTAG Hardware Abstraction Library

Provides an object-oriented interface to JTAG-accessible devices, JTAG adapters, and so on. Intended to be easily
extensible so that applications can add support for custom in-system debug features on FPGAs, etc.

Currently in the process of being imported from SVN
(http://redmine.drawersteak.com/projects/achd-soc/repository/show/trunk/src/jtaghal) and ported to splash v0.2.

Cannot yet be built in a useful form, but if you're curious the legacy/ directory has all of the old code we haven't
yet ported.

The following device support info applies to the legacy build. Many of these are not yet supported in the port.

FTDI-based adapters:
* Digilent HS1
* azonenberg usb-jtag-mini

Digilent-based adapters:
* All (tested HS1, AC701)

Network-based adapters
* libjtaghal jtagd
* Antikernel NetworkedJtagInterface
