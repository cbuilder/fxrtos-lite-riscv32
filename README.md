Description
-----------

This repository contains preconfigured version of FX-RTOS Lite for RISC-V.
Linux host support will be available soon.

Note that the kernel has limitations which are listed in [main repository](https://github.com/Eremex/fxrtos-lite).

Getting started
---------------

How to build the library from sources:

- Ensure supported compiler is available via PATH
- Set GCC_PREFIX as compiler prefix if you use GCC (i.e. 'riscv-none-embed-')
- Enter directory where build.bat is located
- Run 'build.bat'

Prebuilt kernel binary is available in 'demo' folder (FXRTOS.h and libfxrtos.a).
