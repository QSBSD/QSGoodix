<div align="center">
  <img src="https://avatars.githubusercontent.com/u/329755507?s=200&v=4" alt="QSGoodix" width="120" height="120">

  # QSGoodix

  Goodix touchscreen driver (I2C, ACPI)

  [![License: BSD 2-Clause](https://img.shields.io/badge/License-BSD%202--Clause-blue.svg)](./LICENSE)

  [![Русский](https://img.shields.io/badge/lang-Русский-lightgrey)](README.md)
  [![English](https://img.shields.io/badge/lang-English-blue)](README.en.md)
</div>

## About

QSGoodix is a FreeBSD kernel module for Goodix touchscreens connected over I2C and described in ACPI.

> [!WARNING]
> **Tested:** GDIX1002, GT9110, 3840x2190 grid, 10 touches.
>
> **Not tested:** other Goodix chips using the same protocol (GT911, GT912, GT927, GT928, GT967, GT9147, GT9271). They require changes in `gt_acpi_ids` and `gt_identify`. GT1151, GT5663, GT5688, GT917S and GT9286 additionally require reading the config at address 0x8050. Reports of operation on other hardware are welcome via issues.

## Features

**Multitouch via evdev**
 - Up to 10 simultaneous touches, ABS_MT protocol (slots)

**Interrupt or polling**
 - If the interrupt is unavailable or does not fire, the driver polls the controller: slowly when idle, quickly during a touch

**Controller reset via GPIO**
 - The INT and RST lines are taken from `_CRS`. If the controller does not respond on the bus (at startup, after resume, or after a series of errors), the driver resets it via RST

**Suspend/resume**
 - If the GPIO lines are found, the controller is put to sleep on suspend and woken on resume

**Axis and panel configuration**
 - invert_x, invert_y - mirror the axes
 - swap_xy - swap the X and Y axes (only before the module is loaded)
 - x_override, y_override - chip grid size along the X and Y axes (before swap_xy) instead of the value from the config
 - poll_active_hz, poll_idle_hz - polling frequency
 - suspend_sleep - put the controller to sleep on suspend
 - debug - debug mask: 1 init/power, 2 bus errors, 4 touch reports

All parameters are set in `/boot/loader.conf` as `dev.goodix_acpi.0.<name>="value"`. `swap_xy`, `x_override` and `y_override` take effect only when the module is loaded; the rest can be changed on the fly via `sysctl dev.goodix_acpi.0`.

## Build / Run

**FreeBSD**

```
doas git clone --depth 1 --filter=blob:none --sparse -b releng/$(freebsd-version -k | cut -d- -f1) https://git.FreeBSD.org/src.git /usr/src
doas git -C /usr/src sparse-checkout set sys

make
doas kldload ig4 ./QSGoodix.ko
```

## Technologies

 - C, iicbus, ACPI, gpiobus, evdev

## License

Distributed under the terms of the BSD 2-Clause license, as stated
in the [LICENSE](./LICENSE) file.

```
Copyright (c) 2026, QSBSD Contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```