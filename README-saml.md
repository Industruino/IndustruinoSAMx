# Industruino Core for SAML21B CPU

This repository contains the source code and configuration files of the Industruino Core
for Atmel's SAML21B processor (used on the Industruino 4-20mA.ker board).

This core is based on the [Arduino Core for SAMD21 CPU](https://github.com/arduino/ArduinoCore-samd).

The SAML21B specific code is derived/included from [MattairTech SAM D|L|C Core for Arduino](https://github.com/mattairtech/ArduinoCore-samd) delevoped by [Justin Mattair](https://github.com/mattairtech) of [MattairTech LLC](https://www.mattairtech.com).

## Installation on Arduino IDE

This core can be installed as a package by the Arduino IDE cores manager.

Add the following URL to the "Additional Boards Manager URLs" field of the Arduino IDE preferences

https://static.industruino.com/downloads/code/IndustruinoCores/IndustruinoSAML/pkgdef/package_industruino_saml_index.json

then open the "Boards Manager" and install the package called "Industruino SAML Boards (32-bit ARM Cortex-M0+)"

## Support

If you are looking for support, documentation or getting started directions, please check the page at the following URL:

http://industruino.com/support

## Bugs or Issues

If you find a bug you can submit an issue here on github:

https://github.com/Industruino/IndustruinoSAMx/issues

Before posting a new issue, please check if the same problem has been already reported by someone else
to avoid duplicates.

## Contributions

Contributions are always welcome. The preferred way to receive code contribution is by submitting a
Pull Request on GitHub.

## License and credits

This core has been developed by Arduino LLC in collaboration with Atmel.

The SAML21B specific code has been originally delevoped by [Justin Mattair](https://github.com/mattairtech) of [MattairTech LLC](https://www.mattairtech.com) ([MattairTech SAM D|L|C Core for Arduino](https://github.com/mattairtech/ArduinoCore-samd)).

```
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2017-2018 MattairTech LLC. All right reserved.
  Copyright (C) 2018 Industruino <connect@industruino.com>  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
```
