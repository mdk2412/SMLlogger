# SMLlogger - Smart Message Language (SML)
## read SML-data (OBIS) from Zweirichtungszähler ISKRA MT681

based on 
- http://wiki.volkszaehler.org/hardware/channels/meters/power/edl-ehz/edl21-ehz
- http://wiki.volkszaehler.org/hardware/channels/meters/power/edl-ehz/emh-ehz-h1
- http://volkszaehler.org/pipermail/volkszaehler-users/2012-September/000451.html
- http://wiki.volkszaehler.org/software/sml
- http://www.mscons.net/obis_kennzahlen.pdf
- https://www.mikrocontroller.net/attachment/89888/Q3Dx_D0_Spezifikation_v11.pdf
- https://eclipse.org/paho/clients/python/
- https://www.stefan-weigert.de/php_loader/sml.php

requirements:
```
based on Raspberry Pi OS Lite (Legacy)

Release date: September 22nd 2022
System: 32-bit
Kernel version: 5.10
Debian version: 10 (buster))

sudo apt install python-serial python-pip librrd-dev libpython-dev
sudo pip install paho-mqtt
sudo pip install rrdtool
```
Data is read via IR transistor on RPI GPIO.

Length of stream is 456 characters (ISKRA MT681).
