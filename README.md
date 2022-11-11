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

requirements:
```
    sudo apt-get install python-dev python-pip python-serial python3-serial 
    sudo pip install RPi.GPIO
    sudo pip install paho-mqtt
```
Data is read via USB using a [http://wiki.volkszaehler.org/hardware/controllers/ir-schreib-lesekopf](http://wiki.volkszaehler.org/hardware/controllers/ir-schreib-lesekopf)

For details, please see: [volkszaehler.org](http://wiki.volkszaehler.org/hardware/channels/meters/power/edl-ehz/emh-ehz-h1)

Length of stream in my case is 456 characters (ISKRA MT681).
