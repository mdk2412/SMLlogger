#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
########################################################################
#
#   SMLlogger:  
#		7.2.2016 Dirk Clemens 	iot@adcore.de
#		6.5.2017 Dirk Clemens	improvements, range check while converting hex to int
#		read data using a IR-USB-Head from a SML-counter (OBIS)
#		tested with "Zweirichtungszähler ISKRA MT681"
#	
#	based on 
#		http://wiki.volkszaehler.org/hardware/channels/meters/power/edl-ehz/edl21-ehz
#		http://wiki.volkszaehler.org/hardware/channels/meters/power/edl-ehz/emh-ehz-h1
#		http://volkszaehler.org/pipermail/volkszaehler-users/2012-September/000451.html
#		http://wiki.volkszaehler.org/software/sml
#		https://sharepoint.infra-fuerth.de/unbundling/obis_kennzahlen.pdf
#		https://www.mikrocontroller.net/attachment/89888/Q3Dx_D0_Spezifikation_v11.pdf
#		https://eclipse.org/paho/clients/python/ 
#
#	requirements:
#	sudo apt-get install python-dev python-pip python-serial python3-serial 
#	sudo pip install RPi.GPIO
#	sudo pip install paho-mqtt
#
########################################################################

import sys
import os
import serial
import time
import math
import paho.mqtt.publish as publish
import rrdtool
import logging

# ------------- #
# settings      #
# ------------- #

serialport='/dev/ttyAMA0'
broker='10.9.11.60'
bezug_rrd = "%s/strom-iskra.rrd" % (os.path.dirname(os.path.abspath(__file__)))
einspeisung_rrd= "%s/strom-iskra-einspeisung.rrd" % (os.path.dirname(os.path.abspath(__file__)))

# hex string to signed integer, inkl. range check http://stackoverflow.com/a/6727975 
def hexstr2signedint(hexval):
	uintval = int(hexval,16)
	if uintval > 0x7FFFFFFF:		# > 2147483647
		uintval -= 0x100000000		# -= 4294967296 
	return uintval

# parse hex string from USB serial stream and extract values for obis_id
# print result and publish as mqtt message 
def parseSML(data_hex, obis_id, obis_string, pos, length):
	obis_value = 0
	# find position of OBIS-Kennzahl 
	position = data_hex.find(obis_string)
	
	# break, do not send mqtt message
	if position <= 0:
		return 0 

	# extract reading from position start: 34 length: 10 (for 1.8.0.)
	hex_value = data_hex[position+pos:position+pos+length]
	
	# convert to integer, check range  
	obis_value = hexstr2signedint(hex_value)
	return obis_value

########################################################################
# MAIN  	
########################################################################
def main():

	# logging
	global logging
	logging.basicConfig(filename='/var/log/SMLlogger.log', 
					level=logging.DEBUG, 
					format='%(asctime)s %(message)s', 
					datefmt='%Y-%m-%d %H:%M:%S')

	#logger = logging.getLogger(__name__)
	#logger.setLevel(logging.DEBUG)

	# eHZ-Datentelegramme können mittels eines optischen Auslesekopfs nach DIN EN 62056-21 
	# z. B. über die serielle Schnittstelle eines PC ausgelesen werden.
	# Einstellung: bit/s= 9600, Datenbit = 7, Parität = gerade, Stoppbits = 1, Flusssteuerung = kein.
	ser = serial.Serial(
		port=serialport, 
		baudrate=9600, 
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		bytesize=serial.EIGHTBITS,
		timeout=1, 
		xonxoff=False, 
		rtscts=False, 
		dsrdtr=False)
	ser.flushInput()
	ser.flushOutput()


	data_hex = ''
	reading_ok = False
	
	while True:
		try:
			# read n chars, change that in case it's too short
			while True:
				data_raw = ser.read(40)
				#print(data_raw.encode("hex"))
	
			# find start escape sequence: 1b1b1b1b01010101
				if data_raw.encode("hex").find("1b1b1b1b01010101") >= 0 :
					data_raw += ser.read(406) #Gesamtlänge 456 Zeichen, letzte 3 Zeichen CRC
					reading_ok = True 
					break # found enough data, stop reading serial port
		except serial.serialutil.SerialException, e:
			reading_ok = False
			logging.debug("Error reading serial port: %s" % (e,))
			print("Error reading serial port: ", e)

		# convert reading to hex:
		data_hex = data_raw.encode("hex")
		# print (data_hex)

	if reading_ok:
		# tested with ISKRA MT681
            sml180 = parseSML(data_hex, "180", '070100010800ff', 42, 10) # Wirkenergie in Wh (total)
#            sml181 = parseSML(data_hex, "181", '070100010801ff', 34, 10) # Wirkenergie Tarif 1 in Wh (total)
#            sml182 = parseSML(data_hex, "182", '070100010802ff', 34, 10) # Wirkenergie Tarif 2 in Wh (total)
            sml167 = parseSML(data_hex, "167", '070100100700ff', 28, 8)  # Wirkleistung in W
            sml367 = parseSML(data_hex, "367", '070100240700ff', 28, 8)  # Wirkleistung L1 in W
            sml567 = parseSML(data_hex, "567", '070100380700ff', 28, 8)  # Wirkleistung L2 in W
            sml767 = parseSML(data_hex, "767", '0701004c0700ff', 28, 8)  # Wirkleistung L3 in W
            sml280 = parseSML(data_hex, "280", '070100020800ff', 34, 10)  # Wirkenergie in Wh (Einspeisung)
#            sml281 = parseSML(data_hex, "281", '070100020801ff', 34, 10)  # Wirkenergie Tarif 1 in Wh (Einspeisung)
#            sml282 = parseSML(data_hex, "282", '070100020802ff', 34, 10)  # Wirkenergie Tarif 2 in Wh (Einspeisung)
            msgs = [{'topic':"iskra_mt681/bezug", 'payload':(sml180/10000.0000)},
                    ("iskra_mt681/leistung-L", float(sml167)),
                ("iskra_mt681/leistung-L1", float(sml367)),
                ("iskra_mt681/leistung-L2", float(sml567)),
                ("iskra_mt681/leistung-L3", float(sml767)),
                ("iskra_mt681/einspeisung", sml280/10000.0000)]
            publish.multiple(msgs, hostname=broker, port=1883)
            try:
                rrdtool.update(bezug_rrd, 'N:%s:%s:%s:%s:%s' % ((sml180/10000.0000),(max(0, sml167)),(max(0, sml367)),(max(0, sml567)),(max(0, sml767))))
                rrdtool.update(einspeisung_rrd, 'N:%s:%s:%s:%s:%s' % ((sml280/10000.0000),(min(0, sml167)),(min(0, sml367)),(min(0, sml567)),(min(0, sml767))))
	    except rrdtool.OperationalError, e:
                logging.debug("RRDtool error:" % (e,))

if __name__ == "__main__":

    main()

