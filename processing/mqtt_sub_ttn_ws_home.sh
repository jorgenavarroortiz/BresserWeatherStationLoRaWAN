#!/bin/bash
SERVER='eu1.cloud.thethings.network'
#TOPIC='#'
TOPIC='v3/wimunet-application01@ttn/devices/wimunet-heltec-wireless-stick-02/up'
APPID='wimunet-application01'
APIKEY='NNSXS.UMBJVX3XHUE3BCY5A3UNSO354HO6DVRXHSYZENQ.FI6FFS6SO4MZBGNZU3WDIERUJLQ7REHMMXK6UGXLCYISOSRN3SPA'

mosquitto_sub -d -h $SERVER -u $APPID -P $APIKEY -p 1883 -t $TOPIC
