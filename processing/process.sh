#!/bin/bash
# Example: ./process.sh ws_etsiit_test2.txt

DEVICE='"device_id":"wimunet-heltec-wireless-stick'
cat $1 | grep ${DEVICE} | awk -F '[,TZ"]' '{for(i=1;i<=NF;i++){if($i ~ /received_at/){print $(i+3); break}}}' | awk -F ':' '{ print 60*60*$1 + 60*$2 + $3 }' | awk 'NR > 1 { print $0 - prev } { prev = $0 }' | awk '{if(min==""){min=max=$1}; if($1>max) {max=$1}; if($1<min) {min=$1}; total+=$1; count+=1} END {print "avg " total/count," | max "max," | min " min}'

