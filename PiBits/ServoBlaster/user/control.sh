#!/bin/bash

speed=1270us
echo "Nuetral speed: $speed"
echo 0=${speed} > /dev/servoblaster
while true; do
    read speed
    echo "DBG: speed=$speed"
    echo 0=${speed}us > /dev/servoblaster
done
