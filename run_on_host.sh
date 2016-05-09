#!/bin/bash


interface="$(ls /dev/tty.usbserial* | head -1)"
port=115200

echo "$interface"

screen $interface $port -S installROS
