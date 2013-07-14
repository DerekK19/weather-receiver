#!/bin/bash
#
# Repeatedly run the WS1093 Weather station receiver
#
# Derek Knight
# July 2013
#
while $(sudo /usr/local/bin/ws1093_rf) -ge 0
do
	sleep 5
	echo Restarting
done
