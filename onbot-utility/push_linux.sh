#!/bin/bash
cd "$(dirname "$0")"

./ftc_http_linux -u "../teamcode/" -b

#EXIT_STATUS=$?
#
#echo $EXIT_STATUS
#
#
#if [ $EXIT_STATUS -eq 0 ]; then
#    notify-send "Build Finished" "Onbot build finished successfully"
#    paplay /usr/share/mint-artwork/sounds/plug.oga
#elif [ $EXIT_STATUS -eq 3 ]; then
#    notify-send -u critical "Build Failed" "Could not find the robot, is it connected?"
#    paplay /usr/share/mint-artwork/sounds/unplug.oga
#else
#    notify-send -u critical "Build Failed" "Onbot build failed, see the console for details"
#    paplay /usr/share/mint-artwork/sounds/unplug.oga
#fi