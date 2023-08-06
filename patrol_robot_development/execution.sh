#!/usr/bin/env bash

BASEDIR=$(dirname "$0")
chmod +x $BASEDIR/scripts/*.sh

if [ "$1" == "welcome" ]; then
    $BASEDIR/scripts/execution-welcome.sh
elif [ "$1" == "patrol" ]; then
    $BASEDIR/scripts/execution-patrol.sh $2
elif [ "$1" == "gmap" ]; then
    $BASEDIR/scripts/execution-gmap.sh $2
elif [ "$1" == "generate" ]; then
    $BASEDIR/scripts/execution-generate.sh $2
else
    echo "SYNTAX: ./execution <version>. "
    echo "Where <version> is 'welcome', 'patrol', 'gmap' or 'generate'"
    echo "If version is gmap, patrol or generate, use parameter value for _map_name. Example: ./execution gmap plateau"
fi