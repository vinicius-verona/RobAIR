#!/usr/bin/env bash

BASEDIR=$(dirname "$0")
chmod +x $BASEDIR/scripts/*.sh

if [ "$1" == "welcome" ]; then
    $BASEDIR/scripts/execution-welcome.sh
elif [ "$1" == "patrol" ]; then
    $BASEDIR/scripts/execution-patrol.sh
else 
    echo "SYNTAX: ./execution <version>"
    echo "Where <version> is either 'welcome' or 'patrol'"
fi