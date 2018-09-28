#!/bin/bash         

hppcorbaserver & 
gepetto-viewer-server &
ipython -i --no-confirm-exit ./$1

pkill -f  'gepetto-viewer-server'
pkill -f  'hppcorbaserver'
