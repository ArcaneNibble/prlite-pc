#!/bin/sh 

dhttpd -p 8181 -r `rospack find pr2lite_rosbridge`

