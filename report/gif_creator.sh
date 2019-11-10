#!/bin/bash
# start from second 0 (ss) for a duration of 2.5s (-t) with a frame rate of 10 (-r)
ffmpeg -y -i $1 -ss 0 -r 10 -t 1.5 -vf scale=320:250 $2 -hide_banner