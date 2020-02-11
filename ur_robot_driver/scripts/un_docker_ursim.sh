#!/bin/bash
docker volume create dockerursim
docker run --name="dockerursim" -d \
  -e ROBOT_MODEL=UR5 \
  -p 8080:8080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  -v /path/to/mount/program/folder:/ursim/programs \
  -v dockursim:/ursim \
  --privileged \
  --cpus=1 \
  arranhs/dockursim:latest
