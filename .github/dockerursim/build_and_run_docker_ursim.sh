#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"


docker build ${DIR} -t mydockerursim
docker volume create dockerursim
docker run --name="mydockerursim" -d \
  -e ROBOT_MODEL=UR5 \
  -p 8080:8080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  -v "${DIR}/.vol":/ursim/programs \
  -v dockursim:/ursim \
  --privileged \
  --cpus=1 \
  mydockerursim
