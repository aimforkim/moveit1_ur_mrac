#!/usr/bin/env bash

echo -e "Building moveit1_ur:lastest \nWARNING: this script must be run from the root of the repo not from within the .docker folder"
# build                        location dockerfile        name:tag if you want to change tag*
docker build --pull --rm -f ./.docker/Dockerfile_zsh  -t moveit1_ur:latest .
