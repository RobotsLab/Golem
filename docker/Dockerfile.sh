#!/bin/bash

TAG=$1
docker image build -t ${TAG} . -f Dockerfile
