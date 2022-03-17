#!/bin/bash

# build
python3 -m build

# install
pip3 install  --no-index --find-links=./dist/ TMC_2209_Raspberry_Pi