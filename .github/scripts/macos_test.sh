#!/bin/bash

# Python scripts installed with: `pip3 install --user <package>`
export PATH=$PATH:$HOME/Library/Python/3.11/bin

colcon test --merge-install
colcon test-result --all --verbose
