#!/bin/bash

export GZ_VERSION=harmonic
colcon test --merge-install
colcon test-result --all --verbose
