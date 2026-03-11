#!/usr/bin/env bash

colcon build \
  --parallel-workers "$(nproc)" \
  --executor parallel \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
