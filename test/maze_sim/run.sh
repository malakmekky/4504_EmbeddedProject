#!/bin/sh
# Runs main/maze.c on the PC against simulated mazes (loops, dead ends, noisy distances).
# usage: ./run.sh [noise]   e.g. ./run.sh 0.15
cd "$(dirname "$0")"
cc -std=c11 -w -DSIM_NOISE=${1:-0.10} -Istub -I../../main sim.c ../../main/maze.c -o /tmp/maze_sim && /tmp/maze_sim | tail -25
