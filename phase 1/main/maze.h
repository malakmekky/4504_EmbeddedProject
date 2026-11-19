/*
 * maze.h - map of the maze as a graph + systematic exploration + shortest path.
 *
 * Every place the car stops to decide (intersection, corner, dead end, start/end
 * circle) is a NODE with:
 *   - coordinates (x, y)    : estimated from heading and driving time
 *   - exits                 : one boolean per direction N/E/S/W that has a line
 *   - neighbours + lengths  : which node each exit leads to (-1 = not explored yet)
 *
 * Directions are normalized: the car leaves the start facing N(orth), and every
 * 90 degree turn rotates the heading by one step (N -> E -> S -> W).
 *
 * Exploration (depth-first): take an unexplored exit if the current node has one,
 * otherwise drive (shortest known route) to the nearest node that still has one.
 * DONE when no node has an unexplored exit. Then Dijkstra gives the shortest
 * start -> end path, which is turned into a list of moves for the solve run.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MAZE_MAX_NODES 96
#define MAZE_MAX_MOVES 128

// directions relative to the car's heading = number of 90 degree steps clockwise
#define REL_S 0   // straight
#define REL_R 1   // right
#define REL_B 2   // back
#define REL_L 3   // left

// start a new exploration: start node at (0,0), heading north
void maze_reset(void);

// the car has reached a node
//   relExits : bits (1 << REL_x) for every branch seen (back is added automatically)
//   circle   : the start or end circle (all sensors black)
//   segMs    : driving time since the last node (distance)
void maze_arrive(uint8_t relExits, bool circle, uint32_t segMs);

// decide where to go from the current node and update the heading
// returns 'S', 'R', 'L', 'B', or 'D' when the whole maze has been explored
char maze_next_move(void);

// the car already went this way (overshot a corner), just update the map
void maze_force_move(char move);

bool maze_goal_found(void);

// Dijkstra from start to end -> best path (list of moves for the solve run)
bool maze_compute_best(void);

const char *maze_best(void);
int maze_best_len(void);
const char *maze_raw(void);     // every move taken while exploring

// send the whole map to the PC (MAP lines)
void maze_send_map(void);

// keep the best path in flash so it survives a reset between training and solving
bool maze_save(void);

// erase the saved path from flash and clear the map (the maze changed / new start circle)
void maze_forget(void);

// after training: plan the drive back to the start circle.
//   turnedAround : the car has just U-turned at a dead end (it faces back the way it came)
// returns the first thing to do: 'C' = back out of the circle we're in, 'H' = already home,
// 'L'/'S'/'R'/'B' = move at the current node, 0 = no route. Then maze_home() has one
// move per node on the way, like the solve path.
char maze_plan_home(bool turnedAround);
const char *maze_home(void);
int maze_home_len(void);
bool maze_load(void);
