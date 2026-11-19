/*
 * maze.c - graph map, depth-first exploration, Dijkstra shortest path.
 *
 * Coordinates: x/y in "ms of driving at NORMAL_SPEED" (no wheel encoders, so time
 * is the distance). N = +y, E = +x. Because every turn is 90 degrees, a segment
 * only ever changes one coordinate, so the error builds up slowly.
 *
 * Recognising a node we've seen before:
 *   - driving down an edge we already know -> we know exactly which node it leads to
 *   - driving down a NEW edge -> look for an existing node that is close to the
 *     estimated position AND has an unexplored exit pointing back at us.
 *     If one exists we just closed a loop, otherwise it's a new node.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nvs_flash.h"
#include "nvs.h"

#include "config.h"
#include "ble_link.h"
#include "maze.h"

typedef struct {
  int32_t x, y;
  uint8_t exits;            // bit d = there is a line leaving in direction d (N E S W)
  bool circle;              // start or end circle
  int16_t nbr[4];           // node reached through exit d, -1 = not explored
  uint32_t len[4];          // length of that edge (ms)
} node_t;

static node_t nodes[MAZE_MAX_NODES];
static int nodeCount = 0;
static int curNode = 0;
static int goalNode = -1;
static int heading = 0;               // absolute direction the car faces, 0 = N
static bool exploreDone = false;

// route to the nearest unexplored branch (node ids), routePos = node we are at
static int16_t route[MAZE_MAX_NODES];
static int routeLen = 0;
static int routePos = 0;

static const char *arriveHow = "";    // printed with the decision
static uint32_t driftMs = 0;          // distance driven since we last knew for sure where we are
static bool atCircle = false;         // just arrived in the start or end circle
static int dijkstraAvoid = -1;        // node the route must not drive through (the end circle on the way home)
static char homePath[MAZE_MAX_MOVES + 1];   // moves to drive back to the start after training
static int homeLen = 0;
static int circleBack = 0;            // direction to leave that circle (back the way we came)

static char rawPath[MAZE_MAX_MOVES + 1];
static int rawLen = 0;
static char bestPath[MAZE_MAX_MOVES + 1];
static int bestLen = 0;
static int16_t bestNodes[MAZE_MAX_NODES];
static int bestNodeCount = 0;

static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };
static const char DIR_CH[] = "NESW";
static const char REL_CH[] = "SRBL";

#define NO_DIST 0xFFFFFFFFu

// ---------- helpers ----------

static int opposite(int d) {
  return (d + 2) % 4;
}

static int newNode(int32_t x, int32_t y, uint8_t exits, bool circle) {

  if (nodeCount >= MAZE_MAX_NODES)
    return -1;

  node_t *n = &nodes[nodeCount];
  n->x = x;
  n->y = y;
  n->exits = exits;
  n->circle = circle;
  for (int d = 0; d < 4; d++) {
    n->nbr[d] = -1;
    n->len[d] = 0;
  }
  return nodeCount++;
}

static void connect(int a, int d, int b, uint32_t len) {
  nodes[a].nbr[d] = b;
  nodes[a].len[d] = len;
  nodes[a].exits |= 1 << d;
  nodes[b].nbr[opposite(d)] = a;
  nodes[b].len[opposite(d)] = len;
  nodes[b].exits |= 1 << opposite(d);
}

static bool hasUnexplored(int i) {
  for (int d = 0; d < 4; d++)
    if ((nodes[i].exits >> d & 1) && nodes[i].nbr[d] < 0)
      return true;
  return false;
}

static int dirTo(int a, int b) {
  for (int d = 0; d < 4; d++)
    if (nodes[a].nbr[d] == b)
      return d;
  return -1;
}

static void exitsStr(uint8_t e, char *out) {
  int k = 0;
  for (int d = 0; d < 4; d++)
    if (e >> d & 1)
      out[k++] = DIR_CH[d];
  if (k == 0)
    out[k++] = '-';
  out[k] = '\0';
}

// shortest distance from src to every node over the edges we know
static void dijkstra(int src, uint32_t *dist, int16_t *prev) {

  bool done[MAZE_MAX_NODES] = { 0 };

  for (int i = 0; i < nodeCount; i++) {
    dist[i] = NO_DIST;
    prev[i] = -1;
  }
  dist[src] = 0;

  while (1) {

    int u = -1;
    for (int i = 0; i < nodeCount; i++)
      if (!done[i] && dist[i] != NO_DIST && (u < 0 || dist[i] < dist[u]))
        u = i;

    if (u < 0)
      break;
    done[u] = true;

    if (u == dijkstraAvoid && u != src)
      continue;                       // can arrive here, but not drive through it

    for (int d = 0; d < 4; d++) {
      int v = nodes[u].nbr[d];
      if (v >= 0 && dist[u] + nodes[u].len[d] < dist[v]) {
        dist[v] = dist[u] + nodes[u].len[d];
        prev[v] = u;
      }
    }
  }
}

// existing node ahead of us (direction h from prev) near (x, y), whose exit `back`
// points at us and isn't explored yet. The allowed error grows with the distance
// driven since the last node we were sure about.
static int findMatch(int prev, int h, int32_t x, int32_t y, uint32_t segMs, uint8_t seen) {

  int back = opposite(h);
  int32_t tol = (int32_t)((driftMs + segMs) * MATCH_TOL_FRAC);
  if (tol < MATCH_TOL_MIN)
    tol = MATCH_TOL_MIN;

  int best = -1;
  int32_t bestDist = 0;

  for (int i = 0; i < nodeCount; i++) {

    node_t *n = &nodes[i];
    if (i == prev || n->circle) continue;
    if (!(n->exits >> back & 1) || n->nbr[back] >= 0) continue;

    // it must also look like that node: the exits we see now may differ from the ones
    // stored by at most one (a branch can be missed once), otherwise it's another node
    if (__builtin_popcount((n->exits ^ seen) & 0x0F) > MATCH_EXIT_DIFF) continue;

    // must be in front of the car, not behind it
    int32_t ahead = (n->x - nodes[prev].x) * DX[h] + (n->y - nodes[prev].y) * DY[h];
    if (ahead <= 0) continue;

    int32_t dx = abs(n->x - x);
    int32_t dy = abs(n->y - y);
    if (dx > tol || dy > tol) continue;

    if (best < 0 || dx + dy < bestDist) {
      best = i;
      bestDist = dx + dy;
    }
  }

  return best;
}

// a circle reached through a new edge. The maze has only two circles (start and end),
// so it is one of the circles we already know, never a third one:
//   - end not found yet: it is the start again if we are near it, otherwise it is the end
//   - end already found: whichever of the two is closer to where we think we are
// returns 0 (start), goalNode, or -1 (a circle we haven't seen: the end)
static int matchCircle(int32_t x, int32_t y, uint32_t segMs) {

  int32_t ds = abs(x - nodes[0].x) + abs(y - nodes[0].y);

  if (goalNode < 0) {
    int32_t tol = (int32_t)((driftMs + segMs) * CIRCLE_TOL_FRAC);
    if (tol < CIRCLE_TOL_MIN) tol = CIRCLE_TOL_MIN;
    if (tol > CIRCLE_TOL_MAX) tol = CIRCLE_TOL_MAX;
    return ds <= tol ? 0 : -1;
  }

  int32_t dg = abs(x - nodes[goalNode].x) + abs(y - nodes[goalNode].y);
  return ds <= dg ? 0 : goalNode;
}

// leave the current node in absolute direction d
static char depart(int d, const char *why) {

  char move = REL_CH[(d - heading + 4) % 4];
  node_t *n = &nodes[curNode];
  char ex[5];
  exitsStr(n->exits, ex);

  link_send("NODE %d (%ld,%ld) exits=%s %s -> %c %s",
            curNode, (long)n->x, (long)n->y, ex, arriveHow, move, why);

  heading = d;

  if (rawLen < MAZE_MAX_MOVES) {
    rawPath[rawLen++] = move;
    rawPath[rawLen] = '\0';
  }

  return move;
}

// ---------- exploration ----------

void maze_reset(void) {
  nodeCount = 0;
  newNode(0, 0, 1 << 0, true);        // node 0 = start circle, only exit is north
  curNode = 0;
  goalNode = -1;
  heading = 0;
  exploreDone = false;
  driftMs = 0;
  atCircle = false;
  routeLen = 0;
  rawLen = 0;
  rawPath[0] = '\0';
  bestLen = 0;
  bestPath[0] = '\0';
  bestNodeCount = 0;
  homeLen = 0;
  homePath[0] = '\0';
}

void maze_arrive(uint8_t relExits, bool circle, uint32_t segMs) {

  int h = heading;
  int back = opposite(h);

  // what the sensors saw, turned into absolute directions (+ the way we came in)
  uint8_t absExits = 1 << back;
  if (!circle)
    for (int r = 0; r < 4; r++)
      if (relExits >> r & 1)
        absExits |= 1 << ((h + r) % 4);

  int prev = curNode;
  node_t *p = &nodes[prev];
  int cur;

  if (p->nbr[h] >= 0) {

    // known edge: we know where we are, refine its length
    cur = p->nbr[h];
    uint32_t len = (p->len[h] + segMs) / 2;
    p->len[h] = len;
    nodes[cur].len[back] = len;
    arriveHow = "known";
    driftMs = 0;

  } else {

    int32_t x = p->x + DX[h] * (int32_t)segMs;
    int32_t y = p->y + DY[h] * (int32_t)segMs;

    if (circle) {
      cur = matchCircle(x, y, segMs);

      if (cur == 0) {
        // back in the START circle: we know exactly where we are again (0,0), and since
        // the start has one line we must have come down it heading south. The edge is only
        // stored on our side, so the solve run still always leaves the start northwards.
        p->nbr[h] = 0;
        p->len[h] = segMs;
        curNode = 0;
        heading = 2;                  // S
        circleBack = 0;               // leave north again
        atCircle = true;
        driftMs = 0;
        arriveHow = "START again";
        return;
      }
    } else {
      cur = findMatch(prev, h, x, y, segMs, absExits);
    }

    if (cur >= 0) {
      arriveHow = circle ? "END again" : "loop";
      driftMs = 0;
    } else {
      cur = newNode(x, y, absExits, circle);
      if (cur < 0) {
        link_send("ERR map full (%d nodes)", MAZE_MAX_NODES);
        exploreDone = true;
        return;
      }
      arriveHow = "new";
      driftMs += segMs;
    }

    connect(prev, h, cur, segMs);
  }

  if (!circle)
    nodes[cur].exits |= absExits;     // a branch missed last time gets added now

  if (circle && cur != 0 && goalNode < 0) {
    nodes[cur].circle = true;
    goalNode = cur;
    arriveHow = "END";
  }

  if (circle) {
    atCircle = true;
    circleBack = back;                // the car backs out and turns around on the same line
  }

  curNode = cur;
}

char maze_next_move(void) {

  if (exploreDone)
    return 'D';

  node_t *n = &nodes[curNode];

  // 0. in a circle the car always backs out on the line it came in on
  if (atCircle) {
    atCircle = false;
    bool left = false;
    for (int i = 0; i < nodeCount && !left; i++)
      left = hasUnexplored(i);
    if (!left) {
      exploreDone = true;
      link_send("NODE %d %s -> every branch explored", curNode, arriveHow);
      return 'D';
    }
    routeLen = 0;
    return depart(circleBack, "leave circle");
  }

  // 1. an unexplored branch right here: right, straight, left, back
  static const int PREF[4] = { REL_R, REL_S, REL_L, REL_B };
  for (int k = 0; k < 4; k++) {
    int d = (heading + PREF[k]) % 4;
    if ((n->exits >> d & 1) && n->nbr[d] < 0) {
      routeLen = 0;
      return depart(d, "explore");
    }
  }

  // 2. keep following the route we planned
  if (routeLen > 0 && route[routePos] == curNode && routePos + 1 < routeLen) {
    int next = route[++routePos];
    if (routePos == routeLen - 1)
      routeLen = 0;
    int d = dirTo(curNode, next);
    if (d >= 0)
      return depart(d, "route");
  }
  routeLen = 0;

  // 3. plan a route to the nearest node that still has an unexplored branch
  uint32_t dist[MAZE_MAX_NODES];
  int16_t prev[MAZE_MAX_NODES];
  dijkstra(curNode, dist, prev);

  int target = -1;
  for (int i = 0; i < nodeCount; i++)
    if (hasUnexplored(i) && dist[i] != NO_DIST && (target < 0 || dist[i] < dist[target]))
      target = i;

  if (target < 0) {
    exploreDone = true;
    link_send("NODE %d %s -> every branch explored", curNode, arriveHow);
    return 'D';
  }

  int len = 0;
  int16_t tmp[MAZE_MAX_NODES];
  for (int v = target; v >= 0 && len < MAZE_MAX_NODES; v = prev[v])
    tmp[len++] = v;
  for (int i = 0; i < len; i++)
    route[i] = tmp[len - 1 - i];

  routeLen = len;
  routePos = 1;
  if (routePos == routeLen - 1)
    routeLen = 0;

  link_send("PLAN go to node %d (%d edges)", target, len - 1);
  return depart(dirTo(curNode, route[1]), "backtrack");
}

void maze_force_move(char move) {
  const char *p = strchr(REL_CH, move);
  int d = (heading + (p ? (int)(p - REL_CH) : 0)) % 4;
  nodes[curNode].exits |= 1 << d;
  routeLen = 0;
  depart(d, "forced");
}

bool maze_goal_found(void) {
  return goalNode >= 0;
}

// ---------- shortest path ----------

bool maze_compute_best(void) {

  bestLen = 0;
  bestPath[0] = '\0';
  bestNodeCount = 0;

  if (goalNode < 0)
    return false;

  uint32_t dist[MAZE_MAX_NODES];
  int16_t prev[MAZE_MAX_NODES];
  dijkstra(0, dist, prev);

  if (dist[goalNode] == NO_DIST)
    return false;

  int16_t tmp[MAZE_MAX_NODES];
  int k = 0;
  for (int v = goalNode; v >= 0 && k < MAZE_MAX_NODES; v = prev[v])
    tmp[k++] = v;
  for (int i = 0; i < k; i++)
    bestNodes[i] = tmp[k - 1 - i];
  bestNodeCount = k;

  // one move per node between start and end
  for (int i = 1; i < k - 1; i++) {

    int in = dirTo(bestNodes[i - 1], bestNodes[i]);
    int out = dirTo(bestNodes[i], bestNodes[i + 1]);
    int rel = (out - in + 4) % 4;

    // a node with no side branch (only straight through) won't be detected while
    // solving, so it must not use up a move
    uint8_t e = nodes[bestNodes[i]].exits;
    if (rel == REL_S && e == ((1 << opposite(in)) | (1 << out)))
      continue;

    if (bestLen < MAZE_MAX_MOVES)
      bestPath[bestLen++] = REL_CH[rel];
  }
  bestPath[bestLen] = '\0';

  link_send("SHORTEST length=%lu ms through %d nodes", (unsigned long)dist[goalNode], k);
  return true;
}

// ---------- drive back to the start after training ----------

char maze_plan_home(bool turnedAround) {

  homeLen = 0;
  homePath[0] = '\0';

  int cur = curNode;
  if (cur == 0)
    return 'H';                       // already in the start circle

  // where the car really is and which way it faces
  int from, inDir;
  char first = 0;
  int h = turnedAround ? opposite(heading) : heading;

  if (nodes[cur].circle) {
    first = 'C';                      // back out of the circle and turn round first
    inDir = circleBack;
    from = nodes[cur].nbr[inDir];
  } else {
    from = cur;
    inDir = h;
  }
  if (from < 0)
    return 0;

  uint32_t dist[MAZE_MAX_NODES];
  int16_t prev[MAZE_MAX_NODES];
  dijkstraAvoid = goalNode;           // don't drive through the end circle on the way
  dijkstra(from, dist, prev);
  dijkstraAvoid = -1;

  if (dist[0] == NO_DIST)
    return 0;

  int16_t tmp[MAZE_MAX_NODES], path[MAZE_MAX_NODES];
  int k = 0;
  for (int v = 0; v >= 0 && k < MAZE_MAX_NODES; v = prev[v])
    tmp[k++] = v;
  for (int i = 0; i < k; i++)
    path[i] = tmp[k - 1 - i];

  int startIdx = 0;
  if (first == 0) {
    // at a normal node: the first move is made right here
    if (k < 2)
      return 0;
    first = REL_CH[(dirTo(path[0], path[1]) - h + 4) % 4];
    startIdx = 1;
  }

  // one move per node on the way, like the solve run
  for (int i = startIdx; i < k - 1; i++) {
    int in = (i == 0) ? inDir : dirTo(path[i - 1], path[i]);
    int out = dirTo(path[i], path[i + 1]);
    int rel = (out - in + 4) % 4;
    uint8_t e = nodes[path[i]].exits;
    if (rel == REL_S && e == ((1 << opposite(in)) | (1 << out)))
      continue;                       // straight-through node, not detected while driving
    if (homeLen < MAZE_MAX_MOVES)
      homePath[homeLen++] = REL_CH[rel];
  }
  homePath[homeLen] = '\0';
  return first;
}

const char *maze_home(void) { return homePath; }
int maze_home_len(void)     { return homeLen; }

const char *maze_best(void) { return bestPath; }
int maze_best_len(void)     { return bestLen; }
const char *maze_raw(void)  { return rawPath; }

// ---------- map to the PC ----------

void maze_send_map(void) {

  if (nodeCount <= 1)
    return;

  link_send_wait("MAP BEGIN %d", nodeCount);

  for (int i = 0; i < nodeCount; i++) {
    char ex[5];
    exitsStr(nodes[i].exits, ex);
    link_send_wait("MAP N %d %ld %ld %s %c", i, (long)nodes[i].x, (long)nodes[i].y, ex,
                   i == 0 ? 'S' : (i == goalNode ? 'E' : '-'));
  }

  for (int i = 0; i < nodeCount; i++)
    for (int d = 0; d < 4; d++)
      if (nodes[i].nbr[d] > i)
        link_send_wait("MAP E %d %d %lu", i, nodes[i].nbr[d], (unsigned long)nodes[i].len[d]);

  // shortest path as node ids, a few per line
  char line[LINK_MSG_LEN];
  int pos = 0;
  for (int i = 0; i < bestNodeCount; i++) {
    if (pos == 0)
      pos = snprintf(line, sizeof(line), "MAP P");
    pos += snprintf(line + pos, sizeof(line) - pos, " %d", bestNodes[i]);
    if (pos > LINK_MSG_LEN - 12 || i == bestNodeCount - 1) {
      link_send_wait("%s", line);
      pos = 0;
    }
  }

  link_send_wait("MAP END");
}

// ---------- flash ----------

bool maze_save(void) {

  nvs_handle_t h;
  if (nvs_open("maze", NVS_READWRITE, &h) != ESP_OK)
    return false;

  bool ok = nvs_set_str(h, "raw", rawPath) == ESP_OK &&
            nvs_set_str(h, "best", bestPath) == ESP_OK &&
            nvs_commit(h) == ESP_OK;

  nvs_close(h);
  return ok;
}

void maze_forget(void) {

  maze_reset();

  nvs_handle_t h;
  if (nvs_open("maze", NVS_READWRITE, &h) == ESP_OK) {
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
  }
}

bool maze_load(void) {

  nvs_handle_t h;
  if (nvs_open("maze", NVS_READONLY, &h) != ESP_OK)
    return false;

  size_t len = sizeof(rawPath);
  if (nvs_get_str(h, "raw", rawPath, &len) == ESP_OK)
    rawLen = strlen(rawPath);

  len = sizeof(bestPath);
  bool ok = nvs_get_str(h, "best", bestPath, &len) == ESP_OK;
  bestLen = ok ? strlen(bestPath) : 0;

  nvs_close(h);
  return ok && bestLen > 0;
}
