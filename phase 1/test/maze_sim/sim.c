// simulator: true maze on a grid, robot reports what its sensors would see
#ifndef SIM_NOISE
#define SIM_NOISE 0.10   // +-10% error on every distance
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "maze.h"

int verbose = 1;
void link_send(const char *f, ...) { if(!verbose) return; va_list a; va_start(a,f); vprintf(f,a); va_end(a); printf("\n"); }
void link_send_wait(const char *f, ...) { va_list a; va_start(a,f); vprintf(f,a); va_end(a); printf("\n"); }

typedef struct { int x, y; int nb[4]; } tnode;   // nb = index of neighbour in dir N E S W
tnode T[32]; int TN = 0; int startN, goalN;
int add(int x,int y){ T[TN].x=x; T[TN].y=y; for(int d=0;d<4;d++) T[TN].nb[d]=-1; return TN++; }
int at(int x,int y){ for(int i=0;i<TN;i++) if(T[i].x==x&&T[i].y==y) return i; return add(x,y); }
void edge(int x1,int y1,int x2,int y2){ int a=at(x1,y1), b=at(x2,y2); int d = x2>x1?1: x2<x1?3: y2>y1?0:2; T[a].nb[d]=b; T[b].nb[(d+2)%4]=a; }
int deg(int i){int c=0; for(int d=0;d<4;d++) c+=T[i].nb[d]>=0; return c;}
int len(int a,int d){ int b=T[a].nb[d]; return abs(T[a].x-T[b].x)+abs(T[a].y-T[b].y); }

double noise = 0.10;
uint32_t segOf(int a,int d){ double r = 1.0 + noise*(2.0*rand()/RAND_MAX - 1.0); return (uint32_t)(len(a,d)*1000*r); }
int relOf(char m){ return strchr("SRBL",m)-"SRBL"; }

int run(int maze, unsigned seed){
  srand(seed);
  maze_reset();
  int cur = startN, h = 0, moves = 0;
  while (moves++ < 400) {
    int prev = cur; uint32_t seg = segOf(cur,h); cur = T[cur].nb[h];
    if (cur < 0) { printf("BUG: drove into nothing from %d dir %d\n", prev, h); return 0; }
    int back = (h+2)%4; bool circle = (cur==startN || cur==goalN);
    char m;
    if (!circle && deg(cur)==1) {                 // dead end: robot U-turns, then tells the map
      maze_arrive(0,false,seg); m = maze_next_move();
      if (m!='B' && m!='D') { printf("BUG: dead end gave %c\n", m); return 0; }
    } else {
      uint8_t rel=0;
      if (!circle) for(int d=0;d<4;d++) if(d!=back && T[cur].nb[d]>=0) rel |= 1<<((d-h+4)%4);
      maze_arrive(rel,circle,seg); m = maze_next_move();
    }
    if (m=='D') break;
    h = (h + relOf(m)) % 4;
    if (T[cur].nb[h] < 0) { printf("BUG: map says %c at true node %d but no line there\n", m, cur); return 0; }
  }
  if (!maze_goal_found()) { printf("FAIL: no goal\n"); return 0; }
  if (!maze_compute_best()) { printf("FAIL: no best\n"); return 0; }
  const char *best = maze_best();
  // replay the solve run on the true maze
  cur = startN; h = 0; int step = 0, dist = 0;
  for (int guard=0; guard<100; guard++) {
    dist += len(cur,h); cur = T[cur].nb[h];
    if (cur==goalN) break;
    int back=(h+2)%4; bool side=false;
    for(int d=0;d<4;d++) if(d!=back && d!=h && T[cur].nb[d]>=0) side=true;
    if (!side) continue;                          // straight-through, not detected
    if (step >= (int)strlen(best)) { printf("FAIL: path too short\n"); return 0; }
    h = (h + relOf(best[step++])) % 4;
    if (T[cur].nb[h] < 0) { printf("FAIL: solve drove into nothing\n"); return 0; }
  }
  printf("maze %d seed %u: explore moves=%d raw=%s best=%s solve distance=%d %s\n",
         maze, seed, moves, maze_raw(), best, dist, cur==goalN ? "REACHED END" : "MISSED END");
  return cur==goalN ? dist : 0;
}

void maze1(void){ // loops + dead ends, shortest S-A-D-H-I = 4
  TN=0; startN=at(0,0);
  edge(0,0,0,1); edge(0,1,0,2); edge(0,1,1,1); edge(0,2,1,2); edge(0,2,-1,2);
  edge(1,1,1,2); edge(1,1,2,1); edge(1,2,1,3); edge(1,3,2,3); edge(1,3,1,4);
  edge(2,3,2,1); edge(2,1,3,1); goalN=at(3,1);
}
void maze2(void){ // outer loop + inner loop, goal off the outer loop, shortest = 7
  TN=0; startN=at(0,0);
  edge(0,0,0,1);
  edge(0,1,-2,1); edge(-2,1,-2,4); edge(-2,4,-2,5); edge(-2,4,1,4); edge(1,4,2,4);
  edge(2,4,2,2); edge(2,2,2,1); edge(2,1,0,1);
  edge(0,1,0,2); edge(0,2,1,2); edge(1,2,2,2); edge(1,2,1,3); edge(1,3,1,4);
  edge(1,3,0,3); edge(0,3,0,2);
  goalN=at(-2,5);
}
void maze3(void){ // tree (no loops) with several dead ends, shortest = 6
  TN=0; startN=at(0,0);
  edge(0,0,0,2); edge(0,2,-1,2); edge(0,2,0,3); edge(0,2,2,2); edge(2,2,2,1); edge(2,2,2,4);
  edge(0,3,1,3); edge(0,3,0,4); edge(0,4,-2,4); edge(-2,4,-2,5); edge(0,4,0,5);
  goalN=at(-2,5);
}
int main(void){
  maze2(); verbose=1; run(2,1);
  verbose = 0;
  int pass=0, tot=0;
  int want[4]={0,4,7,7};
  for (int m=1;m<=3;m++) for (unsigned s=1;s<=100;s++){ if(m==1) maze1(); else if(m==2) maze2(); else maze3(); tot++; noise = SIM_NOISE; int d=run(m,s); if (d==want[m]) pass++; else printf("  ^ maze %d seed %u wrong\n", m, s); }
  printf("\nshortest path found in %d/%d noisy runs\n", pass, tot);
  return 0;
}
