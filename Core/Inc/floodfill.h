
#pragma once

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <stdlib.h>

uint16_t calb_ir = 410;

int mouse_x=0; //calculation
int mouse_y=0;  //calculation
int place1=0; //placeholder
int place2=0; //placeholder
int dir=0;  //calculation
int turn=0;  //calculation
int target=0; //constant
int MAZE_SIZE=16; //constant
int distance[4]; //outside data
int num=14; //mouse distance
int z=0;  //condition for queue
int min=0;  //target square distance
int max=0;

bool flags[3] = {0};

void markWall(int x, int y, int dir, bool hasWall);
int floodfill();
void enqueue(int element1, int element2);
void dequeue();
void trackPath() ;
int min4(int a, int b, int c, int d);
bool hasKnownWall(int x, int y, int a);
void checkAdjacentSquares(int x, int y) ;
void flood();
void checkWalls(int x, int y);
void initWall() ;
void markWall(int x, int y, int dir, bool hasWall);

extern uint16_t ir_data[];
extern uint8_t algo;

unsigned char maze[16][16] ={
        {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14},
        {13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
        {12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
        {11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
        {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
        {9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
        {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
        {7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
        {7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
        {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
        {9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
        {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
        {11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
        {12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
        {13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
        {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14}
    };
unsigned char check[16][16];
char walls[16][16];

int queuex[5000];
int queuey[5000];
int front = -1;
int rear = -1;

int count=0;



int floodfill() {

//	 float dist_calibration_turn = .2;
//	 float dist_calibration_straight = .05;
	 float dist_calibration_turn = .3;
	 float dist_calibration_straight = -.70;
	 float nindydegrees = (M_PI * WHEEL_BASE / 4 )+dist_calibration_turn;  // For 90 degrees turn
	 float oneCell = 18.0 + dist_calibration_straight;
	 int once = 0;

    initWall(); //setting up the borders and the right wall on the starting square

    go_distance(4.5,4.5, 1);
    while (maze[mouse_x][mouse_y]!=0) { //until the distance equals zero
        if (!check[mouse_x][mouse_y]){ //updates walls if it is a new square
                 checkWalls(mouse_x,mouse_y);
               }

               checkAdjacentSquares(mouse_x,mouse_y); //finds the shortest distance from adjacent squares


      if ((min<num) || once == 5 ){ //only move if the distance of nearby squares are less than the current one

        if ((turn-dir)==1){
        	go_distance(nindydegrees, -nindydegrees, 0);
        }
        else if((turn-dir)==-1){
        	go_distance(-nindydegrees, nindydegrees, 0);
        }
        else if((turn-dir)==3){
        	go_distance(-nindydegrees, nindydegrees, 0);
        }
        else if((turn-dir)==-3){
        	go_distance(nindydegrees, -nindydegrees, 0);
        }
        else if((turn-dir)==2){
        	if((ir_data[0] >calb_ir )&&(ir_data[1] >calb_ir )&&(ir_data[2] > calb_ir )){
        	go_distance(nindydegrees, -nindydegrees, 0);
        	go_distance(nindydegrees, -nindydegrees, 0);
            go_distance(-10,-9, 1);
            go_distance(4.5,4.5, 1);
            }else{
            	go_distance(nindydegrees, -nindydegrees, 0);
            	go_distance(nindydegrees, -nindydegrees, 0);
            }
        }
        else if((turn-dir)==-2){


            if((ir_data[0] > calb_ir )&&(ir_data[1] > calb_ir )&&(ir_data[2] > calb_ir)){
            go_distance(nindydegrees, -nindydegrees, 0);
            go_distance(nindydegrees, -nindydegrees, 0);
            go_distance(-10,-9, 1);
            go_distance(4.5,4.5, 1);
            }else {
            	go_distance(nindydegrees, -nindydegrees, 0);
                go_distance(nindydegrees, -nindydegrees, 0);
            }
        }




        num=min;

        dir=turn;

        go_distance(oneCell,oneCell, 1);

        trackPath(); //updates mouse coordinates
        once = 0;
      }
      else{
        flood(); //update square distances
        num=maze[mouse_x][mouse_y];
        once += 1;
    }


    }
    return 0;
}


void enqueue(int element1, int element2) {
    z=0;
    if (rear == 2000 - 1) {
        //printf("Queue is full");
        return ;
    }
    if (front == -1) {
        front = 0;
    }
    rear++;
    queuex[rear] = element1;
    queuey[rear] = element2;
    return;
}

void dequeue() {
    if (front == -1) {
        //printf("Queue is empty");
        z=-1;
        return -1;  // Return a default value when the queue is empty
    }
    else{
      place1 = queuex[front];
      place2 = queuey[front];
      front++;
      if (front>rear){
        front=-1;
        rear=-1;
      }
      return 0;  // Return a value to cover all control paths
    }
}


void trackPath() { //updates mouse position
  if (dir==0){
    mouse_y++;
  }
  else if (dir==1){
    mouse_x++;
  }
  else if (dir==2){
    mouse_y--;
  }
  else if (dir==3){
    mouse_x--;
  }
}

int min4(int a, int b, int c, int d) {
    int minVal = a;
    int turnDir = 0; // Assume initial direction is 0

    if (b < minVal) {
        minVal = b;
        turnDir = 1;
    }
    if (c < minVal) {
        minVal = c;
        turnDir = 2;
    }
    if (d < minVal) {
        minVal = d;
        turnDir = 3;
    }

    turn = turnDir;
    return minVal;
}

/*int manhattanDist(int x1, int y1, int x2, int y2) {
  return abs(x1-x2) + abs(y1-y2);
}*/

bool hasKnownWall(int x, int y, int a) {
  return walls[x][y] & (1 << a);
}

void checkAdjacentSquares(int x, int y) {
  int x1;
  int y1;

  if (y + 1 < MAZE_SIZE) {
    y1=y+1;
    if (hasKnownWall(mouse_x,mouse_y,0)){ //if wall is in that direction
      distance[0]=256;
    }
    else{ //distance is taken if within boundaries and there is no wall inbetween
      distance[0]=maze[x][y1];
    }
  }
  else{ // if square is outside boundaries
    distance[0]=256;
  }

  if (x + 1 < MAZE_SIZE) {
    x1=x+1;
    if (hasKnownWall(x,y,1)){
      distance[1]=256;
    }
    else{
      distance[1]=maze[x1][y];
    }
  }
  else{
    distance[1]=256;
  }

  if (y - 1 >= 0) {
    y1=y-1;
    if (hasKnownWall(x,y,2)){
      distance[2] =256;
    }
    else{
      distance[2]=maze[x][y1];
    }
  }
  else{
    distance[2]=256;
  }

  if (x - 1 >= 0) {
    x1=x-1;
    if (hasKnownWall(x,y,3)){
      distance[3]=256;

    }
    else{
      distance[3]=maze[x1][y];
    }
  }
  else{
    distance[3]=256;
  }

  min=min4(distance[0], distance[1], distance[2], distance[3]);
}


void flood(){
    enqueue(mouse_x, mouse_y);
    //printf("%d %d", place1, place2);
    while (z != -1) {
        dequeue();
        checkAdjacentSquares(place1, place2);
        if (min >= maze[place1][place2] && min!=0 && min<50) { //only enqueue if smaller than min
            maze[place1][place2] = min + 1;
            if (place1 - 1 >= 0 && !hasKnownWall(place1, place2, 3) && check[place1][place2]) {
                enqueue(place1 - 1, place2);
            }
            if (place2 - 1 >= 0 && !hasKnownWall(place1, place2, 2) && check[place1][place2]) {
                enqueue(place1, place2 - 1);
            }
            if (algo == 1){
            if (place2 + 1 < MAZE_SIZE && !hasKnownWall(place1, place2, 0) && check[place1][place2]) { //only enqueue if it is known and has no wall in between
                enqueue(place1, place2 + 1);
            }}else if (algo ==2){
            if (place1 + 1 < MAZE_SIZE && !hasKnownWall(place1, place2, 1) && check[place1][place2]) {
                enqueue(place1 + 1, place2);
            }else {
                if (place2 + 1 < MAZE_SIZE && !hasKnownWall(place1, place2, 0) && check[place1][place2]) { //only enqueue if it is known and has no wall in between
                    enqueue(place1, place2 + 1);
            }

            }
        }
    }
}
}

void checkWalls(int x, int y) { //checks front, left, and right walls; can be modify depending on number of sensors
  check[x][y]=1;
  int a=dir+1;
  int b=dir-1;

  if (a>3){
    a=0;
  }
  else if (b<0){
    b=3;
  }

  if (ir_data[1] > calb_ir){
    markWall(x, y, dir, true);
  }
  if (ir_data[2] > calb_ir){
    markWall(x, y, a, true);
  }
  if (ir_data[0] > calb_ir){
    markWall(x, y, b, true);
  }
}

void initWall() { //creating wall borders
  for (int x = 0; x < MAZE_SIZE; x++) {
    markWall(x, MAZE_SIZE - 1, 0, true); //North walls
    markWall(x, 0, 2, true); //South Walls
  }
  for (int y = 0; y < MAZE_SIZE; y++) {
    markWall(0, y, 3, true); //West Walls
    markWall(MAZE_SIZE - 1, y, 1, true); //East Walls
  }
  markWall(0, 0, 1, true);
}

void markWall(int x, int y, int dir, bool hasWall) {
  walls[x][y] |= (1 << dir);
  switch (dir) {
  case 0: //north
    if (y < 16 - 1) {
      walls[x][y] |= (1 << 0); //binary shifting, or-ing with 0001
    }
    break;
  case 2: //south
    if (y > 0) {
      walls[x][y] |= (1 << 2); //or-ing with 0100
    }
    break;
  case 3: //west
    if (x > 0) {
      walls[x][y] |= (1 << 3); //or-ing with 1000
    }
    break;
  case 1: //east
    if (x < 16 - 1) {
      walls[x][y] |= (1 << 1); //or-ing with 0010
    }
    break;
  }
}

