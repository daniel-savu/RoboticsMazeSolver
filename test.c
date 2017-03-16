
#include "simpletools.h"
#include "abdrive.h"
#include "ping.h"
#include "math.h"

const int squareSize = 400; //size of the squares in the maze
const float tickLength = 3.25;
const int speed = 31; // base speed
/**
 * At every index in the matrix there is an integer
 * the first four bits represent the four directions
 * for example 0101 means that there are obstacles north and south
 */
const int north =1; //0001
const int east =2; //0010
const int south =4; //0100
const int west =8; //1000

/*
|12|13|14|15|
|08|09|10|11|   Nodes indicies in the marix
|04|05|06|07|
|00|01|02|03|
 */
unsigned int matrix [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int discovered [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //0-not discovered 1-discovered
int direction = 1; //facing direction of the robot; 1-north 2-east 4-south 8-west
int irLeft, irRight;  //Infrared distances left and right
int initialLeft=0; //The robot calibrates in the beginning and uses the initial left and right distances as a reference
int initialRight=0;
int targetFront = 21;

/**
 * STACK IMPLEMETATION
 */
const int stackMAXSIZE = 16;
int top = -1;
unsigned int stack [16];
int stackIsEmpty()
{
  if(top == -1)
  {
    return 1;
  }
  return 0;
}
int stackIsFull()
{
  if(top==stackMAXSIZE-1)
  {
    return 1;
  }
  return 0;
}
int stackPeek()
{
  if(stackIsEmpty()==0)
  {
    return stack[top];
  }
  else
  {
    printf("Could not retrieve data, stack is empty\n");
  }
}
int stackPop()
{
  int data;
  if(stackIsEmpty()==0)
  {
    data = stack[top];
    top = top-1;
    return data;
  }
  printf("Could not retrieve data, stack is empty\n");
}
int stackPush(int data)
{
  printf("Pushing %d \n",data );
  if(stackIsFull()==0)
  {
    top = top+1;
    stack[top] = data;
  }
  else
  {
    printf("Stack is full, cannot insert data\n");
  }
}
/**
 * STACK IMPLEMETATION
 */

 /**
  * Tries to detect an obstacle at the maximum possible distance 20 times.
  */
 void getIR()
 {
   irLeft = 0;
   irRight = 0;
   for (int dacVal = 0; dacVal < 160; dacVal += 8) {
       dac_ctr(26, 0, 0);
       freqout(11, 1, 38000);
       irLeft += input(10);

       dac_ctr(27, 1, 0);
       freqout(1, 1, 38000);
       irRight += input(2);
   }
 }
/**
 * Uses the infrared sensor to detect obstacles on both sides and returns values
 * ranging from 0 to 20 based on how close they are.
 */
 void getIRNav()
 {
   irLeft = 0;
   irRight = 0;
   for(int dacVal = 0; dacVal < 160; dacVal += 8)
  {
    dac_ctr(26, 0, dacVal);
    freqout(11, 1, 38000);
    irLeft += input(10);
    dac_ctr(27, 1, dacVal);
    freqout(1, 1, 38000);
    irRight += input(2);
  }
 }

 int abs(int x)
 {
     if(x<0)
         return -x;
     return x;
 }

 void rotate180()
 {
     drive_goto(0,0);
     drive_goto(51,-51);
 }
/**
 * returns the number indicating what the direction is going to be
 * if you turn right
 */
 unsigned int directionRight(int current)
 {
   if(current == 8)
   {
     return 1;
   }
     return current*2;
 }

 unsigned int directionLeft(int current)
 {
   if(current == 1)
   {
     return 8;
   }
   return current/2;
 }

/**
 * Checks if the bit corresponding to north is 1.
 * If it is returns 1 because there must be an obstacle north
 */
 int checkNorth(int bits)
 {
   if ((bits&north)==north)
   {
     return 1;
   }
   return 0;
 }
 int checkEast(int bits)
 {
   if((bits&east)==east)
   {
     return 1;
   }
   return 0;
 }
 int checkSouth(int bits)
 {
   if((bits&south)==south)
   {
     return 1;
   }
   return 0;
 }
 int checkWest(int bits)
 {
   if((bits&west)==west)
   {
     return 1;
   }
   return 0;
 }

 /**
  * Moves one square ahead
  */
void driveSquare()
{
  //IF distance is more than initial on both sides but one is below 20 try to go closer to that wall
  int distance = squareSize/tickLength;
  int leftStart, rightStart, left, right;
  drive_getTicks(&leftStart,&rightStart);
  left = leftStart;
  right = rightStart;
  int tick = squareSize/tickLength;
  while(left<leftStart+tick|| right<rightStart+tick)
  {
    getIRNav();
    int leftAdjust = 0;
    int rightAdjust = 0;
    if(irLeft<initialLeft)
    {
      leftAdjust = (int)((double)(initialLeft-irLeft));
    }
    if(irRight<initialRight)
    {
      rightAdjust = (int)((double)(initialRight-irRight));
    }
    drive_getTicks(&left,&right);
    drive_speed(speed+leftAdjust,speed+rightAdjust);
  }
  drive_speed(0,0);
}

/**
 * Analyzes a square and updates the matrix if it detects obstacles
 */
void analyze(int squareID)
{
  getIR();
  if(irLeft<20)
  {
    matrix[squareID] = matrix[squareID]|directionLeft(direction);
  }
  if(irRight<20)
  {
    matrix[squareID] = matrix[squareID]|directionRight(direction);
  }
  if(ping_cm(8)<25)
  {
    matrix[squareID] = matrix[squareID]|direction;
  }
  printf("%d \n",matrix[squareID] );
}

void turnLeft()
{
  drive_goto(-25,26);
}
void turnRight()
{
  drive_goto(26,-25);
}
/**
 * Turns the robot so that it faces north
 */
void faceNorth()
{
  if(direction==2)
  {
    turnLeft();
  }
  if(direction==4)
  {
    rotate180();
  }
  if(direction==8)
  {
    turnRight();
  }
  direction=1;
}
void faceEast()
{
  if(direction==1)
  {
    turnRight();
  }
  if(direction==4)
  {
    turnLeft();
  }
  if(direction==8)
  {
    rotate180();
  }
  direction=2;
}
void faceSouth()
{
  if(direction==1)
  {
    rotate180();
  }
  if(direction==2)
  {
    turnRight();
  }
  if(direction==8)
  {
    turnLeft();
  }
  direction=4;
}
void faceWest()
{
  if(direction==1)
  {
    turnLeft();
  }
  if(direction==2)
  {
    rotate180();
  }
  if(direction==4)
  {
    turnRight();
  }
  direction=8;
}
/**
 * Goes to an adjacent square.
 */
void gotoAdjacent(int from, int to)
{
  if(to==from+1)
  {
    faceEast();
    driveSquare();
  }
  if(to==from+4)
  {
    faceNorth();
    driveSquare();
  }
  if(to==from-1)
  {
    faceWest();
    driveSquare();
  }
  if(to==from-4)
  {
    faceSouth();
    driveSquare();
  }
}
/**
 * Checks if the two squares are adjacent(the difference in their indecies)
 * has to be 1 or 4
 */
int isAdjacent(int from, int to)
{
  if(abs(from-to)==4||abs(from-to)==1)
  {return 1;}
  return 0;
}
/**
 * Checks if there are any accessible neighbours of the square
 * and if they are not visited, pushes them to the stack.
 */
void pushNextToStack(int index)
{
  if(checkNorth(matrix[index])==0&&discovered[index+4]==0)
  {
    stackPush(index+4);
  }
  if(checkEast(matrix[index])==0&&discovered[index+1]==0)
  {
    stackPush(index+1);
  }
  if(checkSouth(matrix[index])==0&&discovered[index-4]==0)
  {
    stackPush(index-4);
  }
  if(checkWest(matrix[index])==0&&discovered[index-1]==0)
  {
    stackPush(index-1);
  }
}


/**
 * DepthFirstSearch that is used to build a map for the graph
 * and then return to the initial position.
 */
void depthFirstSearch()
{
  int counter = 0;
  int currentpath [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int found =0;
  driveSquare(); //go to Square 0
  analyze(0);
  found++;
  if(checkNorth(matrix[0])==0)
  {
    stackPush(0+4);
  }
  if(checkEast(matrix[0])==0)
  {
    stackPush(0+1);
  }
  discovered[0]=1;
  int current = 0;
  int next;
  while(!stackIsEmpty())
  {
    next = stackPop();
    if(isAdjacent(current,next)==1) //next is adjacent to current
    {
      gotoAdjacent(current,next);
    }
    else                            //next is not adjacent to current
    {
      while(isAdjacent(current,next)!=1)//Go back the current path until it becomes adjacent
      {
        gotoAdjacent(current,currentpath[counter]);
        current = currentpath[counter];
        counter --;
      }
      gotoAdjacent(current,next);
    }
    analyze(next);
    discovered[next]=1;
    found ++;
    printf("Found: %d \n",found );
    pushNextToStack(next);
    counter ++;
    currentpath[counter]=current;
    current = next;
    if(found==16) //If 16 squares are discovered, stop.
    {
      break;
    }
  }
  while(counter>=0)//Go back the current path to the beginning.
  {
    gotoAdjacent(current,currentpath[counter]);
    current = currentpath[counter];
    counter--;
  }
  faceSouth(); //Face south and go to the initial square
  driveSquare();
  while(ping_cm(8)>targetFront)
  {
    drive_speed(10,10);
  }
  drive_speed(0,0);
  rotate180();
}


/**
 * centers the robot in the first square on the simulator
 */
void center()
{
  int ticks = squareSize/3.75/tickLength;
  drive_goto(ticks, ticks);
}

/**
 * Calibrates the robot
 */
void calibrate()
{
  getIRNav();
  initialLeft = irLeft;
  initialRight = irRight;
}

//Dijkstra
//unsigned int testMatrix[16] = {13,4,4,6,12,2,10,10,10,9,2,11,9,5,1,7};
unsigned int distance[16];
int turns[15];
int visited[16];
int parent[16];

void Dijkstra()
{
  int inf = 30000,min,minNode;
  parent[15]=16; //starting node is 15
  for(int i=0;i<15;i++)
  {
    distance[i] = inf;
    turns[i] = inf;
  }
  for(int i=0;i<16;i++)
  {
    min=inf;
    for(int j=0;j<16;j++)
      if(distance[j] < min && !visited[j])
      {
        min=distance[j];
        minNode=j;
      }
    visited[minNode]=1;
    if(checkEast(matrix[minNode])==0)
    if((distance[minNode+1] > distance[minNode]+1) || (distance[minNode+1] == distance[minNode]+1 && turns[minNode+1]>turns[minNode]))
    {
      distance[minNode+1] = distance[minNode]+1;
      parent[minNode+1] = minNode;
      turns[minNode+1] = turns[minNode];
      if(minNode != 15)
      {
        if(abs(minNode-parent[minNode]) != 1)
          turns[minNode+1]++;
      }
    }
    if(checkNorth(matrix[minNode])==0)
    if((distance[minNode+4] > distance[minNode]+1)  || (distance[minNode+4] == distance[minNode]+1 && turns[minNode+4]>turns[minNode]))
    {
      distance[minNode+4] = distance[minNode]+1;
      parent[minNode+4] = minNode;
      turns[minNode+4] = turns[minNode];
      if(minNode != 15)
      {
        if(abs(minNode-parent[minNode]) != 4)
          turns[minNode+4]++;
      }
    }
    if(checkWest(matrix[minNode])==0)
    if((distance[minNode-1] > distance[minNode]+1) || (distance[minNode-1] == distance[minNode]+1 && turns[minNode-1]>turns[minNode]))
    {
      distance[minNode-1] = distance[minNode]+1;
      parent[minNode-1] = minNode;
      turns[minNode-1] = turns[minNode];
      if(minNode != 15)
      {
        if(abs(minNode-parent[minNode]) != 1)
          turns[minNode-1]++;
      }
    }
    if(checkSouth(matrix[minNode])==0)
    if((distance[minNode-4] > distance[minNode]+1)  || (distance[minNode-4] == distance[minNode]+1 && turns[minNode-4]>turns[minNode]))
    {
      distance[minNode-4] = distance[minNode]+1;
      parent[minNode-4] = minNode;
      turns[minNode-4] = turns[minNode];
      if(minNode != 15)
      {
        if(abs(minNode-parent[minNode]) != 4)
          turns[minNode-4]++;
      }
    }
  }
}
//Prints the path produced by dijkstra
void showDijkstra()
{
  int x=0;
  while(x!=16)
  {
    printf("%d \n", x);
    x = parent[x];
  }
}

 int main() {
   calibrate();
   center();
   depthFirstSearch();
   Dijkstra();
   showDijkstra();
}
