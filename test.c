
#include "simpletools.h"
#include "abdrive.h"
#include "ping.h"
#include "math.h"

const int squareSize = 400; //size of the squares in the maze
const float tickLength = 3.25;
/**
 * positions are represented by a 4 bit integer
 * for example 0101 means that there are obstacles north and south
 */
const int north =1; //0001
const int east =2; //0010
const int south =4; //0100
const int west =8; //1000

unsigned int matrix [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int discovered [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int direction = 1;
int irLeft, irRight;  //current left and right distances

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
  * gets the infrared sensor values
  */
 void getIR()
 {
   irLeft = 0;
   irRight = 0;
   for (int dacVal = 0; dacVal < 160; dacVal += 8) {
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
    int ticks = squareSize / tickLength;
    drive_goto(ticks,ticks);
  }

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
  if(ping_cm(8)<20)
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

int isAdjacent(int from, int to)
{
  if(abs(from-to)==4||abs(from-to)==1)
  {return 1;}
  return 0;
}

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

//extract as much as possible into methods

/**
 * DepthFirstSearch that is used to build a map for the graph
 */
void depthFirstSearch()
{
  int currentpath [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int counter =0;
  driveSquare();
  analyze(0);
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
    else
    {
      while(isAdjacent(current,next)!=1)
      {
        gotoAdjacent(current,currentpath[counter]);
        current = currentpath[counter];
        counter --;
      }
      gotoAdjacent(current,next);
    }
    analyze(next);
    discovered[next]=1;
    pushNextToStack(next);
    counter ++;
    currentpath[counter]=current;
    current = next;
  }

}


/**
 * centers the robot in the first square on the simulator
 */
void center()
{
  int ticks = squareSize/3.75/tickLength;
  drive_goto(ticks, ticks);
}

unsigned int testMatrix[16] = {13,4,4,6,12,2,10,10,10,9,2,11,9,5,1,7};
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
    if(checkEast(testMatrix[minNode])==0)
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
    if(checkNorth(testMatrix[minNode])==0)
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
    if(checkWest(testMatrix[minNode])==0)
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
    if(checkSouth(testMatrix[minNode])==0)
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
//   center();
//   depthFirstSearch();
   Dijkstra();
   showDijkstra();
}
