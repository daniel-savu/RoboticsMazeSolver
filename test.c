
#include "simpletools.h"
#include "abdrive.h"
#include "ping.h"
#include "math.h"

const int squareSize = 365; //size of the squares in the maze
const float tickLength = 3.25;
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
unsigned int matrix [16] = {8,5,5,6,9,6,12,3,12,1,1,4,9,5,7,11};
//unsigned int matrix [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int discovered [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //0-not discovered 1-discovered
int direction = 1; //facing direction of the robot; 1-north 2-east 4-south 8-west
int irLeft, irRight;  //Infrared distances left and right
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
int stackContains(int data)
{
    for(int i=0;i<=top;i++)
    {
        if(stack[i]==data)
        {
            return 1;
        }
    }
    return 0;
}
/**
 * STACK IMPLEMETATION
 */

void calculateIR(int * irLeft, int * irRight, int * irLeftOld, int * irRightOld){
    *irLeftOld = *irLeft;
    *irRightOld = *irRight;
    *irLeft=*irRight=0;
    for(int dacVal = 0; dacVal < 160; dacVal += 8) {
        dac_ctr(26, 0, dacVal);
        freqout(11, 1, 38000);
        *irLeft += input(10);
        
        dac_ctr(27, 1, dacVal);
        freqout(1, 1, 38000);
        *irRight += input(2);
    }
}

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
void driveSquare(){
    int dist = squareSize/tickLength;
    int left, right, leftStart, rightStart;
    int irLeft, irRight, irLeftOld, irRightOld;
    
    drive_getTicks(&leftStart, &rightStart);
    calculateIR(&irLeft, &irRight, &irLeftOld, &irRightOld);
    left = leftStart, right = rightStart;
    drive_setRampStep(6);
    
    while(left < leftStart+dist || right < rightStart+dist){
        drive_getTicks(&left, &right);
        
        calculateIR(&irLeft, &irRight, &irLeftOld, &irRightOld);
        
        int dl = irLeft-irLeftOld; if(dl>3||dl<-3) dl=0;
        int dr = irRight-irRightOld; if(dr>3||dr<-3) dr=0;
        
        int correcterLeft = (irRight-irLeft)*1; correcterLeft=0;
        int correcterRight = (irLeft-irRight)*1; correcterRight=0;
        if(left-leftStart>dist/3 || right-rightStart>dist/3)
        correcterLeft=correcterRight=0;
        correcterLeft += (dr - dl)*3;
        correcterRight += (dl - dr)*3;
        if(irLeft<6 && left-leftStart<dist*3/4) {correcterLeft+=8; correcterRight+=-8;}
        if(irRight<6 && left-leftStart<dist*3/4) {correcterLeft+=-8; correcterRight+=8;}
        if(irRight==20&&irLeft>12) {correcterRight+=1; correcterLeft-=1;}
        if(irLeft==20&&irRight>12) {correcterLeft+=1; correcterRight-=1;}
        correcterLeft *= 1.5;
        correcterRight *= 1.6;
        {correcterLeft/=2; correcterRight/=2;}
        drive_rampStep(32*1.5+correcterLeft, 32*1.5+correcterRight);
        pause(5);
    }
    drive_ramp(0,0);
    drive_setRampStep(4);
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
    if(ping_cm(8)<30)
    {
        matrix[squareID] = matrix[squareID]|direction;
    }
    printf("%d \n",matrix[squareID] );
}

void turnInPlace(double angle){
    float wheelDistance = 32.8538f;
    if(angle>PI) { turnInPlace(angle-2*PI); return; }
    if(angle<-PI) { turnInPlace(angle+2*PI); return; }
    double dist = wheelDistance/2*angle;
    int ticks = (int)(dist);
    drive_goto(0,0);
    drive_goto(ticks, -ticks);
    drive_goto(0,0);
}

void turnLeft()
{
    turnInPlace(-PI/2);
}
void turnRight()
{
    turnInPlace(PI/2);
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

int isAccessible(int from, int to)
{
    int difference = to-from;
    if(difference==4)
    {
        if(checkNorth(matrix[from])==0)
        {
            return 1;
        }
    }
    if(difference==1)
    {
        if(checkEast(matrix[from])==0)
        {
            return 1;
        }
    }
    if(difference==-4)
    {
        if(checkSouth(matrix[from])==0)
        {
            return 1;
        }
    }
    if(difference==-1)
    {
        if(checkWest(matrix[from])==0)
        {
            return 1;
        }
    }
    return 0;
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
    if(checkNorth(matrix[index])==0&&discovered[index+4]==0&&(!stackContains(index+4)))
    {
        stackPush(index+4);
    }
    if(checkEast(matrix[index])==0&&discovered[index+1]==0&&(!stackContains(index+1)))
    {
        stackPush(index+1);
    }
    if(checkSouth(matrix[index])==0&&discovered[index-4]==0&&(!stackContains(index-4)))
    {
        stackPush(index-4);
    }
    if(checkWest(matrix[index])==0&&discovered[index-1]==0&&(!stackContains(index-1)))
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
        printf("Current %d Next %d \n",current, next );
        if(isAdjacent(current,next)==1&&isAccessible(current,next)) //next is adjacent to current
        {
            gotoAdjacent(current,next);
        }
        else                            //next is not adjacent to current
        {
            while((isAdjacent(current,next)!=1)||isAccessible(current,next)==0)//Go back the current path until it becomes adjacent
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
    direction=1;
}


/**
 * centers the robot in the first square on the simulator
 */
void center()
{
    int ticks = squareSize/3.75/tickLength;
    drive_goto(ticks, ticks);
}


//Dijkstra
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
//Prints the path produced by dijkstra and adds it to an array
unsigned int path [18];
int length =0;

void showDijkstra()
{
    int x=0;
    while(x!=16)
    {
        printf("%d \n", x);
        path[length]=x;
        x = parent[x];
        length++;
    }
}

void exitMaze()
{
    driveSquare();
    int from = path[0];
    for(int i=1;i<length;i++)
    {
        printf("Goto from %d to %d \n",from, path[i] );
        gotoAdjacent(from,path[i]);
        from = path[i];
    }
}

void exitFaster()
{
    int i;
   // drive_goto(squareSize/tickLength,squareSize/tickLength);
    int newPath[17];
    for(i=1;i<=16;i++)
        newPath[i] = path[i-1];
    newPath[0]=-4;
    length++;
    for(i=0;i<length;i++)
    path[i] = newPath[i];
    for(i = 1; i < length - 2 ;i++)
    {
        int k=1;
        while( abs(path[i-1] - path[i]) == abs(path[i] - path[i+1]) )
        {
            k++;
            i++;
        }
        printf("k: %d i: %d \n",k,i);
        drive_goto(squareSize*k/tickLength+7,squareSize*k/tickLength+7);
        if(path[i+1]==path[i]+1)
        {
            faceEast();
        }
        if(path[i+1]==path[i]+4)
        {
            faceNorth();
        }
        if(path[i+1]==path[i]-1)
        {
            faceWest();
        }
        if(path[i+1]==path[i]-4)
        {
            faceSouth();
        }

    }
    i--;
     printf("i: %d \n",i);
    while(i<length-1)
    {
        if(path[i+1]==path[i]+1)
        {
            faceEast();
        }
        if(path[i+1]==path[i]+4)
        {
            faceNorth();
        }
        if(path[i+1]==path[i]-1)
        {
            faceWest();
        }
        if(path[i+1]==path[i]-4)
        {
            faceSouth();
        }
        drive_goto(squareSize/tickLength+7,squareSize/tickLength+7);
    }
    
}


//matrix in beginning is changed for testing. Revert it to 0,0,0... and uncomment DFS below
int main() {
    center();
    //depthFirstSearch();
    Dijkstra();
    showDijkstra();
    
    exitFaster();
}
