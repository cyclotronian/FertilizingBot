/********************************************************************************
 Written by: Anshu Aviral, Adith Sagar Vastrad, Shubham Mehrotra, Samyak Parakh
 
 AVR Studio Version 6

 Date: 3rd March 2014


 This code determines the shortest path to be followed by the fertilizing robot.

 
*/
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
#include <math.h> //included to support power function

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char sharp1,sharp2, distance, adc_reading;
unsigned int value1,value2;

int Input[4];					//for storing the input tuples
int a,b,c,i,k,j=0,f;			//a,b,c,k,i,j,f are all counters, used for traversing in different loops
int start;						//
int total_path[100];			//it stores the sequence of nodes that give the shortest path
int count=0,count1=0,count2=0;  //stores the count of number of values in total path for the three tuples 
int direction[100];				//it stores the sequence of directions, the bot needs to face to move to the next node in total_path
int turn[6];					//turn
int dir[3];
int d;
int path[4];
int min=100;					//
int weight[100];
int bot_rotate[100];
int perm[3],correct_order[3];

//A is the matrix which stores the nodes corresponding to the 18 units
int A[18][3] = {{16},{15},{14,1},{1,3},{4},{5},{16,18,20},{15,21},{14,22},{3,13,11},{4,10},{5,9,7},{20},{21},{22,23},{11,23},{10},{9}};
	
//B is the matrix which stores the shortest path between every two nodes (total nodes = 23)
int B[23][23][11] =
{{{1},{1,2},{1,2,3},{1,2,3,4},{1,2,3,4,5},{1,2,3,4,5,6},{1,2,3,4,5,6,7},{1,2,3,4,5,6,7,8},{1,2,13,12,11,10,9},{1,2,13,12,11,10},{1,2,13,12,11},{1,2,13,12},{1,2,13},{1,2,14},{1,2,14,15},{1,2,14,15,16},{1,2,14,15,16,17},{1,2,14,15,16,17,18},{1,2,14,15,16,17,18,19},{1,2,13,12,22,21,20},{1,2,13,12,22,21},{1,2,13,12,22},{1,2,13,12,23}},

{{2,1},{2},{2,3},{2,3,4},{2,3,4,5},{2,3,4,5,6},{2,3,4,5,6,7},{2,3,4,5,6,7,8},{2,13,12,11,10,9},{2,13,12,11,10},{2,13,12,11},{2,13,12},{2,13},{2,14},{2,14,15},{2,14,15,16},{2,14,15,16,17},{2,14,15,16,17,18},{2,14,15,16,17,18,19},{2,13,12,22,21,20},{2,13,12,22,21},{2,13,12,22},{2,13,12,23}},

{{3,2,1},{3,2},{3},{3,4},{3,4,5},{3,4,5,6},{3,4,5,6,7},{3,4,5,6,7,8},{3,4,5,6,7,8,9},{3,2,13,12,11,10},{3,2,13,12,11},{3,2,13,12},{3,2,13},{3,2,14},{3,2,14,15},{3,2,14,15,16},{3,2,14,15,16,17},{3,2,14,15,16,17,18},{3,2,14,15,16,17,18,19},{3,2,13,12,22,21,20},{3,2,13,12,22,21},{3,2,13,12,22},{3,2,13,12,23}},

{{4,3,2,1},{4,3,2},{4,3},{4},{4,5},{4,5,6},{4,5,6,7},{4,5,6,7,8},{4,5,6,7,8,9},{4,5,6,7,8,9,10},{4,3,2,13,12,11},{4,3,2,13,12},{4,3,2,13},{4,3,2,14},{4,3,2,14,15},{4,3,2,14,15,16},{4,3,2,14,15,16,17},{4,3,2,14,15,16,17,18},{4,3,2,14,15,16,17,18,19},{4,3,2,13,12,22,21,20},{4,3,2,13,12,22,21},{4,3,2,13,12,22},{4,3,2,13,12,23}},

{{5,4,3,2,1},{5,4,3,2},{5,4,3},{5,4},{5},{5,6},{5,6,7},{5,6,7,8},{5,6,7,8,9},{5,6,7,8,9,10},{5,6,7,8,9,10,11},{5,4,3,2,13,12},{5,4,3,2,13},{5,4,3,2,14},{5,4,3,2,14,15},{5,4,3,2,14,15,16},{5,4,3,2,14,15,16,17},{5,4,3,2,14,15,16,17,18},{5,4,3,2,14,15,16,17,18,19},{5,4,3,2,13,12,22,21,20},{5,4,3,2,13,12,22,21},{5,4,3,2,13,12,22},{5,4,3,2,13,12,23}},

{{6,5,4,3,2,1},{6,5,4,3,2},{6,5,4,3},{6,5,4},{6,5},{6},{6,7},{6,7,8},{6,7,8,9},{6,7,8,9,10},{6,7,8,9,10,11},{6,7,8,9,10,11,12},{6,5,4,3,2,13},{6,5,4,3,2,14},{6,5,4,3,2,14,15},{6,5,4,3,2,14,15,16},{6,5,4,3,2,14,15,16,17},{6,5,4,3,2,14,15,16,17,18},{6,5,4,3,2,14,15,16,17,18,19},{6,5,4,3,2,13,12,22,21,20},{6,5,4,3,2,13,12,22,21},{6,5,4,3,2,13,12,22},{6,5,4,3,2,13,12,23}},

{{7,6,5,4,3,2,1},{7,6,5,4,3,2},{7,6,5,4,3},{7,6,5,4},{7,6,5},{7,6},{7,0},{7,8},{7,8,9},{7,8,9,10},{7,8,9,10,11},{7,8,9,10,11,12},{7,8,9,10,11,12,13},{7,6,5,4,3,2,14},{7,6,5,4,3,2,14,15},{7,6,5,4,3,2,14,15,16},{7,6,5,4,3,2,14,15,16,17},{7,6,5,4,3,2,14,15,16,17,18},{7,8,9,10,11,12,22,21,20,19},{7,8,9,10,11,12,22,21,20},{7,8,9,10,11,12,22,21},{7,8,9,10,11,12,22},{7,8,9,10,11,12,23}},

{{8,7,6,5,4,3,2,1},{8,7,6,5,4,3,2},{8,7,6,5,4,3},{8,7,6,5,4},{8,7,6,5},{8,7,6},{8,7},{8,0},{8,9},{8,9,10},{8,9,10,11},{8,9,10,11,12},{8,9,10,11,12,13},{8,7,6,5,4,3,2,14},{8,7,6,5,4,3,2,14,15},{8,7,6,5,4,3,2,14,15,16},{8,7,6,5,4,3,2,14,15,16,17},{8,9,10,11,12,22,21,20,19,18},{8,9,10,11,12,22,21,20,19},{8,9,10,11,12,22,21,20},{8,9,10,11,12,22,21},{8,9,10,11,12,22},{8,9,10,11,12,23},},

{{9,10,11,12,13,2,1},{9,10,11,12,13,2},{9,10,11,12,13,2,3},{9,8,7,6,5,4},{9,8,7,6,5},{9,8,7,6},{9,8,7},{9,8},{9,0},{9,10},{9,10,11},{9,10,11,12},{9,10,11,12,13},{9,10,11,12,13,2,14},{9,10,11,12,13,2,14,15},{9,10,11,12,13,2,14,15,16},{9,10,11,12,13,2,14,15,16,17},{9,10,11,12,22,21,20,19,18},{9,10,11,12,22,21,20,19},{9,10,11,12,22,21,20},{9,10,11,12,22,21},{9,10,11,12,22},{9,10,11,12,23}},

{{10,11,12,13,2,1},{10,11,12,13,2},{10,11,12,13,2,3},{10,9,8,7,6,5,4},{10,9,8,7,6,5},{10,9,8,7,6},{10,9,8,7},{10,9,8},{10,9},{10},{10,11},{10,11,12},{10,11,12,13},{10,11,12,13,2,14},{10,11,12,13,2,14,15},{10,11,12,13,2,14,15,16},{10,11,12,13,2,14,15,16,17},{10,11,12,22,21,20,19,18},{10,11,12,22,21,20,19},{10,11,12,22,21,20},{10,11,12,22,21},{10,11,12,22},{10,11,12,23}},

{{11,12,13,2,1},{11,12,13,2},{11,12,13,2,3},{11,12,13,2,3,4},{11,10,9,8,7,6,5},{11,10,9,8,7,6},{11,10,9,8,7},{11,10,9,8},{11,10,9},{11,10},{11},{11,12},{11,12,13},{11,12,13,2,14},{11,12,13,2,14,15},{11,12,13,2,14,15,16},{11,12,13,2,14,15,16,17},{11,12,22,21,20,19,18},{11,12,22,21,20,19},{11,12,22,21,20},{11,12,22,21},{11,12,22},{11,12,23}},

{{12,13,2,1},{12,13,2},{12,13,2,3},{12,13,2,3,4},{12,13,2,3,4,5},{12,11,10,9,8,7,6},{12,11,10,9,8,7},{12,11,10,9,8},{12,11,10,9},{12,11,10},{12,11},{12,0},{12,13},{12,13,2,14},{12,13,2,14,15},{12,13,2,14,15,16},{12,13,2,14,15,16,17},{12,22,21,20,19,18},{12,22,21,20,19},{12,22,21,20},{12,22,21},{12,22},{12,23}},

{{13,2,1},{13,2},{13,2,3},{13,2,3,4},{13,2,3,4,5},{13,2,3,4,5,6},{13,12,11,10,9,8,7},{13,12,11,10,9,8},{13,12,11,10,9},{13,12,11,10},{13,12,11},{13,12},{13,0},{13,2,14},{13,2,14,15},{13,2,14,15,16},{13,2,14,15,16,17},{13,12,22,21,20,19,18},{13,12,22,21,20,19},{13,12,22,21,20},{13,12,22,21},{13,12,22},{13,12,23}},

{{14,2,1},{14,2},{14,2,3},{14,2,3,4},{14,2,3,4,5},{14,2,3,4,5,6},{14,2,3,4,5,6,7},{14,2,3,4,5,6,7,8},{14,2,13,12,11,10,9},{14,2,13,12,11,10},{14,2,13,12,11},{14,2,13,12},{14,2,13},{14},{14,15},{14,15,16},{14,15,16,17},{14,15,16,17,18},{14,15,16,17,18,19},{14,2,13,12,22,21,20},{14,2,13,12,22,21},{14,2,13,12,22},{14,2,13,12,23}},

{{15,14,2,1},{15,14,2},{15,14,2,3},{15,14,2,3,4},{15,14,2,3,4,5},{15,14,2,3,4,5,6},{15,14,2,3,4,5,6,7},{15,14,2,3,4,5,6,7,8},{15,14,2,13,12,11,10,9},{15,14,2,13,12,11,10},{15,14,2,13,12,11},{15,14,2,13,12},{15,14,2,13},{15,14},{15},{15,16},{15,16,17},{15,16,17,18},{15,16,17,18,19},{15,16,17,18,19,20},{15,14,2,13,12,22,21},{15,14,2,13,12,22},{15,14,2,13,12,23}},

{{16,15,14,2,1},{16,15,14,2},{16,15,14,2,3},{16,15,14,2,3,4},{16,15,14,2,3,4,5},{16,15,14,2,3,4,5,6},{16,15,14,2,3,4,5,6,7},{16,15,14,2,3,4,5,6,7,8},{16,15,14,2,13,12,11,10,9},{16,15,14,2,13,12,11,10},{16,15,14,2,13,12,11},{16,15,14,2,13,12},{16,15,14,2,13},{16,15,14},{16,15},{16},{16,17},{16,17,18},{16,17,18,19},{16,17,18,19,20},{16,15,14,2,13,12,22,21},{16,15,14,2,13,12,22},{16,15,14,2,13,12,23}},

{{17,16,15,14,2,1},{17,16,15,14,2},{17,16,15,14,2,3},{17,16,15,14,2,3,4},{17,16,15,14,2,3,4,5},{17,16,15,14,2,3,4,5,6},{17,16,15,14,2,3,4,5,6,7},{17,16,15,14,2,3,4,5,6,7,8},{17,16,15,14,2,13,12,11,10,9},{17,16,15,14,2,13,12,11,10},{17,16,15,14,2,13,12,11},{17,16,15,14,2,13,12},{17,16,15,14,2,13},{17,16,15,14},{17,16,15},{17,16},{17},{17,18},{17,18,19},{17,18,19,20},{17,18,19,20,21},{17,18,19,20,21,22},{17,18,19,20,21,22,12,23}},

{{18,17,16,15,14,2,1},{18,17,16,15,14,2},{18,17,16,15,14,2,3},{18,17,16,15,14,2,3,4},{18,17,16,15,14,2,3,4,5},{18,17,16,15,14,2,3,4,5,6},{18,17,16,15,14,2,3,4,5,6,7},{18,19,20,21,22,12,11,10,9,8},{18,19,20,21,22,12,11,10,9},{18,19,20,21,22,12,11,10},{18,19,20,21,22,12,11},{18,19,20,21,22,12},{18,19,20,21,22,12,13},{18,17,16,15,14},{18,17,16,15},{18,17,16},{18,17},{18},{18,19},{18,19,20},{18,19,20,21},{18,19,20,21,22},{18,19,20,21,22,12,23}},

{{19,18,17,16,15,14,2,1},{19,18,17,16,15,14,2},{19,18,17,16,15,14,2,3},{19,18,17,16,15,14,2,3,4},{19,18,17,16,15,14,2,3,4,5},{19,18,17,16,15,14,2,3,4,5,6},{19,20,21,22,12,11,10,9,8,7},{19,20,21,22,12,11,10,9,8},{19,20,21,22,12,11,10,9},{19,20,21,22,12,11,10},{19,20,21,22,12,11},{19,20,21,22,12},{19,20,21,22,12,13},{19,18,17,16,15,14},{19,18,17,16,15},{19,18,17,16},{19,18,17},{19,18},{19},{19,20},{19,20,21},{19,20,21,22},{19,20,21,22,12,23}},

{{20,21,22,12,13,2,1},{20,21,22,12,13,2},{20,21,22,12,13,2,3},{20,21,22,12,13,2,3,4},{20,21,22,12,13,2,3,4,5},{20,21,22,12,13,2,3,4,5,6},{20,21,22,12,11,10,9,8,7},{20,21,22,12,11,10,9,8},{20,21,22,12,11,10,9},{20,21,22,12,11,10},{20,21,22,12,11},{20,21,22,12},{20,21,22,12,13},{20,21,22,12,13,2,14},{20,19,18,17,16,15},{20,19,18,17,16},{20,19,18,17},{20,19,18},{20,19},{20},{20,21},{20,21,22},{20,21,22,12,23}},

{{21,22,12,13,2,1},{21,22,12,13,2},{21,22,12,13,2,3},{21,22,12,13,2,3,4},{21,22,12,13,2,3,4,5},{21,22,12,13,2,3,4,5,6},{21,22,12,11,10,9,8,7},{21,22,12,11,10,9,8},{21,22,12,11,10,9},{21,22,12,11,10},{21,22,12,11},{21,22,12},{21,22,12,13},{21,22,12,13,2,14},{20,19,18,17,16,15},{21,20,19,18,17,16},{21,20,19,18,17},{21,20,19,18},{21,20,19},{21,20},{21},{21,22},{21,22,12,23}},

{{22,12,13,2,1},{22,12,13,2},{22,12,13,2,3},{22,12,13,2,3,4},{22,12,13,2,3,4,5},{22,12,13,2,3,4,5,6},{22,12,11,10,9,8,7},{22,12,11,10,9,8},{22,12,11,10,9},{22,12,11,10},{22,12,11},{22,12},{22,12,13},{22,12,13,2,14},{22,21,20,19,18,17,16,15},{22,21,20,19,18,17,16},{22,21,20,19,18,17},{22,21,20,19,18},{22,21,20,19},{22,21,20},{22,21},{22},{22,12,23}},

{{23,12,13,2,1},{23,12,13,2},{23,12,13,2,3},{23,12,13,2,3,4},{23,12,13,2,3,4,5},{23,12,13,2,3,4,5,6},{23,12,13,2,3,4,5,6,7},{23,12,11,10,9,8},{23,12,11,10,9},{23,12,11,10},{23,12,11},{23,12},{23,12,13},{23,12,13,2,14},{23,12,13,2,14,15},{23,12,13,2,14,15,16},{23,12,13,2,14,15,16,17},{23,12,13,2,14,15,16,17,18},{23,12,13,2,14,15,16,17,18,19},{23,12,22,21,20},{23,12,22,21},{23,12,22},{23}}};

//C is the matrix which gives the distance between every two nodes (1 unit = distance between two consecutive node)
int C[23][23]=
{{0,1,2,3,4,5,6,7,6,5,4,3,2,2,3,4,5,6,7,6,5,4,4},
{1,0,1,2,3,4,5,6,5,4,3,2,1,1,2,3,4,5,6,5,4,3,3},
{2,1,0,1,2,3,4,5,6,5,4,3,2,2,3,4,5,6,7,6,5,4,4},
{3,2,1,0,1,2,3,4,5,6,5,4,3,3,4,5,6,7,8,7,6,5,5},
{4,3,2,1,0,1,2,3,4,5,6,5,4,4,5,6,7,8,9,8,7,6,6},
{5,4,3,2,1,0,1,2,3,4,5,6,5,5,6,7,8,9,10,9,8,7,7},
{6,5,4,3,2,1,0,1,2,3,4,5,6,6,7,8,9,10,9,8,7,6,6},
{7,6,5,4,3,2,1,0,1,2,3,4,5,7,8,9,10,9,8,7,6,5,5},
{6,5,6,5,4,3,2,1,0,1,2,3,4,6,7,8,9,8,7,6,5,4,4},
{5,4,5,6,5,4,3,2,1,0,1,2,3,5,6,7,8,7,6,5,4,3,3},
{4,3,4,5,6,5,4,3,2,1,0,1,2,4,5,6,7,6,5,4,3,2,2},
{3,2,3,4,5,6,5,4,3,2,1,0,1,3,4,5,6,5,4,3,2,1,1},
{2,1,2,3,4,5,6,5,4,3,2,1,0,2,3,4,5,6,5,4,3,2,2},
{2,1,2,3,4,5,6,7,6,5,4,3,2,0,1,2,3,4,5,6,5,4,4},
{3,2,3,4,5,6,7,8,7,6,5,4,3,1,0,1,2,3,4,5,6,5,5},
{4,3,4,5,6,7,8,9,8,7,6,5,4,2,1,0,1,2,3,4,5,6,6},
{5,4,5,6,7,8,9,10,9,8,7,6,5,3,2,1,0,1,2,3,4,5,7},
{6,5,6,7,8,9,10,9,8,7,6,5,6,4,3,2,1,0,1,2,3,4,6},
{7,6,7,8,9,10,9,8,7,6,5,4,5,5,4,3,2,1,0,1,2,3,5},
{6,5,6,7,8,9,8,7,6,5,4,3,4,6,5,4,3,2,1,0,1,2,4},
{5,4,5,6,7,8,7,6,5,4,3,2,3,5,6,5,4,3,2,1,0,1,2},
{4,3,4,5,6,7,6,5,4,3,2,1,2,4,5,6,5,4,3,2,1,0,2},
{4,3,4,5,6,7,6,5,4,3,2,1,2,4,5,6,7,6,5,4,3,2,0}};
	
//D is the matrix which gives the direction, bot needs to be facing for moving to the next node
int D[23][23]=
{{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{9,0,3,0,0,0,0,0,0,0,0,0,1,7,0,0,0,0,0,0,0,0,0},
{0,7,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,7,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,7,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,7,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,9,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,9,0,7,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,3,0,7,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,3,0,7,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,3,0,7,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,3,0,9,0,0,0,0,0,0,0,0,7,1},
{0,9,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
{0,3,0,0,0,0,0,0,0,0,0,0,0,0,7,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,7,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,7,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,1,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,1,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,3,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,0,3,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,0,3,0},
{0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,7,0,0},
{0,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0}};
	
//Returns turn required to move from current to next direction
int get_turn(int init, int final)	
{
	if(init==final)
		return 1;
	if((init*3)%10==final)
		return 2;
	if((init*9)%10==final)
		return 3;
	if((init*7)%10==final)
		return 4;
	else return 0;
}

int get(int i)
{
	if(i>=3)
		return (i-3);
	else 
		return i;
}

int get1(int i)
{
	if(i>=4)
		return (i-4);
	else
		return i;
}

//Function that cacomputes different permutations of the set of 3 tuples and determines the shortest path
void permutation3()
{
    for(i=0;i<3;i++)
				{
					weight[j] = C[start-1][perm[get(i)]-1] + C[perm[get(i)]-1][perm[get(i+1)]-1] + C[perm[get(i+1)]-1][perm[get(i+2)]-1]; //this computes the weight(length) of the path followed for a particular sequence

					if(weight[j]<min)
					{
						min = weight[j];                                                           // the min variable is updated as and when the new weight is lower than the current min value
						
						path[0] = perm[get(i)];        
						path[1] = perm[get(i+1)];                                                 //the path[] array is updated with different permutations of the input as the value of i varies
						path[2] = perm[get(i+2)];
						for(k=0;k<3;k++)
						{if(get(i)==k)
							{correct_order[0]=Input[get(k)];correct_order[1]=Input[get(k+1)];correct_order[2]=Input[get(k+2)];}}
																								  //the correct_order[] array stores the correct order of the tubels to be followed
					}
					
					j++;


					weight[j] = C[start-1][perm[get(i)]-1] + C[perm[get(i)]-1][perm[get(i+2)]-1] + C[perm[get(i+2)]-1][perm[get(i+1)]-1];

					if(weight[j]<min)															
					{
						min = weight[j];                
						path[0] = perm[get(i)];
						path[1] = perm[get(i+2)];                                                //the path[] array is updated with different permutations of the input as the value of i varies
						path[2] = perm[get(i+1)];

						for(k=0;k<3;k++)
						{if(get(i)==k)
							{correct_order[0]=Input[get(k)];correct_order[1]=Input[get(k+2)];correct_order[2]=Input[get(k+1)];}}

					}
					
				}
}


//Function that cacomputes different permutations of the set of 4 tuples and determines the shortest 	
void permutation4()																
{
    min=100;
    for(i=0;i<4;i++)
				{
					weight[j] = C[start-1][perm[get1(i)]-1] + C[perm[get1(i)]-1][perm[get1(i+1)]-1] + C[perm[get1(i+1)]-1][perm[get1(i+2)]-1] + C[perm[get1(i+2)]-1][perm[get1(i+3)]-1];

					if(weight[j]<min)
					{
						min = weight[j];
						path[0] = perm[get1(i)];
						path[1] = perm[get1(i+1)];                                               //the path[] array is updated with different permutations of the input as the value of i varies
						path[2] = perm[get1(i+2)];
						path[3] = perm[get1(i+3)];
						for(k=0;k<4;k++)
						{if(get(i)==k)
							{correct_order[0]=Input[get1(k)];correct_order[1]=Input[get1(k+1)];correct_order[2]=Input[get1(k+2)];correct_order[3]=Input[get1(k+3)];}}

					}
					
					j++;


					weight[j] = C[start-1][perm[get1(i)]-1] + C[perm[get1(i)]-1][perm[get1(i+2)]-1] + C[perm[get1(i+2)]-1][perm[get1(i+3)]-1] + C[perm[get1(i+3)]-1][perm[get1(i+1)]-1];

					if(weight[j]<min)
					{
						min = weight[j];
						path[0] = perm[get1(i)];
						path[1] = perm[get1(i+2)];
						path[2] = perm[get1(i+3)];                                              //the path[] array is updated with different permutations of the input as the value of i varies
						path[3] = perm[get1(i+1)];
						for(k=0;k<4;k++)
						{if(get1(i)==k)
							{correct_order[0]=Input[get1(k)];correct_order[1]=Input[get1(k+1)];correct_order[2]=Input[get1(k+2)];correct_order[3]=Input[get1(k+3)];}}

					}
					
					j++;



					weight[j] = C[start-1][perm[get1(i)]-1] + C[perm[get1(i)]-1][perm[get1(i+3)]-1] + C[perm[get1(i+3)]-1][perm[get1(i+2)]-1] + C[perm[get1(i+2)]-1][perm[get1(i+1)]-1];

					if(weight[j]<min)
					{
						min = weight[j];
						path[0] = perm[get1(i)];
						path[1] = perm[get1(i+3)];
						path[2] = perm[get1(i+2)];                                               //the path[] array is updated with different permutations of the input as the value of i varies
						path[3] = perm[get1(i+1)];
						for(k=0;k<4;k++)
						{if(get1(i)==k)
							{correct_order[0]=Input[get1(k)];correct_order[1]=Input[get1(k+1)];correct_order[2]=Input[get1(k+2)];correct_order[3]=Input[get1(k+3)];}}

					}
					
					j++;




					weight[j] = C[start-1][perm[get1(i)]-1] + C[perm[get1(i)]-1][perm[get1(i+1)]-1] + C[perm[get1(i+1)]-1][perm[get1(i+3)]-1] + C[perm[get1(i+3)]-1][perm[get1(i+2)]-1];

					if(weight[j]<min)
					{
						min = weight[j];
						path[0] = perm[get1(i)];
						path[1] = perm[get1(i+1)];
						path[2] = perm[get1(i+3)];                                             //the path[] array is updated with different permutations of the input as the value of i varies
						path[3] = perm[get1(i+2)];
						for(k=0;k<4;k++)
						{if(get1(i)==k)
							{correct_order[0]=Input[get1(k)];correct_order[1]=Input[get1(k+1)];correct_order[2]=Input[get1(k+2)];correct_order[3]=Input[get1(k+3)];}}

					}
					
					j++;


                    weight[j] = C[start-1][perm[get1(i)]-1] + C[perm[get1(i)]-1][perm[get1(i+2)]-1] + C[perm[get1(i+2)]-1][perm[get1(i+1)]-1] + C[perm[get1(i+1)]-1][perm[get1(i+3)]-1];
					if(weight[j]<min)
					{
						min = weight[j];
						path[0] = perm[get1(i)];
						path[1] = perm[get1(i+2)];                                           //the path[] array is updated with different permutations of the input as the value of i varies
						path[2] = perm[get1(i+1)];
						path[3] = perm[get1(i+3)];
						for(k=0;k<4;k++)
						{if(get1(i)==k)
							{correct_order[0]=Input[get1(k)];correct_order[1]=Input[get1(k+1)];correct_order[2]=Input[get1(k+2)];correct_order[3]=Input[get1(k+3)];}}

					}
					
					j++;



					weight[j] = C[start-1][perm[get1(i)]-1] + C[perm[get1(i)]-1][perm[get1(i+3)]-1] + C[perm[get1(i+3)]-1][perm[get1(i+1)]-1] + C[perm[get1(i+1)]-1][perm[get1(i+2)]-1];

					if(weight[j]<min)
					{
						min = weight[j];
						path[0] = perm[get1(i)];
						path[1] = perm[get1(i+3)];                                           //the path[] array is updated with different permutations of the input as the value of i varies
						path[2] = perm[get1(i+1)];
						path[3] = perm[get1(i+2)];
						for(k=0;k<4;k++)
						{if(get1(i)==k)
							{correct_order[0]=Input[get1(k)];correct_order[1]=Input[get1(k+1)];correct_order[2]=Input[get1(k+2)];correct_order[3]=Input[get1(k+3)];}}

					}
					
					j++;

				}
}
	
//function that calculates the shortest path for the given input	
void shortest_path()
{
	for(f=0;f<2;f++)	//two loops for the first two tuples of three unit each
	{
		if(f==0){
	start = 1;
	Input[0] = 1;
	Input[1] = 18;
	Input[2] = 9;
	}
	else{
		start = path[2];
		Input[0] = 6;
		Input[1] = 8;
		Input[2] = 17;
	}
	for(a=0;a<3;a++)																		//loop to get the minimum weight and the correct order of units 
	{
		for(b=0;b<3;b++)
		{
			for(c=0;c<3;c++)
			{
				if((A[Input[0]-1][a]!=0)&&(A[Input[1]-1][b]!=0)&&(A[Input[2]-1][c]!=0))		//if there is an element in the matrix A corresponding to given input
				{
					perm[0] = A[Input[0]-1][a];												
					perm[1] = A[Input[1]-1][b];
					perm[2] = A[Input[2]-1][c];
	
					permutation3();															//we get optimized path nodes in path[] array for 3 unit tuple

				}
			}
		}
	}
	
			for(c=0;c<11;c++)																//loop that adds node from matrix B to the total_path array
			{																				//between the start and first correct node 
				if(B[start-1][path[0]-1][c] == 0)
					break;
				total_path[count] = B[start-1][path[0]-1][c];
				count++;
			}

			for(c=0;c<11;c++)																//loop that adds node to the total_path array
			{																				//between the first and second correct node 
				if(B[path[0]-1][path[1]-1][c] == 0)
					break;
				total_path[count] = B[path[0]-1][path[1]-1][c];
				count++;
			}

			for(c=0;c<11;c++)																//loop that adds node to the total_path array
			{																				//between the second and third correct node 
				if(B[path[1]-1][path[2]-1][c] == 0)
					break;
				total_path[count] = B[path[1]-1][path[2]-1][c];
				count++;
			}

	for(a=count1;a<count;a++)																//adds element to direction array from matrix D
	{
		direction[a] = D[total_path[a]-1][total_path[a+1]-1];
	}

	i=0;j=0;

	for(a=count1;a<count;a++)																//Loop for adding element to dir[] array
	{																						//Basic logic is to add element only when 
		if(direction[a]==0)																	//the direction value is zero
		{																					
			direction[a] = direction[a-1];													//direction is set to previous value once it is found to be zero
			for(b=0;b<18;b++)
			{
				for(c=0;c<3;c++)
				{
					if(A[b][c]==total_path[a])
					{
						turn[i] = b+1;														
						if(i%2!=0)
						{
							dir[j] = correct_order[j]==turn[i]?(turn[i]>turn[i-1]?(direction[a-1]==3?2:1):(direction[a-1]==3?1:2)):(turn[i]>turn[i-1]?(direction[a-1]==3?1:2):(direction[a-1]==3?2:1));
							j++;
						}
						i++;
					}
				}
			}

		}
	}


	bot_rotate[0] = 1;																		//gives initial value to bot_rotate for first tuple using get_turn function
	bot_rotate[count1] = get_turn(direction[count1-1],direction[count1]);					//gives initial value to bot_rotate for second tuple using get_turn function
	for(a=count1;a<count;a++)
	{
		bot_rotate[a+1] = get_turn(direction[a],direction[a+1]);							//gives value to bot_rotate array
	}
	count1 = count;																			//count1 is initialized with a non zero value at the end of first tuple
}

	count2=count;																			//count2 is initialized with a non zero value at the end of second tuple
	start = path[2];																		//start is set to end node of second tuple
	Input[0] = 4;																			//input to the third tuple
	Input[1] = 13;
	Input[2] = 3;
	Input[3] = 7;

for(a=0;a<3;a++)																			//loop to get the minimum weight and the correct order of units
{
	for(b=0;b<3;b++)
	{
		for(c=0;c<3;c++)
		{
		    for(d=0;d<3;d++)
            {
                if((A[Input[0]-1][a]!=0)&&(A[Input[1]-1][b]!=0)&&(A[Input[2]-1][c]!=0)&&(A[Input[3]-1][d]!=0)) //if there is an element in the matrix A corresponding to given input
                {
                    perm[0] = A[Input[0]-1][a];
                    perm[1] = A[Input[1]-1][b];
                    perm[2] = A[Input[2]-1][c];
                    perm[3] = A[Input[3]-1][d];
					
                    permutation4();															//we get optimized path nodes in path[] array for four unit tuple
                }
			}
		}
	}
}


			for(c=0;c<11;c++)																//loop that adds node from matrix B to the total_path array
			{																				//between the start and first correct node
				if(B[start-1][path[0]-1][c] == 0)
					break;
				total_path[count] = B[start-1][path[0]-1][c];
				count++;
			}

			for(c=0;c<11;c++)																//loop that adds node from matrix B to the total_path array
			{																				//between the first and second correct node
				if(B[path[0]-1][path[1]-1][c] == 0)
					break;
				total_path[count] = B[path[0]-1][path[1]-1][c];
				count++;
			}

			for(c=0;c<11;c++)																//loop that adds node from matrix B to the total_path array
			{																				//between the second and third correct node
				if(B[path[1]-1][path[2]-1][c] == 0)
					break;
				total_path[count] = B[path[1]-1][path[2]-1][c];
				count++;
			}
			for(c=0;c<11;c++)																//loop that adds node from matrix B to the total_path array
			{																				//between the third and fourth correct node
				if(B[path[2]-1][path[3]-1][c] == 0)
					break;
				total_path[count] = B[path[2]-1][path[3]-1][c];
				count++;
			}

	for(a=count2;a<count;a++)																//adds element to direction array from matrix D
	{
		direction[a] = D[total_path[a]-1][total_path[a+1]-1];
	}

	i=0;j=0;

	for(a=count2;a<count;a++)																//Loop for adding element to dir[] array
	{																						//Logic is similar to the three unit tuple case 
		if(direction[a]==0)
		{
			direction[a] = direction[a-1];
			for(b=0;b<18;b++)
			{
				for(c=0;c<4;c++)
				{
					if(A[b][c]==total_path[a])
					{
						turn[i] = b+1;
						if(i%2!=0)
						{
							dir[j] = correct_order[j]==turn[i]?(turn[i]>turn[i-1]?(direction[a-1]==3?2:1):(direction[a-1]==3?1:2)):(turn[i]>turn[i-1]?(direction[a-1]==3?1:2):(direction[a-1]==3?2:1));
							j++;
						}
						i++;
					}
				}
			}

		}
	}

	bot_rotate[count2] = get_turn(direction[count2-1],direction[count2]);						//gives initial value to bot_rotate for third tuple using get_turn function
	for(a=count2;a<count;a++)
	{
		bot_rotate[a+1] = get_turn(direction[a],direction[a+1]);								//gives value to bot_rotate array
	}

}
//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}
//Function to initialize ports
void port_init()
{
	buzzer_pin_config();
	lcd_port_config();
	adc_pin_config();
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
	servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}
//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{//lcd_print(0,0,ShaftCountRight,6);
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	velocity(250,250);
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	velocity(255,255);
	right(); //Turn right
	angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	velocity(255,255);
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	velocity(255,255);
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

//Function to initialize all the devices
void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	adc_init();
	timer5_init();
	timer1_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
}

unsigned int mod(unsigned int x)
{
		if(x>255)
			x=255;
		return x;
}

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
} 


//Main Function

int main(void)
{	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	int pos=0,num=0,v;									//pos: counter for total_path, num: counter for dir[], v: counter for delay loop
	unsigned char i = 0;										//specifies angle for servo motor
	unsigned int x,y,in_count=0;								//x,y: left wheel and right wheel velocity, in_count: counter used in turning
	
	i=60;
	servo_3(i);
	_delay_ms(1000);											//initial angle set to 60 degrees
	
	shortest_path();
	//These are the output of the shortest_path function, presented here for reference
	//total_path[100] = {1, 2, 14, 15, 16, 16, 15, 14, 14, 2, 13, 12, 11, 10, 9, 9, 10, 10, 9, 8, 7, 6, 5, 5, 4, 3, 2, 14, 15, 15,14,2,1,1,1,2,13,12,22,21,20,20};
	//bot_rotate[100] = {1, 4, 1, 1, 1, 3, 1, 1, 1, 4, 1, 2, 1, 1, 1, 3, 1, 3, 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 3, 1, 2, 1, 1, 3, 1, 1, 4, 1, 1, 1, 1};
	//dir[20] = {2, 2, 2, 1, 2, 1, 2, 1, 2, 1};
		
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		sharp1 = ADC_Conversion(9);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp1"
		value1 = Sharp_GP2D12_estimation(sharp1);		//converts sharp value to mm units

		sharp2 = ADC_Conversion(13);					//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp2"
		value2 = Sharp_GP2D12_estimation(sharp2);		//converts sharp value to mm units
		
		if(((value2<250)||(value1<250))||((Center_white_line>60)&&((Right_white_line>60)||(Left_white_line>60))))	//if a node is detected
		{
			velocity(0,0);
			
			if(total_path[pos]==total_path[pos+1])		//a fertilizing unit found
			{
				if(dir[num]==1)							//drop right
				{
					velocity(0,0);				 
					i=27;
					servo_3(i);
					_delay_ms(200);
					i=60;
					servo_3(i);
					_delay_ms(1000);
					pos++;
					num++;
				}
				else                                   //drop right
				{
					velocity(0,0);
					i=87;
					servo_3(i);
					_delay_ms(200);
					i=60;
					servo_3(i);
					_delay_ms(1000);
					pos++;
					num++;
					if((num==7)||(num==9))				//condition for fertilizing plants on both left and right
					{
						velocity(0,0);				 
						i=27;
						servo_3(i);
						_delay_ms(400);
						i=60;
						servo_3(i);
						_delay_ms(1000);
						pos++;
						num++;
					}
				}
		if((num==3)||(num==6))							//500ms buzzer at the end of tuples
		{
			buzzer_on();
			_delay_ms(500);		
			buzzer_off();
		}
		if(num==10)										//5sec buzzer at the end of task
		{
			buzzer_on();
			stop();
			_delay_ms(5000);		
			buzzer_off();
			_delay_ms(5000);	
			break;
		}	
			
		
			}
			
			switch(bot_rotate[pos])											//condition for rotating the bot
			{
				case 1:		break;											//move forward without rorating
				case 2:		in_count=0;										//90 degree right turn
							while(1)
							{
								in_count++;
								soft_right_degrees(5);
								stop();
								Center_white_line = ADC_Conversion(1);
								if((Center_white_line>60)&&(in_count>3))
									break;
							}
							velocity(0,0);
							break;
							
				case 3:		in_count=0;										//180 degree turn
							while(1)
							{
								in_count++;
								right_degrees(5);
								stop();
								Right_white_line = ADC_Conversion(2);
								if((Right_white_line>60)&&(in_count>3))
									break;
							}
						
						break;
						
				case 4:	in_count=0;											//90 degree left turn
						while(1)
							{
								in_count++;
								soft_left_degrees(5);
								stop();
								Center_white_line = ADC_Conversion(3);
								if((Center_white_line>60)&&(in_count>3))
									break;
							}
						velocity(0,0);
						break;
			}
			pos++;
			
		for(v=0;v<2750;v++)													//Delay Loop: Providing a delay after detecting a node before detecting the next one
		{					
			//Line following using Proportional Control
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			x = 250 - 250*(Left_white_line-25)/120;							
			y = 250 -250*(Right_white_line-25)/120;
			x = mod(x);														//so that velocity doesnot exceed 255
			y = mod(y);														//so that velocity doesnot exceed 255
			forward();
			velocity(x,y);
			}
		}
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		//Line following using Proportional Control
		if(Center_white_line>60)											
		{
			flag=1;
			forward();
			velocity(220,220);
		}
		else{					
		x = 250 - 250*(Left_white_line-25)/120;
		y = 250 -250*(Right_white_line-25)/120;
		x = mod(x);
		y = mod(y);
		forward();
		velocity(x,y);
		}
		
	}
}


