#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <unistd.h>
#include <math.h> //cos sin and all
#include <cmath> //pow,sqrt
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

int getch();
void trot(double t);
void walk(double t);
void left(double t);
void right(double t);
void bound(double t);
void wiggle();
void highfiveleft();
void ikforeleft(double fx,double fy);
void ikforeright(double fx,double fy);
void ikhindleft(double fx,double fy);
void ikhindright(double fx,double fy);
double foreleftkneeservoangle(double ft1,double ft2);
double forerightkneeservoangle(double ft1,double ft2);
double hindleftkneeservoangle(double ht1,double ht2);
double hindrightkneeservoangle(double ht1,double ht2);
double forelefthipservoangle(double ft1);
double forerighthipservoangle(double ft1);
double hindlefthipservoangle(double ht1);
double hindrighthipservoangle(double ht1);
double kneeservoangle(double l);
double pi=3.1428;

//foreleg dimensions
double fl1=31.0;
double fl2=26.0;
double fl3=13.0;
double ft1i=54.24;
double ft2i=87.65;
double ft3i=155.0;
//hindleg dimensions
double hl1=35.0;
double hl2=40.0;
double hl3=19.0;
double ht1i=7.49;
double ht2i=108.93;
double ht3i=110.0;

double fpos_init[1][2]={28,55.15};	//corresponding to trot gait
double hpos_init[1][2]={20,59.15};	//corresponding to trot gait

//foreleg and hindleg position matrix for trot and walk trajectory
// double fpos_trot[5][2]={{0,0},{30,0},{60,0},{40,15},{7,10}}; //original values
// double hpos_trot[5][2]={{0,0},{30,0},{60,0},{50,17},{7,12}}; //original values
// double fpos_trotinit[1][2]={33,44.15};	//original values
// double hpos_trotinit[1][2]={20,54.15};	//original values
double fpos_trot[5][2]={{0,0},{25,0},{50,0},{40,25},{12,20}}; 	//refined values
double hpos_trot[5][2]={{0,5},{25,5},{50,0},{45,35},{20,30}};	//refined values
double fpos_trotinit[1][2]={28,55.15};	//refined values
double hpos_trotinit[1][2]={20,59.15};	//refined values

//foreleg and hindleg position matrix for pounce and bound trajectory
double fpos_bound[6][2]={{0,0},{37,1.5},{56,-15.5},{60,4},{30,18},{7,10}}; 
double hpos_bound[6][2]={{0,0},{37,1.5},{56,-15.5},{60,4},{50,20},{7,10}};
double fpos_boundinit[1][2]={35,41.15};
double hpos_boundinit[1][2]={25,51.15};

//global variables of positions for each servos *foreleft=fl, hindleft=hl.
double fla1=108; 
double fla2=73;
double fra1=76;
double fra2=107;
double hla1=139; 
double hla2=110;
double hra1=70; 
double hra2=90;

//******************************************************************************************************************************************************************************************
//Gait patterns and variables
/*
Nice walking gait (https://www.youtube.com/watch?v=dRthdBr46cs)
duty factor=4.3/11
phase diff=2.8/11
fl|-------4.3				11|
fr|--1.6  		 8.3--------11|
hl|	2.7---------7       	11|
hr|			5.5--------9.8  11|
double df=0.39; 		//duty factor (percentage where leg is on the ground)
double phaseoffset=0.26; 
*/

/*
Trotting gait (from siggraph https://www.youtube.com/watch?v=dRthdBr46cs)
duty factor=6.7/11
phase diff=5.6/11
fl|--------------6.7		11|
fr|-1.3	  5.6---------------11|
hl|-1.3   5.6---------------11|
hr|--------------6.7		11|
double df=0.60; 		//duty factor (percentage where leg is on the ground)
double phaseoffset=0.50; 
*/

/*
Bounding gait (from experimentation)
duty factor=0.4
phase diff=0.6
fl|  		 	  6---------10|
fr|   		 	  6---------10|
hl|---------4				10|
hr|---------4				10|
double df=0.4; 		//duty factor (percentage where leg is on the ground)
double phaseoffset=0.6; 
*/
//*****************************************************************************************************************************************************************************************
//global array, this is the one that will be read by the publisher
double stridetime=0.2;	//time taken for leg to move in 1 cycle.Servo can move 1 degree in 2ms, at the moment this is 0.2 due to the bluetooth module issue
double streamfreq=500; 	//how many set of values are published to the teensy per second. (maximum of 500 values per second)
int arrsize=stridetime*streamfreq; 		//error shown if its declared as NRPMouseservoarray[arrsize][9]. 
double NRPMouseservoarray[100][11]; 	//size of array needs to be input manually
double NRPMouseservoarraytemp[100][11]; //temporary sotred for phase difference computation
char dir;								//direction
/*	0-timestamp
	1-fla1	(Foreleft hip servo)
	2-fla2 	(Foreleft knee servo)
	3-fra1	(Foreright hip servo)
	4-fra2	(Foreright knee servo)
	5-hla1	(Hindleft hip servo)
	6-hla2	(Hindleft knee servo)
	7-hra1	(Hindright hip servo)
	8-hra2	(Hindright knee servo) 
	9-spine 
	10-tail 
*/
//********************************************************************************************************************************************************************************

int main(int argc, char **argv)
{
    // ROS objects
  	ros::init(argc, argv, "nrpmouse_pub");
  	std_msgs::Float64MultiArray msgarr;
  	msgarr.data.resize(11); //need to declare the size, else it wont work
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("nrpmouse_servotopic", 512);
	ros::Rate loop_rate(streamfreq); //run at Hz, not sleep time

  	//publishing here
	while (ros::ok())
	{	
		
		dir =getch();
		if (dir=='w')
		{	trot(stridetime);
			std::cout<<"Straight ahead"<<std::endl;}
		else if(dir=='d')
		{	trot(stridetime); 
			right(stridetime);
			std::cout<<"Right Turn"<<std::endl; }
		else if(dir=='a')
		{	trot(stridetime); 
			left(stridetime);
			std::cout<<"Left Turn"<<std::endl; }
		else if (dir=='t')
		{	wiggle();
			std::cout<<"Wiggle"<<std::endl;}
		else if (dir=='b')
		{	bound(stridetime);
			std::cout<<"Bound"<<std::endl;}
		else if (dir=='c')
		{	walk(stridetime);
			std::cout<<"Walk"<<std::endl;}
		else if (dir=='h')
		{	highfiveleft();
			std::cout<<"Gimme five"<<std::endl;}
			
	  	for(int p=0;p<arrsize;p++)	
	  	{	
			msgarr.data[0]=(NRPMouseservoarray[p][0]);
			msgarr.data[1]=(NRPMouseservoarray[p][1]);
			msgarr.data[2]=(NRPMouseservoarray[p][2]);
			msgarr.data[3]=(NRPMouseservoarray[p][3]);
			msgarr.data[4]=(NRPMouseservoarray[p][4]);
			msgarr.data[5]=(NRPMouseservoarray[p][5]);
			msgarr.data[6]=(NRPMouseservoarray[p][6]);
			msgarr.data[7]=(NRPMouseservoarray[p][7]);
			msgarr.data[8]=(NRPMouseservoarray[p][8]);
			msgarr.data[9]=(NRPMouseservoarray[p][9]);
			msgarr.data[10]=(NRPMouseservoarray[p][10]);
			//std::cout<<NRPMouseservoarray[p][9]<<std::endl; 
			pub.publish(msgarr);
			ros::spinOnce();
			loop_rate.sleep();
	  	}
	}	
  	
}

int getch()
{
	//a non blocking getchar()
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering      
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
	int dir = getchar();  // read character (non-blocking)
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return dir;
}

void left(double t)
{	
	double sv =0; //spine value
	//just changing the spine movement now
	//(extreme left, extreme right, difference)=(20, 160, 140)
	double svadd=(140/(arrsize/2));
	sv=0;
	for (int f=0; f<(arrsize/2); f++, sv=sv+svadd)
	{	//spine actuates from 90 to 160
		NRPMouseservoarray[f][9]=90+sv;
	}
	sv=0;
	for (int f=(arrsize/2); f<=arrsize; f++, sv=sv+svadd)
	{	
		//spine actuates from 160 to 90
		NRPMouseservoarray[f][9]=160-sv; 
	}
}
void right(double t)
{	
	double sv =0; //spine value
	//just changing the spine movement now
	//(extreme left, extreme right, difference)=(20, 160, 140)
	double svadd=(140/(arrsize/2));
	sv=0;
	for (int f=0; f<(arrsize/2); f++, sv=sv+svadd)
	{	//spine actuates from 90 to 160
		NRPMouseservoarray[f][9]=20+sv;
	}
	sv=0;
	for (int f=(arrsize/2); f<=arrsize; f++, sv=sv+svadd)
	{	
		//spine actuates from 160 to 90
		NRPMouseservoarray[f][9]=90-sv; 
	}
}

void trot(double t)
{   
	double df=0.60; 		//duty factor (percentage where leg is on the ground)
	double phaseoffset=0.5; 
	//local temporary variables to store previous position states, loop counters
	double fla1i=0; 
	double fla2i=0;
	double fra1i=0;
	double fra2i=0;
	double hla1i=0; 
	double hla2i=0;
	double hra1i=0; 
	double hra2i=0;
	double timeoffset=0;
	int arrcount=0;
	int arrcount2=0;
	int arrcount3=0;
	int arrcount4=0;
	double temp=0;
	double temp2=0;
	int initpos=0;
	int loopedpos=0;
	double sv =0; //spine value
	double svadd_trot;
	double tv =0; //tail value
	double tvadd_trot;

	//initiallise the init position
	fpos_init[0][0]=fpos_trotinit[0][0];
	fpos_init[0][1]=fpos_trotinit[0][1];
	hpos_init[0][0]=hpos_trotinit[0][0];
	hpos_init[0][1]=hpos_trotinit[0][1];

   	//Calculation for front leg time array
    //double tf1=0.5*df*t;
    //double tf2=tf1;
    double tf1=((fpos_trot[1][0]-fpos_trot[0][0])/(fpos_trot[2][0]-fpos_trot[0][0]))*df*t;
    double tf2=((fpos_trot[2][0]-fpos_trot[1][0])/(fpos_trot[2][0]-fpos_trot[0][0]))*df*t ;
    double sumf=sqrt((pow((fpos_trot[2][0]-fpos_trot[3][0]),2))+(pow((fpos_trot[2][1]-fpos_trot[3][1]),2)))+sqrt(pow((fpos_trot[3][0]
    	-fpos_trot[4][0]),2)+pow((fpos_trot[3][1]-fpos_trot[4][1]),2))+sqrt(pow((fpos_trot[4][0]-fpos_trot[0][0]),2)+pow((fpos_trot[4][1]-fpos_trot[0][1]),2));
   	double tf3=sqrt(pow((fpos_trot[2][0]-fpos_trot[3][0]),2)+(pow((fpos_trot[2][1]-fpos_trot[3][1]),2)))/sumf*(1-df)*t;
   	double tf4=sqrt(pow((fpos_trot[3][0]-fpos_trot[4][0]),2)+(pow((fpos_trot[3][1]-fpos_trot[4][1]),2)))/sumf*(1-df)*t;
   	double tf5=sqrt(pow((fpos_trot[4][0]-fpos_trot[0][0]),2)+(pow((fpos_trot[4][1]-fpos_trot[0][1]),2)))/sumf*(1-df)*t;
   	double timef[]={tf1,tf2,tf3,tf4,tf5}; //time array

   	//Calculation for hind leg time array
    double th1=((hpos_trot[1][0]-hpos_trot[0][0])/(hpos_trot[2][0]-hpos_trot[0][0]))*df*t;
   	double th2=((hpos_trot[2][0]-hpos_trot[1][0])/(hpos_trot[2][0]-hpos_trot[0][0]))*df*t ;
   	double sumh=sqrt((pow((hpos_trot[2][0]-hpos_trot[3][0]),2))+(pow((hpos_trot[2][1]-hpos_trot[3][1]),2)))+sqrt(pow((hpos_trot[3][0]
   		-hpos_trot[4][0]),2)+pow((hpos_trot[3][1]-hpos_trot[4][1]),2))+sqrt(pow((hpos_trot[4][0]-hpos_trot[0][0]),2)+pow((hpos_trot[4][1]-hpos_trot[0][1]),2));
   	double th3=(sqrt(pow((hpos_trot[2][0]-hpos_trot[3][0]),2)+(pow((hpos_trot[2][1]-hpos_trot[3][1]),2)))/sumh*(1-df)*t);
   	double th4=(sqrt(pow((hpos_trot[3][0]-hpos_trot[4][0]),2)+(pow((hpos_trot[3][1]-hpos_trot[4][1]),2)))/sumh*(1-df)*t);
   	double th5=(sqrt(pow((hpos_trot[4][0]-hpos_trot[0][0]),2)+(pow((hpos_trot[4][1]-hpos_trot[0][1]),2)))/sumh*(1-df)*t);
   	double timeh[]={th1,th2,th3,th4,th5}; //time array 
   	
	//this part writes all the positions into a file and stored as an array with time stamp
	//covering pos 1 to pos 4
	for (int i=0;i<=3;i++)
	{	
			ikhindleft(hpos_trot[i][0],hpos_trot[i][1]);
			hla1i=hla1; hla2i=hla2;
			ikhindleft(hpos_trot[i+1][0],hpos_trot[i+1][1]);
			ikhindright(hpos_trot[i][0],hpos_trot[i][1]);
			hra1i=hra1; hra2i=hra2;
			ikhindright(hpos_trot[i+1][0],hpos_trot[i+1][1]);

			ikforeleft(fpos_trot[i][0],fpos_trot[i][1]);
			fla1i=fla1; fla2i=fla2;
			ikforeleft(fpos_trot[i+1][0],fpos_trot[i+1][1]);
			ikforeright(fpos_trot[i][0],fpos_trot[i][1]);
			fra1i=fra1; fra2i=fra2;
			ikforeright(fpos_trot[i+1][0],fpos_trot[i+1][1]);

			for (temp=0;temp<timef[i];temp=temp+(1/streamfreq),arrcount++)
			{	
				if (fla1>fla1i){
					 NRPMouseservoarray[arrcount][1]= fla1i+((fla1-fla1i)/(timef[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount][1]= fla1i-((fla1i-fla1)/(timef[i])*(temp));}
				if (fla2>fla2i) {
					 NRPMouseservoarray[arrcount][2]= fla2i+((fla2-fla2i)/(timef[i])*(temp));}
				else    {NRPMouseservoarray[arrcount][2]= fla2i-((fla2i-fla2)/(timef[i])*(temp));}

				if (fra1>fra1i){
					 NRPMouseservoarray[arrcount][3]= fra1i+((fra1-fra1i)/(timef[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount][3]= fra1i-((fra1i-fra1)/(timef[i])*(temp));}
				if (fra2>fra2i) {
					 NRPMouseservoarray[arrcount][4]= fra2i+((fra2-fra2i)/(timef[i])*(temp));}
				else    {NRPMouseservoarray[arrcount][4]= fra2i-((fra2i-fra2)/(timef[i])*(temp));}
				 
				NRPMouseservoarray[arrcount][0]= temp+timeoffset; 
			}
			for (temp=0;temp<timeh[i];temp=temp+(1/streamfreq),arrcount3++)
			{	
				if (hla1>hla1i){
					 NRPMouseservoarray[arrcount3][5]= hla1i+((hla1-hla1i)/(timeh[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount3][5]= hla1i-((hla1i-hla1)/(timeh[i])*(temp));}
				if (hla2>hla2i) {
					 NRPMouseservoarray[arrcount3][6]= hla2i+((hla2-hla2i)/(timeh[i])*(temp));}
				else    {NRPMouseservoarray[arrcount3][6]= hla2i-((hla2i-hla2)/(timeh[i])*(temp));}

				if (hra1>hra1i){
					 NRPMouseservoarray[arrcount3][7]= hra1i+((hra1-hra1i)/(timeh[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount3][7]= hra1i-((hra1i-hra1)/(timeh[i])*(temp));}
				if (hra2>hra2i) {
					 NRPMouseservoarray[arrcount3][8]= hra2i+((hra2-hra2i)/(timeh[i])*(temp));}
				else    {NRPMouseservoarray[arrcount3][8]= hra2i-((hra2i-hra2)/(timeh[i])*(temp));}
				 
				//NRPMouseservoarray[arrcount3][0]= temp+timeoffset;
			}
		timeoffset=timeoffset+timef[i];
	}
	
	//covering pos5 to pos 1
	ikhindleft(hpos_trot[4][0],hpos_trot[4][1]);
	hla1i=hla1; hla2i=hla2;
	ikhindleft(hpos_trot[0][0],hpos_trot[0][1]);
	ikhindright(hpos_trot[4][0],hpos_trot[4][1]);
	hra1i=hra1; hra2i=hra2;
	ikhindright(hpos_trot[0][0],hpos_trot[0][1]);

	ikforeleft(fpos_trot[4][0],fpos_trot[4][1]);
	fla1i=fla1; fla2i=fla2;
	ikforeleft(fpos_trot[0][0],fpos_trot[0][1]);
	ikforeright(fpos_trot[4][0],fpos_trot[4][1]);
	fra1i=fra1; fra2i=fra2;
	ikforeright(fpos_trot[0][0],fpos_trot[0][1]);

	for (temp2=0, arrcount2=arrcount;temp2<timef[4];temp2=temp2+(1/streamfreq),arrcount2++)
	{	
		if (fla1>fla1i){
			NRPMouseservoarray[arrcount2][1]= fla1i+((fla1-fla1i)/(timef[4])*(temp2));}
		else 	{NRPMouseservoarray[arrcount2][1]= fla1i-((fla1i-fla1)/(timef[4])*(temp2));}
		if (fla2>fla2i) {
			NRPMouseservoarray[arrcount2][2]= fla2i+((fla2-fla2i)/(timef[4])*(temp2));}
		else    {NRPMouseservoarray[arrcount2][2]= fla2i-((fla2i-fla2)/(timef[4])*(temp2));}

		if (fra1>fra1i){
			NRPMouseservoarray[arrcount2][3]= fra1i+((fra1-fra1i)/(timef[4])*(temp2));}
		else 	{NRPMouseservoarray[arrcount2][3]= fra1i-((fra1i-fra1)/(timef[4])*(temp2));}
		if (fra2>fra2i) {
			NRPMouseservoarray[arrcount2][4]= fra2i+((fra2-fra2i)/(timef[4])*(temp2));}
		else    {NRPMouseservoarray[arrcount2][4]= fra2i-((fra2i-fra2)/(timef[4])*(temp2));}

		NRPMouseservoarray[arrcount2][0]= temp2+timef[3]+timef[2]+timef[1]+timef[0];
	}
	for (temp2=0, arrcount4=arrcount3;temp2<timeh[4];temp2=temp2+(1/streamfreq),arrcount4++)
	{	
		if (hla1>hla1i){
			NRPMouseservoarray[arrcount4][5]= hla1i+((hla1-hla1i)/(timeh[4])*(temp2)); }
		else 	{NRPMouseservoarray[arrcount4][5]= hla1i-((hla1i-hla1)/(timeh[4])*(temp2)); }
		if (hla2>hla2i) {
			NRPMouseservoarray[arrcount4][6]= hla2i+((hla2-hla2i)/(timeh[4])*(temp2)); }
		else    {NRPMouseservoarray[arrcount4][6]= hla2i-((hla2i-hla2)/(timeh[4])*(temp2)); }

		if (hra1>hra1i){
			NRPMouseservoarray[arrcount4][7]= hra1i+((hra1-hra1i)/(timeh[4])*(temp2));}
		else 	{NRPMouseservoarray[arrcount4][7]= hra1i-((hra1i-hra1)/(timeh[4])*(temp2));}
		if (hra2>hra2i) {
			NRPMouseservoarray[arrcount4][8]= hra2i+((hra2-hra2i)/(timeh[4])*(temp2));}
		else    {NRPMouseservoarray[arrcount4][8]= hra2i-((hra2i-hra2)/(timeh[4])*(temp2));}

		//NRPMouseservoarray[arrcount4][0]= temp2+timeh[3]+timeh[2]+timeh[1]+timeh[0];
	}
	
	//This part shifts the arrays accoring to the phase difference
	//1) Create a temporary array identical to created
	for (int s=0; s<arrcount2;s++)
	{ 	NRPMouseservoarraytemp[s][0]=NRPMouseservoarray[s][0];
		NRPMouseservoarraytemp[s][1]=NRPMouseservoarray[s][1];
		NRPMouseservoarraytemp[s][2]=NRPMouseservoarray[s][2];
		NRPMouseservoarraytemp[s][3]=NRPMouseservoarray[s][3];
		NRPMouseservoarraytemp[s][4]=NRPMouseservoarray[s][4];
		NRPMouseservoarraytemp[s][5]=NRPMouseservoarray[s][5];
		NRPMouseservoarraytemp[s][6]=NRPMouseservoarray[s][6];
		NRPMouseservoarraytemp[s][7]=NRPMouseservoarray[s][7];
		NRPMouseservoarraytemp[s][8]=NRPMouseservoarray[s][8];
	}
	//2) FInd the initial position in the array where the legs are supposed to start
	initpos=phaseoffset*arrsize;
	loopedpos=arrsize-initpos;

	//3) Replace the array values by a phase diff
	for (int u=initpos, v=0 ;u<arrsize;u++,v++)
	{	NRPMouseservoarray[v][3]=NRPMouseservoarraytemp[u][3]; //fra1
		NRPMouseservoarray[v][4]=NRPMouseservoarraytemp[u][4]; //fra2
		NRPMouseservoarray[v][5]=NRPMouseservoarraytemp[u][5]; //hla1
		NRPMouseservoarray[v][6]=NRPMouseservoarraytemp[u][6]; //hla2
	}
	for (int g=0, h=loopedpos;g<initpos;g++,h++)
	{	NRPMouseservoarray[h][3]=NRPMouseservoarraytemp[g][3]; //fra1
		NRPMouseservoarray[h][4]=NRPMouseservoarraytemp[g][4]; //fra2
		NRPMouseservoarray[h][5]=NRPMouseservoarraytemp[g][5]; //hla1
		NRPMouseservoarray[h][6]=NRPMouseservoarraytemp[g][6]; //hla2
	}
	
	//this part is for the spine movement
	//(extreme left, extreme right, difference, center)=(50 c, 130 inverted c, 80, 90)
	sv=0;
	svadd_trot=(80/(arrsize/2));
	for (int f=0; f<(arrsize/2); f++, sv=sv+svadd_trot)
	{	//spine actuates from 50 to 130
		NRPMouseservoarray[f][9]=50+sv;
	}
	sv=0;
	for (int f=(arrsize/2); f<=arrsize; f++, sv=sv+svadd_trot)
	{	
		//spine actuates from 130 to 50
		NRPMouseservoarray[f][9]=130-sv; 
	}
	
	//this part is for the tail movement
	//(extreme left, extreme right, difference, center)=(140, 40, 100, 90)
	tv=0;
	tvadd_trot=(100/(arrsize/2));
	for (int f=0; f<(arrsize/4); f++, tv=tv+tvadd_trot)
	{	
		//tail actuates from 90 to 40
		NRPMouseservoarray[f][10]=90-tv;
	}
	tv=0;
	for (int f=arrsize/4; f<(3*arrsize/4); f++, tv=tv+tvadd_trot)
	{	
		//tail actuates from 40 to 140
		NRPMouseservoarray[f][10]=40+tv;
	}
	tv=0;
	for (int f=(3*arrsize/4); f<=arrsize; f++, tv=tv+tvadd_trot)
	{	
		
		//tail actuates from 140 to 90
		NRPMouseservoarray[f][10]=140-tv;
	}
}

void bound(double t)
{	
	//at the moment this gait is not usable, motors are too slow and this requires a verticle actuation of the spine
	double df=0.4; 		//duty factor (percentage where leg is on the ground)
	double phaseoffset=0.6;  
	//local temporary variables to store previous position states, loop counters
	double fla1i=0; 
	double fla2i=0;
	double fra1i=0;
	double fra2i=0;
	double hla1i=0; 
	double hla2i=0;
	double hra1i=0; 
	double hra2i=0;
	double timeoffset=0;
	int arrcount=0;
	int arrcount2=0;
	int arrcount3=0;
	int arrcount4=0;
	double temp=0;
	double temp2=0;
	int initpos=0;
	int loopedpos=0;

	//trajectory generation for push segment
	double pushseg_df=0.4; //percentage of the duty factor where the leg is in push mode (shows when to start pushing)
	double pushseg_height=15; //height of the push segment
	double pushseg_percentlength=0.6; //percentage of the push segment use for downward motion
	double pushseg_time=0.4; //time percentage of the push segment used for downward motion

	//calculations for new points for push segment (overwrite the ones at the top)
	fpos_bound[1][0]=fpos_bound[3][0]*(1-pushseg_df); //point where it starts the push segment
	fpos_bound[2][0]=fpos_bound[1][0]+(fpos_bound[3][0]-fpos_bound[1][0])*pushseg_percentlength;//x axis lowest point of push segment
	fpos_bound[2][1]=fpos_bound[1][1]-pushseg_height;//y axis lowest point of push segment
	hpos_bound[1][0]=fpos_bound[1][0]; //they are the same for now, can try out with different trajectory too
	hpos_bound[2][0]=fpos_bound[2][0]; //hind leg can be larger  etc etc
	hpos_bound[2][1]=fpos_bound[2][1];

	fpos_init[0][0]=fpos_boundinit[0][0];
	fpos_init[0][1]=fpos_boundinit[0][1];
	hpos_init[0][0]=hpos_boundinit[0][0];
	hpos_init[0][1]=hpos_boundinit[0][1];

   	//Calculation for front leg time array
   	//t1 is for propel segment (propelling the mouse forward)
   	//t2 t3 are for push segment
   	//t4 t5 and t6 are for when the legs are above the ground (hence the trajectory of the leg above the ground can be different, but still maintain the same step length)
    double tf1=(1-pushseg_df)*df*t;
    double tf2=(pushseg_time)*(pushseg_df)*df*t;
    double tf3=(1-pushseg_time)*(pushseg_df)*df*t;
    double sumf=sqrt((pow((fpos_bound[3][0]-fpos_bound[4][0]),2))+(pow((fpos_bound[3][1]-fpos_bound[4][1]),2)))+sqrt(pow((fpos_bound[4][0]
    	-fpos_bound[5][0]),2)+pow((fpos_bound[4][1]-fpos_bound[5][1]),2))+sqrt(pow((fpos_bound[5][0]-fpos_bound[0][0]),2)+pow((fpos_bound[5][1]-fpos_bound[0][1]),2));
   	double tf4=sqrt(pow((fpos_bound[3][0]-fpos_bound[4][0]),2)+(pow((fpos_bound[3][1]-fpos_bound[4][1]),2)))/sumf*(1-df)*t;
   	double tf5=sqrt(pow((fpos_bound[4][0]-fpos_bound[5][0]),2)+(pow((fpos_bound[4][1]-fpos_bound[5][1]),2)))/sumf*(1-df)*t;
   	double tf6=sqrt(pow((fpos_bound[5][0]-fpos_bound[0][0]),2)+(pow((fpos_bound[5][1]-fpos_bound[0][1]),2)))/sumf*(1-df)*t;
   	double timef[]={tf1,tf2,tf3,tf4,tf5,tf6}; //time array

   	//Calculation for hind leg time array
    double th1=(1-pushseg_df)*df*t;
    double th2=(pushseg_time)*(pushseg_df)*df*t;
    double th3=(1-pushseg_time)*(pushseg_df)*df*t;
   	double sumh=sqrt((pow((hpos_bound[3][0]-hpos_bound[4][0]),2))+(pow((hpos_bound[3][1]-hpos_bound[4][1]),2)))+sqrt(pow((hpos_bound[4][0]
   		-hpos_bound[5][0]),2)+pow((hpos_bound[4][1]-hpos_bound[5][1]),2))+sqrt(pow((hpos_bound[5][0]-hpos_bound[0][0]),2)+pow((hpos_bound[5][1]-hpos_bound[0][1]),2));
   	double th4=(sqrt(pow((hpos_bound[3][0]-hpos_bound[4][0]),2)+(pow((hpos_bound[3][1]-hpos_bound[4][1]),2)))/sumh*(1-df)*t);
   	double th5=(sqrt(pow((hpos_bound[4][0]-hpos_bound[5][0]),2)+(pow((hpos_bound[4][1]-hpos_bound[5][1]),2)))/sumh*(1-df)*t);
   	double th6=(sqrt(pow((hpos_bound[5][0]-hpos_bound[0][0]),2)+(pow((hpos_bound[5][1]-hpos_bound[0][1]),2)))/sumh*(1-df)*t);
   	double timeh[]={th1,th2,th3,th4,th5,th6}; //time array 
   	
	//this part writes all the positions into a file and stored as an array with time stamp
	//covering pos 1 to pos 5
	for (int i=0;i<=4;i++)
	{	
			ikhindleft(hpos_bound[i][0],hpos_bound[i][1]);
			hla1i=hla1; hla2i=hla2;
			ikhindleft(hpos_bound[i+1][0],hpos_bound[i+1][1]);
			ikhindright(hpos_bound[i][0],hpos_bound[i][1]);
			hra1i=hra1; hra2i=hra2;
			ikhindright(hpos_bound[i+1][0],hpos_bound[i+1][1]);

			ikforeleft(fpos_bound[i][0],fpos_bound[i][1]);
			fla1i=fla1; fla2i=fla2;
			ikforeleft(fpos_bound[i+1][0],fpos_bound[i+1][1]);
			ikforeright(fpos_bound[i][0],fpos_bound[i][1]);
			fra1i=fra1; fra2i=fra2;
			ikforeright(fpos_bound[i+1][0],fpos_bound[i+1][1]);

			for (temp=0;temp<timef[i];temp=temp+(1/streamfreq),arrcount++)
			{	
				if (fla1>fla1i){
					 NRPMouseservoarray[arrcount][1]= fla1i+((fla1-fla1i)/(timef[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount][1]= fla1i-((fla1i-fla1)/(timef[i])*(temp));}
				if (fla2>fla2i) {
					 NRPMouseservoarray[arrcount][2]= fla2i+((fla2-fla2i)/(timef[i])*(temp));}
				else    {NRPMouseservoarray[arrcount][2]= fla2i-((fla2i-fla2)/(timef[i])*(temp));}

				if (fra1>fra1i){
					 NRPMouseservoarray[arrcount][3]= fra1i+((fra1-fra1i)/(timef[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount][3]= fra1i-((fra1i-fra1)/(timef[i])*(temp));}
				if (fra2>fra2i) {
					 NRPMouseservoarray[arrcount][4]= fra2i+((fra2-fra2i)/(timef[i])*(temp));}
				else    {NRPMouseservoarray[arrcount][4]= fra2i-((fra2i-fra2)/(timef[i])*(temp));}
				 
				NRPMouseservoarray[arrcount][0]= temp+timeoffset; 
			}
			for (temp=0;temp<timeh[i];temp=temp+(1/streamfreq),arrcount3++)
			{	
				if (hla1>hla1i){
					 NRPMouseservoarray[arrcount3][5]= hla1i+((hla1-hla1i)/(timeh[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount3][5]= hla1i-((hla1i-hla1)/(timeh[i])*(temp));}
				if (hla2>hla2i) {
					 NRPMouseservoarray[arrcount3][6]= hla2i+((hla2-hla2i)/(timeh[i])*(temp));}
				else    {NRPMouseservoarray[arrcount3][6]= hla2i-((hla2i-hla2)/(timeh[i])*(temp));}

				if (hra1>hra1i){
					 NRPMouseservoarray[arrcount3][7]= hra1i+((hra1-hra1i)/(timeh[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount3][7]= hra1i-((hra1i-hra1)/(timeh[i])*(temp));}
				if (hra2>hra2i) {
					 NRPMouseservoarray[arrcount3][8]= hra2i+((hra2-hra2i)/(timeh[i])*(temp));}
				else    {NRPMouseservoarray[arrcount3][8]= hra2i-((hra2i-hra2)/(timeh[i])*(temp));}
				 
				//NRPMouseservoarray[arrcount3][0]= temp+timeoffset;
			}
		timeoffset=timeoffset+timef[i];
	}
	
	//covering pos6 to pos 1
	ikhindleft(hpos_bound[5][0],hpos_bound[5][1]);
	hla1i=hla1; hla2i=hla2;
	ikhindleft(hpos_bound[0][0],hpos_bound[0][1]);
	ikhindright(hpos_bound[5][0],hpos_bound[5][1]);
	hra1i=hra1; hra2i=hra2;
	ikhindright(hpos_bound[0][0],hpos_bound[0][1]);

	ikforeleft(fpos_bound[5][0],fpos_bound[5][1]);
	fla1i=fla1; fla2i=fla2;
	ikforeleft(fpos_bound[0][0],fpos_bound[0][1]);
	ikforeright(fpos_bound[5][0],fpos_bound[5][1]);
	fra1i=fra1; fra2i=fra2;
	ikforeright(fpos_bound[0][0],fpos_bound[0][1]);

	for (temp2=0, arrcount2=arrcount;temp2<timef[5];temp2=temp2+(1/streamfreq),arrcount2++)
	{	
		if (fla1>fla1i){
			NRPMouseservoarray[arrcount2][1]= fla1i+((fla1-fla1i)/(timef[5])*(temp2));}
		else 	{NRPMouseservoarray[arrcount2][1]= fla1i-((fla1i-fla1)/(timef[5])*(temp2));}
		if (fla2>fla2i) {
			NRPMouseservoarray[arrcount2][2]= fla2i+((fla2-fla2i)/(timef[5])*(temp2));}
		else    {NRPMouseservoarray[arrcount2][2]= fla2i-((fla2i-fla2)/(timef[5])*(temp2));}

		if (fra1>fra1i){
			NRPMouseservoarray[arrcount2][3]= fra1i+((fra1-fra1i)/(timef[5])*(temp2));}
		else 	{NRPMouseservoarray[arrcount2][3]= fra1i-((fra1i-fra1)/(timef[5])*(temp2));}
		if (fra2>fra2i) {
			NRPMouseservoarray[arrcount2][4]= fra2i+((fra2-fra2i)/(timef[5])*(temp2));}
		else    {NRPMouseservoarray[arrcount2][4]= fra2i-((fra2i-fra2)/(timef[5])*(temp2));}

		NRPMouseservoarray[arrcount2][0]= temp2+timef[4]+timef[3]+timef[2]+timef[1]+timef[0];
	}
	for (temp2=0, arrcount4=arrcount3;temp2<timeh[5];temp2=temp2+(1/streamfreq),arrcount4++)
	{	
		if (hla1>hla1i){
			NRPMouseservoarray[arrcount4][5]= hla1i+((hla1-hla1i)/(timeh[5])*(temp2)); }
		else 	{NRPMouseservoarray[arrcount4][5]= hla1i-((hla1i-hla1)/(timeh[5])*(temp2)); }
		if (hla2>hla2i) {
			NRPMouseservoarray[arrcount4][6]= hla2i+((hla2-hla2i)/(timeh[5])*(temp2)); }
		else    {NRPMouseservoarray[arrcount4][6]= hla2i-((hla2i-hla2)/(timeh[5])*(temp2)); }

		if (hra1>hra1i){
			NRPMouseservoarray[arrcount4][7]= hra1i+((hra1-hra1i)/(timeh[5])*(temp2));}
		else 	{NRPMouseservoarray[arrcount4][7]= hra1i-((hra1i-hra1)/(timeh[5])*(temp2));}
		if (hra2>hra2i) {
			NRPMouseservoarray[arrcount4][8]= hra2i+((hra2-hra2i)/(timeh[5])*(temp2));}
		else    {NRPMouseservoarray[arrcount4][8]= hra2i-((hra2i-hra2)/(timeh[5])*(temp2));}

		//NRPMouseservoarray[arrcount4][0]= temp2+timeh[3]+timeh[2]+timeh[1]+timeh[0];
	}
	
	//This part shifts the arrays accoring to the phase difference
	//1) Create a temporary array identical to created
	for (int s=0; s<arrcount2;s++)
	{ 	NRPMouseservoarraytemp[s][0]=NRPMouseservoarray[s][0];
		NRPMouseservoarraytemp[s][1]=NRPMouseservoarray[s][1];
		NRPMouseservoarraytemp[s][2]=NRPMouseservoarray[s][2];
		NRPMouseservoarraytemp[s][3]=NRPMouseservoarray[s][3];
		NRPMouseservoarraytemp[s][4]=NRPMouseservoarray[s][4];
		NRPMouseservoarraytemp[s][5]=NRPMouseservoarray[s][5];
		NRPMouseservoarraytemp[s][6]=NRPMouseservoarray[s][6];
		NRPMouseservoarraytemp[s][7]=NRPMouseservoarray[s][7];
		NRPMouseservoarraytemp[s][8]=NRPMouseservoarray[s][8];
	}
	//2) FInd the initial position in the array where the legs are supposed to start
	initpos=phaseoffset*arrsize;
	loopedpos=arrsize-initpos;

	//3) Replace the array values by a phase dif
	for (int u=initpos, v=0 ;u<arrsize;u++,v++)
	{	NRPMouseservoarray[v][1]=NRPMouseservoarraytemp[u][1]; //fla1
		NRPMouseservoarray[v][2]=NRPMouseservoarraytemp[u][2]; //fla2
		NRPMouseservoarray[v][3]=NRPMouseservoarraytemp[u][3]; //fra1
		NRPMouseservoarray[v][4]=NRPMouseservoarraytemp[u][4]; //fra2
	}
	for (int g=0, h=loopedpos;g<initpos;g++,h++)
	{	NRPMouseservoarray[h][1]=NRPMouseservoarraytemp[g][1]; //fla1
		NRPMouseservoarray[h][2]=NRPMouseservoarraytemp[g][2]; //fla2
		NRPMouseservoarray[h][3]=NRPMouseservoarraytemp[g][3]; //fra1
		NRPMouseservoarray[h][4]=NRPMouseservoarraytemp[g][4]; //fra2
	}
	
	//no horizontal spine movement
	for (int f=0; f<=arrsize;f++)
	{ NRPMouseservoarray[f][9]=90;}
}

void walk(double t)
{	
	//at the moment this is also not usable, limited by motor speed and spine actuation
	double df=0.39; 		//duty factor (percentage where leg is on the ground)
	double phaseoffset=0.24;  
	//local temporary variables to store previous position states, loop counters
	double fla1i=0; 
	double fla2i=0;
	double fra1i=0;
	double fra2i=0;
	double hla1i=0; 
	double hla2i=0;
	double hra1i=0; 
	double hra2i=0;
	double timeoffset=0;
	int arrcount=0;
	int arrcount2=0;
	int arrcount3=0;
	int arrcount4=0;
	double temp=0;
	double temp2=0;
	int initpos=0;
	int loopedpos=0;
	double sv =0; //spine value
	double svadd_walk;

	fpos_init[0][0]=fpos_trotinit[0][0];
	fpos_init[0][1]=fpos_trotinit[0][1];
	hpos_init[0][0]=hpos_trotinit[0][0];
	hpos_init[0][1]=hpos_trotinit[0][1];

   	//Calculation for front leg time array
   	//t1 and t2 are for when the leg touches the ground
   	//t3 t4 and t5 are for when the legs are above the ground (hence the trajectory of the leg above the ground can be different, but still maintain the same step length)
    double tf1=((fpos_trot[1][0]-fpos_trot[0][0])/(fpos_trot[2][0]-fpos_trot[0][0]))*df*t;
    double tf2=((fpos_trot[2][0]-fpos_trot[1][0])/(fpos_trot[2][0]-fpos_trot[0][0]))*df*t ;
    double sumf=sqrt((pow((fpos_trot[2][0]-fpos_trot[3][0]),2))+(pow((fpos_trot[2][1]-fpos_trot[3][1]),2)))+sqrt(pow((fpos_trot[3][0]
    	-fpos_trot[4][0]),2)+pow((fpos_trot[3][1]-fpos_trot[4][1]),2))+sqrt(pow((fpos_trot[4][0]-fpos_trot[0][0]),2)+pow((fpos_trot[4][1]-fpos_trot[0][1]),2));
   	double tf3=sqrt(pow((fpos_trot[2][0]-fpos_trot[3][0]),2)+(pow((fpos_trot[2][1]-fpos_trot[3][1]),2)))/sumf*(1-df)*t;
   	double tf4=sqrt(pow((fpos_trot[3][0]-fpos_trot[4][0]),2)+(pow((fpos_trot[3][1]-fpos_trot[4][1]),2)))/sumf*(1-df)*t;
   	double tf5=sqrt(pow((fpos_trot[4][0]-fpos_trot[0][0]),2)+(pow((fpos_trot[4][1]-fpos_trot[0][1]),2)))/sumf*(1-df)*t;
   	double timef[]={tf1,tf2,tf3,tf4,tf5}; //time array

   	//Calculation for hind leg time array
    double th1=((hpos_trot[1][0]-hpos_trot[0][0])/(hpos_trot[2][0]-hpos_trot[0][0]))*df*t;
   	double th2=((hpos_trot[2][0]-hpos_trot[1][0])/(hpos_trot[2][0]-hpos_trot[0][0]))*df*t ;
   	double sumh=sqrt((pow((hpos_trot[2][0]-hpos_trot[3][0]),2))+(pow((hpos_trot[2][1]-hpos_trot[3][1]),2)))+sqrt(pow((hpos_trot[3][0]
   		-hpos_trot[4][0]),2)+pow((hpos_trot[3][1]-hpos_trot[4][1]),2))+sqrt(pow((hpos_trot[4][0]-hpos_trot[0][0]),2)+pow((hpos_trot[4][1]-hpos_trot[0][1]),2));
   	double th3=(sqrt(pow((hpos_trot[2][0]-hpos_trot[3][0]),2)+(pow((hpos_trot[2][1]-hpos_trot[3][1]),2)))/sumh*(1-df)*t);
   	double th4=(sqrt(pow((hpos_trot[3][0]-hpos_trot[4][0]),2)+(pow((hpos_trot[3][1]-hpos_trot[4][1]),2)))/sumh*(1-df)*t);
   	double th5=(sqrt(pow((hpos_trot[4][0]-hpos_trot[0][0]),2)+(pow((hpos_trot[4][1]-hpos_trot[0][1]),2)))/sumh*(1-df)*t);
   	double timeh[]={th1,th2,th3,th4,th5}; //time array 
   	
	//this part writes all the positions into a file and stored as an array with time stamp
	//covering pos 1 to pos 4
	for (int i=0;i<=3;i++)
	{	
			ikhindleft(hpos_trot[i][0],hpos_trot[i][1]);
			hla1i=hla1; hla2i=hla2;
			ikhindleft(hpos_trot[i+1][0],hpos_trot[i+1][1]);
			ikhindright(hpos_trot[i][0],hpos_trot[i][1]);
			hra1i=hra1; hra2i=hra2;
			ikhindright(hpos_trot[i+1][0],hpos_trot[i+1][1]);

			ikforeleft(fpos_trot[i][0],fpos_trot[i][1]);
			fla1i=fla1; fla2i=fla2;
			ikforeleft(fpos_trot[i+1][0],fpos_trot[i+1][1]);
			ikforeright(fpos_trot[i][0],fpos_trot[i][1]);
			fra1i=fra1; fra2i=fra2;
			ikforeright(fpos_trot[i+1][0],fpos_trot[i+1][1]);

			for (temp=0;temp<timef[i];temp=temp+(1/streamfreq),arrcount++)
			{	
				if (fla1>fla1i){
					 NRPMouseservoarray[arrcount][1]= fla1i+((fla1-fla1i)/(timef[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount][1]= fla1i-((fla1i-fla1)/(timef[i])*(temp));}
				if (fla2>fla2i) {
					 NRPMouseservoarray[arrcount][2]= fla2i+((fla2-fla2i)/(timef[i])*(temp));}
				else    {NRPMouseservoarray[arrcount][2]= fla2i-((fla2i-fla2)/(timef[i])*(temp));}

				if (fra1>fra1i){
					 NRPMouseservoarray[arrcount][3]= fra1i+((fra1-fra1i)/(timef[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount][3]= fra1i-((fra1i-fra1)/(timef[i])*(temp));}
				if (fra2>fra2i) {
					 NRPMouseservoarray[arrcount][4]= fra2i+((fra2-fra2i)/(timef[i])*(temp));}
				else    {NRPMouseservoarray[arrcount][4]= fra2i-((fra2i-fra2)/(timef[i])*(temp));}
				 
				NRPMouseservoarray[arrcount][0]= temp+timeoffset; 
			}
			for (temp=0;temp<timeh[i];temp=temp+(1/streamfreq),arrcount3++)
			{	
				if (hla1>hla1i){
					 NRPMouseservoarray[arrcount3][5]= hla1i+((hla1-hla1i)/(timeh[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount3][5]= hla1i-((hla1i-hla1)/(timeh[i])*(temp));}
				if (hla2>hla2i) {
					 NRPMouseservoarray[arrcount3][6]= hla2i+((hla2-hla2i)/(timeh[i])*(temp));}
				else    {NRPMouseservoarray[arrcount3][6]= hla2i-((hla2i-hla2)/(timeh[i])*(temp));}

				if (hra1>hra1i){
					 NRPMouseservoarray[arrcount3][7]= hra1i+((hra1-hra1i)/(timeh[i])*(temp));}
				else 	{NRPMouseservoarray[arrcount3][7]= hra1i-((hra1i-hra1)/(timeh[i])*(temp));}
				if (hra2>hra2i) {
					 NRPMouseservoarray[arrcount3][8]= hra2i+((hra2-hra2i)/(timeh[i])*(temp));}
				else    {NRPMouseservoarray[arrcount3][8]= hra2i-((hra2i-hra2)/(timeh[i])*(temp));}
				 
				//NRPMouseservoarray[arrcount3][0]= temp+timeoffset;
			}
		timeoffset=timeoffset+timef[i];
	}
	
	//covering pos5 to pos 1
	ikhindleft(hpos_trot[4][0],hpos_trot[4][1]);
	hla1i=hla1; hla2i=hla2;
	ikhindleft(hpos_trot[0][0],hpos_trot[0][1]);
	ikhindright(hpos_trot[4][0],hpos_trot[4][1]);
	hra1i=hra1; hra2i=hra2;
	ikhindright(hpos_trot[0][0],hpos_trot[0][1]);

	ikforeleft(fpos_trot[4][0],fpos_trot[4][1]);
	fla1i=fla1; fla2i=fla2;
	ikforeleft(fpos_trot[0][0],fpos_trot[0][1]);
	ikforeright(fpos_trot[4][0],fpos_trot[4][1]);
	fra1i=fra1; fra2i=fra2;
	ikforeright(fpos_trot[0][0],fpos_trot[0][1]);

	for (temp2=0, arrcount2=arrcount;temp2<timef[4];temp2=temp2+(1/streamfreq),arrcount2++)
	{	
		if (fla1>fla1i){
			NRPMouseservoarray[arrcount2][1]= fla1i+((fla1-fla1i)/(timef[4])*(temp2));}
		else 	{NRPMouseservoarray[arrcount2][1]= fla1i-((fla1i-fla1)/(timef[4])*(temp2));}
		if (fla2>fla2i) {
			NRPMouseservoarray[arrcount2][2]= fla2i+((fla2-fla2i)/(timef[4])*(temp2));}
		else    {NRPMouseservoarray[arrcount2][2]= fla2i-((fla2i-fla2)/(timef[4])*(temp2));}

		if (fra1>fra1i){
			NRPMouseservoarray[arrcount2][3]= fra1i+((fra1-fra1i)/(timef[4])*(temp2));}
		else 	{NRPMouseservoarray[arrcount2][3]= fra1i-((fra1i-fra1)/(timef[4])*(temp2));}
		if (fra2>fra2i) {
			NRPMouseservoarray[arrcount2][4]= fra2i+((fra2-fra2i)/(timef[4])*(temp2));}
		else    {NRPMouseservoarray[arrcount2][4]= fra2i-((fra2i-fra2)/(timef[4])*(temp2));}

		NRPMouseservoarray[arrcount2][0]= temp2+timef[3]+timef[2]+timef[1]+timef[0];
	}
	for (temp2=0, arrcount4=arrcount3;temp2<timeh[4];temp2=temp2+(1/streamfreq),arrcount4++)
	{	
		if (hla1>hla1i){
			NRPMouseservoarray[arrcount4][5]= hla1i+((hla1-hla1i)/(timeh[4])*(temp2)); }
		else 	{NRPMouseservoarray[arrcount4][5]= hla1i-((hla1i-hla1)/(timeh[4])*(temp2)); }
		if (hla2>hla2i) {
			NRPMouseservoarray[arrcount4][6]= hla2i+((hla2-hla2i)/(timeh[4])*(temp2)); }
		else    {NRPMouseservoarray[arrcount4][6]= hla2i-((hla2i-hla2)/(timeh[4])*(temp2)); }

		if (hra1>hra1i){
			NRPMouseservoarray[arrcount4][7]= hra1i+((hra1-hra1i)/(timeh[4])*(temp2));}
		else 	{NRPMouseservoarray[arrcount4][7]= hra1i-((hra1i-hra1)/(timeh[4])*(temp2));}
		if (hra2>hra2i) {
			NRPMouseservoarray[arrcount4][8]= hra2i+((hra2-hra2i)/(timeh[4])*(temp2));}
		else    {NRPMouseservoarray[arrcount4][8]= hra2i-((hra2i-hra2)/(timeh[4])*(temp2));}

		//NRPMouseservoarray[arrcount4][0]= temp2+timeh[3]+timeh[2]+timeh[1]+timeh[0];
	}
	
	//This part shifts the arrays accoring to the phase difference
	//1) Create a temporary array identical to created
	for (int s=0; s<arrcount2;s++)
	{ 	NRPMouseservoarraytemp[s][0]=NRPMouseservoarray[s][0];
		NRPMouseservoarraytemp[s][1]=NRPMouseservoarray[s][1];
		NRPMouseservoarraytemp[s][2]=NRPMouseservoarray[s][2];
		NRPMouseservoarraytemp[s][3]=NRPMouseservoarray[s][3];
		NRPMouseservoarraytemp[s][4]=NRPMouseservoarray[s][4];
		NRPMouseservoarraytemp[s][5]=NRPMouseservoarray[s][5];
		NRPMouseservoarraytemp[s][6]=NRPMouseservoarray[s][6];
		NRPMouseservoarraytemp[s][7]=NRPMouseservoarray[s][7];
		NRPMouseservoarraytemp[s][8]=NRPMouseservoarray[s][8];
	}
	//2) FInd the initial position in the array where the legs are supposed to start
	initpos=phaseoffset*arrsize;
	loopedpos=arrsize-initpos;

	//3) Replace the array values by a phase diff
	for (int u=initpos, v=0 ;u<arrsize;u++,v++)
	{	NRPMouseservoarray[v][5]=NRPMouseservoarraytemp[u][5]; //hla1
		NRPMouseservoarray[v][6]=NRPMouseservoarraytemp[u][6]; //hla2
	}
	for (int g=0, h=loopedpos;g<initpos;g++,h++)
	{	NRPMouseservoarray[h][5]=NRPMouseservoarraytemp[g][5]; //hla1
		NRPMouseservoarray[h][6]=NRPMouseservoarraytemp[g][6]; //hla2
	}
	for (int u=initpos*2, v=0 ;u<arrsize;u++,v++)
	{	NRPMouseservoarray[v][7]=NRPMouseservoarraytemp[u][7]; //hra1
		NRPMouseservoarray[v][8]=NRPMouseservoarraytemp[u][8]; //hra2
	}
	for (int g=0, h=loopedpos-initpos;g<initpos*2;g++,h++)
	{	NRPMouseservoarray[h][7]=NRPMouseservoarraytemp[g][7]; //hra1
		NRPMouseservoarray[h][8]=NRPMouseservoarraytemp[g][8]; //hra2
	}
	for (int u=initpos*3, v=0 ;u<arrsize;u++,v++)
	{	NRPMouseservoarray[v][3]=NRPMouseservoarraytemp[u][3]; //fra1
		NRPMouseservoarray[v][4]=NRPMouseservoarraytemp[u][4]; //fra2
	}
	for (int g=0, h=loopedpos-initpos*2;g<initpos*3;g++,h++)
	{	NRPMouseservoarray[h][3]=NRPMouseservoarraytemp[g][3]; //fra1
		NRPMouseservoarray[h][4]=NRPMouseservoarraytemp[g][4]; //fra2
	}
	
	//this part is for the spine movement
	//determine how much the spine will actuate input
	//(extreme left, extreme right, difference)=(20, 160, 140)
	sv=0;
	svadd_walk=(100/(arrsize/2));
	for (int f=0; f<(arrsize/2); f++, sv=sv+svadd_walk)
	{	//spine actuates from 40 to 140
		NRPMouseservoarray[f][9]=40+sv;
	}
	sv=0;
	for (int f=(arrsize/2); f<=arrsize; f++, sv=sv+svadd_walk)
	{	
		//spine actuates from 140 to 40
		NRPMouseservoarray[f][9]=140-sv; 
	}	
}


void wiggle()
{
	double sv=0;
	double svadd_wiggle=(140/(arrsize/4));
	double tv=0;
	double tvadd_wiggle=(80/(arrsize/4));
	// For the spine, ( “C”,  “inverted  C”, difference, center)=(50, 130, 80, 90)
	// For the tail, (“inverted  C”, “C”, difference, center)=(140, 40, 100, 90)
	//*****spine
	for (int f=0; f<(arrsize/4); f++, sv=sv+svadd_wiggle)
	{	//spine actuates from 20 to 160
		NRPMouseservoarray[f][9]=20+sv;
	}
	sv=0;
	for (int f=(arrsize/4); f<=arrsize/2; f++, sv=sv+svadd_wiggle)
	{	
		//spine actuates from 160 to 20
		NRPMouseservoarray[f][9]=160-sv; 
	}
	sv=0;
	for (int f=(arrsize/2); f<(3*arrsize/4); f++, sv=sv+svadd_wiggle)
	{	//spine actuates from 20 to 160
		NRPMouseservoarray[f][9]=20+sv;
	}
	sv=0;
	for (int f=(3*arrsize/4); f<=arrsize; f++, sv=sv+(svadd_wiggle/2))
	{	
		//spine actuates from 160 to 90
		NRPMouseservoarray[f][9]=160-sv; 
	}

	//*****tail
	for (int f=0; f<(arrsize/4); f++, tv=tv+tvadd_wiggle)
	{	//tail actuates from 40 to 120
		NRPMouseservoarray[f][10]=40+tv;
	}
	tv=0;
	for (int f=(arrsize/4); f<=arrsize/2; f++, tv=tv+tvadd_wiggle)
	{	
		//tail actuates from 120 to 40
		NRPMouseservoarray[f][10]=120-tv; 
	}
	tv=0;
	for (int f=(arrsize/2); f<(3*arrsize/4); f++, tv=tv+tvadd_wiggle)
	{	//tail actuates from 40 to 120
		NRPMouseservoarray[f][10]=40+tv;
	}
	tv=0;
	for (int f=(3*arrsize/4); f<=arrsize; f++, tv=tv+(tvadd_wiggle/2))
	{	
		//tail actuates from 120 to 80
		NRPMouseservoarray[f][10]=120-tv; 
	}

	//******forelegs
	for (int f=0; f<(arrsize-10); f++)
	{	//shorten forelegs
		NRPMouseservoarray[f][2]=30; 	//73-43
		NRPMouseservoarray[f][4]=150;		//107+43
	}
	
	for(int p=0;p<arrsize;p++)	
	  	{	
	  		// other angles needs to be the same as init pos
			(NRPMouseservoarray[p][0])=0;
			(NRPMouseservoarray[p][1])=108;
			(NRPMouseservoarray[p][3])=76;
			(NRPMouseservoarray[p][5])=139;
			(NRPMouseservoarray[p][6])=110;
			(NRPMouseservoarray[p][7])=40;
			(NRPMouseservoarray[p][8])=70;	
		}
	for (int f=(arrsize-10); f<(arrsize); f++)
	{
		NRPMouseservoarray[arrsize][9]=90; //just to make sure it returns to 90 (straight spine in the end)
		NRPMouseservoarray[arrsize][10]=90; 
		NRPMouseservoarray[arrsize][2]=73; 
		NRPMouseservoarray[arrsize][4]=107;
	}

}

void highfiveleft()
{
	/* 
		dip the hind right leg
		hind left needs to push out
		right leg move back a little to increase height
		tail to the right
		spine to the right
		fore left need to push the button at arr/2
	*/
	//Fore left high five motion
	ikforeleft(-20+fpos_init[0][0],fpos_init[0][1]);
	for(int p=0;p<=(arrsize/4);p++)
	{	
		NRPMouseservoarray[p][1]=fla1;
		NRPMouseservoarray[p][2]=0;
	}
	ikforeleft(-45+fpos_init[0][0],fpos_init[0][1]); //push button
	for(int p=(arrsize/4);p<=(arrsize/2);p++)
	{	
		NRPMouseservoarray[p][1]=fla1;
		NRPMouseservoarray[p][2]=60;
	}
	ikforeleft(-20+fpos_init[0][0],fpos_init[0][1]);
	for(int p=(arrsize/2);p<=(3*arrsize/4);p++)
	{	
		NRPMouseservoarray[p][1]=fla1;
		NRPMouseservoarray[p][2]=0;
	}
	for(int p=(3*arrsize/4);p<=(arrsize);p++)
	{	
		NRPMouseservoarray[p][1]=108;
		NRPMouseservoarray[p][2]=73;
	}

	//fore right move back a little to increase height
	//hindleft to raise
	//hind right to dip
	//tail to the right
	//spine to the right
	for(int p=0;p<=(3*arrsize/4);p++)
	{	
		NRPMouseservoarray[p][3]=90;
		NRPMouseservoarray[p][4]=120;
		NRPMouseservoarray[p][5]=130;
		NRPMouseservoarray[p][6]=110;
		NRPMouseservoarray[p][7]=10;
		NRPMouseservoarray[p][8]=70;
		NRPMouseservoarray[p][9]=40;
		NRPMouseservoarray[p][10]=40;
	}
	for(int p=(3*arrsize/4);p<=(arrsize);p++)
	{	
		NRPMouseservoarray[p][3]=76;
		NRPMouseservoarray[p][4]=108;
		NRPMouseservoarray[p][5]=139;
		NRPMouseservoarray[p][6]=110;
		NRPMouseservoarray[p][7]=40;
		NRPMouseservoarray[p][8]=70;
		NRPMouseservoarray[p][9]=90;
		NRPMouseservoarray[p][10]=90;
	}

	for (int f=(arrsize-10); f<(arrsize); f++)
	{
		NRPMouseservoarray[arrsize][0]=0;
		NRPMouseservoarray[arrsize][1]=108;
		NRPMouseservoarray[arrsize][2]=73;
		NRPMouseservoarray[arrsize][3]=76;
		NRPMouseservoarray[arrsize][4]=107;
		NRPMouseservoarray[arrsize][5]=139;
		NRPMouseservoarray[arrsize][6]=110;
		NRPMouseservoarray[arrsize][7]=40;
		NRPMouseservoarray[arrsize][8]=70;
		NRPMouseservoarray[arrsize][9]=90; //just to make sure it returns to 90 (straight spine in the end)
		NRPMouseservoarray[arrsize][10]=80;		
	}
}


//****************************************************************************************************************************************************************************************
//Inverse kinematic functions
void ikforeleft(double fx,double fy)
{
    //ik of the fore leg, returns the servoangles required for IK
    //theta3 is assumed to be a constant to prevent complex calculations
    // Assuming the kinematics only, no ground contact forces
    double ft1;
    double ft2;
    double flb;
    double ftb;
    fx=fx-fpos_init[0][0];         //origin position of the leg (easier for trajectory planning)(relative to center point of trajectory circle)
    fy=fy-fpos_init[0][1];
    flb=sqrt(pow(fl3,2)+pow(fl2,2)-(2*fl2*fl3*cos(ft3i*pi/180)));
    ftb=acos(((pow(fx,2)+pow(fy,2))-(pow(fl1,2)+pow(flb,2)))/(-2*fl1*flb));
    ft2=ftb+asin(fl3*sin(ft3i*pi/180)/flb);
    if (fx<0)
        {ft1=pi-atan(fy/fx)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}
    else if (fx>0)
            {ft1=-atan(fy/fx)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}
    else if (fx==0)
               {ft1=(90*pi/180)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}
    ft1=ft1*180/pi;
    ft2=ft2*180/pi;
    fla1=forelefthipservoangle(ft1);
    fla2=foreleftkneeservoangle(ft1,ft2);
}

void ikforeright(double fx,double fy)
{
    //ik of the fore leg, returns the servoangles required for IK
    //theta3 is assumed to be a constant to prevent complex calculations
    // Assuming the kinematics only, no ground contact forces
    double ft1;
    double ft2;
    double flb;
    double ftb;
    fx=fx-fpos_init[0][0];         //origin position of the leg (easier for trajectory planning)(relative to center point of trajectory circle)
    fy=fy-fpos_init[0][1];
    flb=sqrt(pow(fl3,2)+pow(fl2,2)-(2*fl2*fl3*cos(ft3i*pi/180)));
    ftb=acos(((pow(fx,2)+pow(fy,2))-(pow(fl1,2)+pow(flb,2)))/(-2*fl1*flb));
    ft2=ftb+asin(fl3*sin(ft3i*pi/180)/flb);
    if (fx<0)
        {ft1=pi-atan(fy/fx)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}
    else if (fx>0)
            {ft1=-atan(fy/fx)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}
    else if (fx==0)
               {ft1=(90*pi/180)-asin(flb*sin(ftb)/sqrt(pow(fx,2)+pow(fy,2)));}
    ft1=ft1*180/pi;
    ft2=ft2*180/pi;
    fra1=forerighthipservoangle(ft1);
    fra2=forerightkneeservoangle(ft1,ft2);
}

void ikhindleft(double hx,double hy)
{
    //ik of the hind leg, returns the servoangles required for IK
    //theta3 is assumed to be a constant to prevent complex calculations
    //The string will be routed on the tibia for now.
    //Assuming the kinematics only, no ground contact forces
    double ht1;
    double ht2;
    double hlb;
    double htb;
    hx=hx-hpos_init[0][0];        //origin position of the leg (easier for trajectory planning)(relative to center point of trajectory circle)
    hy=hy-hpos_init[0][1];
    hlb=sqrt(pow(hl3,2)+pow(hl2,2)-(2*hl2*hl3*cos(ht3i*pi/180)));
    htb=acos(((pow(hx,2)+pow(hy,2))-(pow(hl1,2)+pow(hlb,2)))/(-2*hl1*hlb));
    ht2=htb-asin(hl3*sin(ht3i*pi/180)/hlb);
    if (hx>0)
          {ht1=pi+atan(hy/hx)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));}
    else if (hx<0)
            {ht1=atan(hy/hx)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));}
    else if (hx==0)
            {ht1=(90*pi/180)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));}
    ht1=ht1*180/pi;
    ht2=ht2*180/pi;
    hla1=hindlefthipservoangle(ht1);
    hla2=hindleftkneeservoangle(ht1,ht2);
}
void ikhindright(double hx,double hy)
{
    //ik of the hind leg, returns the servoangles required for IK
    //theta3 is assumed to be a constant to prevent complex calculations
    //The string will be routed on the tibia for now.
    //Assuming the kinematics only, no ground contact forces
    double ht1;
    double ht2;
    double hlb;
    double htb;
    hx=hx-hpos_init[0][0];        //origin position of the leg (easier for trajectory planning)(relative to center point of trajectory circle)
    hy=hy-hpos_init[0][1];
    hlb=sqrt(pow(hl3,2)+pow(hl2,2)-(2*hl2*hl3*cos(ht3i*pi/180)));
    htb=acos(((pow(hx,2)+pow(hy,2))-(pow(hl1,2)+pow(hlb,2)))/(-2*hl1*hlb));
    ht2=htb-asin(hl3*sin(ht3i*pi/180)/hlb);
    if (hx>0)
          {ht1=pi+atan(hy/hx)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));}
    else if (hx<0)
            {ht1=atan(hy/hx)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));}
    else if (hx==0)
            {ht1=(90*pi/180)-asin(hlb*sin(htb)/sqrt(pow(hx,2)+pow(hy,2)));}
    ht1=ht1*180/pi;
    ht2=ht2*180/pi;
    hra1=hindrighthipservoangle(ht1);
    hra2=hindrightkneeservoangle(ht1,ht2);
}

//supporting functions
//*********************************************************************************************************************************************************************************************************
double foreleftkneeservoangle(double ft1,double ft2)
{
    //calculates the servo angle for the knee servo for foreleg
    //hip pulley is 8mm diameter, 1mm string diameter sweep, 3.5mm effective radius
    //distance from string attachment point to knee geometric center=13.914mm
    //initial string length=41.198mm (geometric length)(From Solidworks)
    double lknee;
    double fls;
    double centerdist;
    double lhip;
    double flsi=48.332; 											//flsi=41.198+(fahip*pi/180*frhip); //initial calculated length
    double fltibia=13.914;      										//string attachment point on tibia
    double frhip=3.5;          										//effective radius of the pulley
    double fahip=116.79;       										//initial angle of string routed on the hip
    double fksainit=80; 
    if (ft1>ft1i)         											//when leg is moved forward
        {lhip=(fahip-(ft1-ft1i))*pi/180*3.7;} 						//the string length at the hip pulley,
    else    														//when leg is moved backward
        {lhip=(fahip+(ft1i-ft1))*pi/180*3.7;} 
    centerdist=sqrt(pow(fl1,2)+pow(fltibia,2)-(2*fl1*fltibia*cos(ft2*pi/180)));
    lknee=sqrt(pow(centerdist,2)-pow(frhip,2));
    fls=lhip+lknee;     											//total calculated length
    if (fls>flsi)        											//need to release string, turn cw (+)
        {return fksainit+(kneeservoangle(fls-flsi))*180/pi;}
    else                  											//need to tension string 
        {return fksainit-(kneeservoangle(flsi-fls))*180/pi;} 
}
double forerightkneeservoangle(double ft1,double ft2)
{
    //calculates the servo angle for the knee servo for foreleg
    //hip pulley is 8mm diameter, 1mm string diameter sweep, 3.5mm effective radius
    //distance from string attachment point to knee geometric center=13.914mm
    //initial string length=41.198mm (geometric length)(From Solidworks)
    double lknee;
    double fls;
    double centerdist;
    double lhip;
    double flsi=48.332; 											//flsi=41.198+(fahip*pi/180*frhip); //initial calculated length
    double fltibia=13.914;      										//string attachment point on tibia
    double frhip=3.5;         										//effective radius of the pulley
    double fahip=116.79;      										//initial angle of string routed on the hip
    double fksainit=100; 
    if (ft1>ft1i)         											//move leg forward
        {lhip=(fahip-(ft1-ft1i))*pi/180*3.7;} 						//the string length at the hip pulley,
    else    														//move leg backward
        {lhip=(fahip+(ft1i-ft1))*pi/180*3.7;} 
    centerdist=sqrt(pow(fl1,2)+pow(fltibia,2)-(2*fl1*fltibia*cos(ft2*pi/180)));
    lknee=sqrt(pow(centerdist,2)-pow(frhip,2));
    fls=lhip+lknee;     											//total calculated length
    if (fls>flsi)         											//need to release string, turn anticw(-)
        {return fksainit-(kneeservoangle(fls-flsi))*180/pi;}
    else                  											//need to tension string, turn cw (+)
        {return fksainit+(kneeservoangle(flsi-fls))*180/pi;} 
}
double hindleftkneeservoangle(double ht1,double ht2)
{
    //calculates the servo angle for the knee servo for hingleg
    //hip pulley is 8mm diameter, 1mm string diameter sweep, 3.5mm effective radius
    //distance from string attachment point to knee geometric center=16.037mm
    //initial string length=37.687mm (geometric length)(From Solidworks)
    double lknee;
    double hls;
    double centerdist;
    double lhip;
    double hlsi=47.477;      										//hlsi=37.687+(hahip*pi/180*hrhip); //initial total length 
    double hltibia=16.037;     										//string attachment point on tibia
    double hrhip=3.5;          										//effective radius of the pulley
    double hahip=160.26;       										//initial angle of string routed on the hip
    double hksainit=40; 
    if (ht1>ht1i)           										//when leg is moved backward
        {lhip=(hahip-(ht1-ht1i))*pi/180*3.7;} 						//the string length at the hip pulley,
    else                    										//when leg is moved forward
        {lhip=(hahip+(ht1i-ht1))*pi/180*3.7;}
    centerdist=sqrt(pow(hl1,2)+pow(hltibia,2)-(2*hl1*hltibia*cos(ht2*pi/180)));
    lknee=sqrt(pow(centerdist,2)-pow(hrhip,2));
    
    hls=lhip+lknee;     											//total calculated length
    if (hls>hlsi)         											//need to release string, turn anticw (-)
        {return hksainit-(kneeservoangle(hls-hlsi))*180/pi;}
    else                											//need to tension string
        {return hksainit+(kneeservoangle(hlsi-hls))*180/pi;} 		//returns the final servo angle value in (degrees)
}
double hindrightkneeservoangle(double ht1,double ht2)
{
    //calculates the servo angle for the knee servo for hingleg
    //hip pulley is 8mm diameter, 1mm string diameter sweep, 3.5mm effective radius
    //distance from string attachment point to knee geometric center=16.037mm
    //initial string length=37.687mm (geometric length)(From Solidworks)
    double lknee;
    double hls;
    double centerdist;
    double lhip;
    double hlsi=47.477;      										//hlsi=37.687+(hahip*pi/180*hrhip); //initial total length 
    double hltibia=16.037;     										//string attachment point on tibia
    double hrhip=3.5;          										//effective radius of the pulley
    double hahip=160.26;       										//initial angle of string routed on the hip
    double hksainit=140; 
    if (ht1>ht1i)           										//when leg is moved backward
        {lhip=(hahip-(ht1-ht1i))*pi/180*3.7;} 						//the string length at the hip pulley,
    else                    										//when leg is moved forward
        {lhip=(hahip+(ht1i-ht1))*pi/180*3.7;}
    centerdist=sqrt(pow(hl1,2)+pow(hltibia,2)-(2*hl1*hltibia*cos(ht2*pi/180)));
    lknee=sqrt(pow(centerdist,2)-pow(hrhip,2));
    
    hls=lhip+lknee;     											//total calculated length
    if (hls>hlsi)         											//need to release string, turn cw(+)
        {return hksainit+(kneeservoangle(hls-hlsi))*180/pi;}
    else                											//need to tension string
        {return hksainit-(kneeservoangle(hlsi-hls))*180/pi;} 		//returns the final servo angle value in (degrees)
}
double forelefthipservoangle(double ft1)
{
    //calculates the servoangle for hip servo for foreleg
    double ft1i=53; 	
    double fhsainit=90;	//like the midpointposition of the servo
    double floffset=5;	//offset for calibration of servo position (manufacturing/assembly problem from the servo)
    if (ft1>ft1i)       //move leg forward, turn cw (+)
        {return fhsainit+(ft1-ft1i)+floffset;}
    else                //move leg back
        {return fhsainit-(ft1i-ft1)+floffset;}
}
double forerighthipservoangle(double ft1)
{
    //calculates the servoangle for hip servo for foreleg
    double ft1i=53;		
    double fhsainit=90;	//like the midpointposition of the servo
    if (ft1>ft1i)       //move leg forward, turn anticw (-)
        {return fhsainit-(ft1-ft1i);}
    else                //move leg back,
        {return fhsainit+(ft1i-ft1);}
}
double hindlefthipservoangle(double ht1)
{
    //calculates the servoangle for hip servo for hindleg
    double ht1i=8;		
    double hhsainit=150;//like the midpointposition of the servo
    if (ht1>ht1i)       //move leg back,  turn anti cw (-)
        {return hhsainit-(ht1-ht1i);}
    else                //move leg forward
        {return hhsainit+(ht1i-ht1);}
}
double hindrighthipservoangle(double ht1)
{
    //calculates the servoangle for hip servo for hindleg
    double ht1i=8;		
    double hhsainit=30;//like the midpointposition of the servo
    if (ht1>ht1i)       //move leg back,  turn cw (+)
        {return hhsainit+(ht1-ht1i);}
    else                //move leg forward
        {return hhsainit-(ht1i-ht1);}
}

double kneeservoangle(double l)
{
    //calculates the amount of angle required for the pulley to wind a length of string
    double rp1=6.75;           //radius of the pulley 
    return l/rp1;          //in radians
}
