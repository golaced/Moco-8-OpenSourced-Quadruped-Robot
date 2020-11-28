#include "head.h"
#include "table.h"
#include "spi.h"
#include "nrf.h"
#include "adc.h"
#include "key.h"
#include "mpu9250.h"
#include "FLASH.h"
#include "imu.h"
#include "mymath.h"
#include "math.h"
#include "rc_mine.h"
#include "beep.h"
#include "gui_basic.h"
#include "hw_config.h"
#include "data_transfer.h"
#define USE_AS_MONITER 0
MODULE module;
u8 CHANNAL=44;
float time_fly[5]={0,0,0,0,2.68};
int moniter_sel=0;
int sub_sel[3]={0};
int ts[4];
void OLED_TASK(float dt)//状态界面0
{
	 static int cx,cy;
	 char temp[10][20]={'\0'};
   //GUI_RectangleFill(42,1,123,55,0);
   GUI_RectangleFill(1,1,123,55,0);
   GUI_Rectangle(42,1,123,55,1);
 
   if(plane.module.gps)
		GUI_RectangleFill(115,40,120,45,1);
   else
		GUI_Rectangle(115,40,120,45,1); 
	 
	  if(plane.module.flow)
		GUI_RectangleFill(115,38,120,33,1);
   else
		GUI_Rectangle(115,38,120,33,1); 
	 
	  if(plane.module.vision)
		GUI_RectangleFill(115,31,120,26,1);
   else
		GUI_Rectangle(115,31,120,26,1); 
   #if USE_AS_MONITER
   if(DEBUG[6]!=0&&DEBUG[7]!=0){
   cx=LIMIT(80+(DEBUG[6]-120)*0.35,42+3,123-3);
   cy=LIMIT(28+(DEBUG[7]-160)*0.168,1+3,55-3);
	 GUI_CircleFill(cx,cy,2,1);	 
	 }
	 else
	 GUI_CircleFill(cx,cy,2,0);	 
   #else
   cx=LIMIT(80+(Rc_Get.ROL-1500)*0.1,42+3,123-3);
   cy=LIMIT(28+(Rc_Get.PIT-1500)*0.1,1+3,55-3);
	 GUI_CircleFill(cx,cy,3,1);
   #endif
   GUI_Circle(80,28,5,1);   
   //
   OLED_Refresh_Gram(); 
   delay_us(100);
   //
   OLED_P6x8Str(3,0,"P:");
   my_itoa(Rc_Get.PIT,temp[0]);
	 OLED_P6x8Str(4*3,0,temp[0]);

   OLED_P6x8Str(3,1,"R:");
   my_itoa(Rc_Get.ROL,temp[1]);
	 OLED_P6x8Str(4*3,1,temp[1]);

   OLED_P6x8Str(3,2,"T:");
   my_itoa(Rc_Get.THROTTLE,temp[2]);
	 OLED_P6x8Str(4*3,2,temp[2]);
    delay_us(100);
   OLED_P6x8Str(3,3,"Y:");
   my_itoa(Rc_Get.YA,temp[3]);
	 OLED_P6x8Str(4*3,3,temp[3]);
   //GPS
	 switch(plane.pos_sensor_state)
	 {
	 case 1://flow	 
   OLED_P6x8Str(45,0,"Opt:");
   my_itoa(plane.gps_sv,temp[0]);
	 OLED_P6x8Str(45+5*5,0,temp[0]);
	 break;
	 case 2://qr
	 OLED_P6x8Str(45,0,"QR :");
   my_itoa(plane.gps_sv,temp[0]);
	 OLED_P6x8Str(45+5*5,0,temp[0]);
	 break;
	 case 3://gps	 
   OLED_P6x8Str(45,0,"GPS:");
   my_itoa(plane.gps_sv,temp[0]);
	 OLED_P6x8Str(45+5*5,0,temp[0]);
	 break;
	 default:
	 OLED_P6x8Str(45,0,"Bad Pose");
	 break;
	 
	 }
	   delay_us(100);
  //lock
   if(!plane.lock)
   OLED_P6x8Str(90,0,"*lock*");
   else
	 OLED_P6x8Str(90,0,"*fly!*");
   //SSR
	   delay_us(100);
   OLED_P6x8Str(3,4,"SR:");
   my_itoa(plane.rssi,temp[3]);
	 OLED_P6x8Str(4*5,4,temp[3]);
   OLED_P6x8Str(4*(5+3),4,"%");
   //BAT_RC
	   delay_us(100);
   OLED_P6x8Str(3,5,"BR:");
   my_itoa(adc_rc.bat_percent,temp[4]);
	 OLED_P6x8Str(4*5,5,temp[4]);
   OLED_P6x8Str(4*(5+3),5,"%");
	   delay_us(100);
   //BAT_FLY
   OLED_P6x8Str(3,6,"BF:");
   my_itoa(plane.bat,temp[5]);
	 OLED_P6x8Str(4*5,6,temp[5]);
	 OLED_P6x8Str(4*(5+3),6,"%");
	   delay_us(100);
	 //MODE
   OLED_P6x8Str(3,7,"M:");
   if(plane.mode==1)
		 OLED_P6x8Str(4*3,7,"Munl ");
	 else if(plane.mode==2)
		 OLED_P6x8Str(4*3,7,"Alt B");
	 else if(plane.mode==4)
		 OLED_P6x8Str(4*3,7,"Alt S");
	 else if(plane.mode==3||plane.mode==5)
		 OLED_P6x8Str(4*3,7,"Pos  ");
	   
	 else if(pos_mode==2)
		 OLED_P6x8Str(4*3,7,"Smart");
	 
	 //att
	  OLED_P6x8Str(45,1,"P:");
	  my_itoa(plane.att[1],temp[6]);
	  OLED_P6x8Str(45+3*3,1,temp[6]);
	 
	  OLED_P6x8Str(45+3*8,1,"R:");
	  my_itoa(plane.att[0],temp[7]);
	  OLED_P6x8Str(45+3*8+3*3,1,temp[7]);
	 
	  OLED_P6x8Str(45+3*8*2,1,"Y:");
	  my_itoa(plane.att[2],temp[8]);
	  OLED_P6x8Str(45+3*8*2+3*3,1,temp[8]);
	 
	  OLED_P6x8Str(45,7,"X:");
	  my_itoa(plane.pos[0],temp[6]);
	  OLED_P6x8Str(45+3*3,7,temp[6]);
	 
	  OLED_P6x8Str(45+3*8,7,"Y:");
	  my_itoa(plane.pos[1],temp[6]);
	  OLED_P6x8Str(45+3*8+3*3,7,temp[6]);
		
		OLED_P6x8Str(45+3*8*2,7,"Z:");
	  my_itoa(plane.pos[2],temp[6]);
	  OLED_P6x8Str(45+3*8*2+3*3,7,temp[6]);
		
		//fly time
	  my_itoa(time_fly[0],temp[6]);
	  OLED_P6x8Str(45,6,temp[6]);
		OLED_P6x8Str(45+3*3+3,6,":");
		
	  my_itoa(time_fly[1],temp[6]);
	  OLED_P6x8Str(45+3*3+3*3,6,temp[6]);
}

float set[10]=0;
void OLED_TASK_SUB_SEL(float dt)
{
	 static int cx,cy;
   cx=sub_sel[0]*66+set[1];
   cy=sub_sel[1]*1+set[3];
	 if(sub_sel[2])
	 OLED_P6x8Str(cx,cy,"*");	 
	 else
	 OLED_P6x8Str(cx,cy,"-");
}

void OLED_TASK_PID1(float dt)//参数界面1
{
	 static int cx,cy;
	 char temp[10][40]={'\0'};
   OLED_Fill(0x00);
   OLED_P6x8Str(3+4,0,"P01:");
   my_itoa(plane.PID_RX[0][0],temp[0]);
	 OLED_P6x8Str(36,0,temp[0]);
   OLED_P6x8Str(3+4,1,"I01:");
   my_itoa(plane.PID_RX[0][1],temp[1]);
	 OLED_P6x8Str(36,1,temp[1]);
   OLED_P6x8Str(3+4,2,"D01:");
   my_itoa(plane.PID_RX[0][2],temp[2]);
	 OLED_P6x8Str(36,2,temp[2]);


   OLED_P6x8Str(3+4,4,"P02:");
   my_itoa(plane.PID_RX[1][0],temp[0]);
	 OLED_P6x8Str(36,4,temp[0]);
   OLED_P6x8Str(3+4,5,"I02:");
   my_itoa(plane.PID_RX[1][1],temp[1]);
	 OLED_P6x8Str(36,5,temp[1]);
   OLED_P6x8Str(3+4,6,"D02:");
   my_itoa(plane.PID_RX[1][2],temp[2]);
	 OLED_P6x8Str(36,6,temp[2]);
	 
	 OLED_P6x8Str(3+4+66,0,"P03:");
   my_itoa(plane.PID_RX[2][0],temp[0]);
	 OLED_P6x8Str(36+66,0,temp[0]);
   OLED_P6x8Str(3+4+66,1,"I03:");
   my_itoa(plane.PID_RX[2][1],temp[1]);
	 OLED_P6x8Str(36+66,1,temp[1]);
   OLED_P6x8Str(3+4+66,2,"D03:");
   my_itoa(plane.PID_RX[2][2],temp[2]);
	 OLED_P6x8Str(36+66,2,temp[2]);
	 
	 OLED_P6x8Str(3+4+66,4,"P04:");
   my_itoa(plane.PID_RX[3][0],temp[0]);
	 OLED_P6x8Str(36+66,4,temp[0]);
   OLED_P6x8Str(3+4+66,5,"I04:");
   my_itoa(plane.PID_RX[3][1],temp[1]);
	 OLED_P6x8Str(36+66,5,temp[1]);
   OLED_P6x8Str(3+4+66,6,"D04:");
   my_itoa(plane.PID_RX[3][2],temp[2]);
	 OLED_P6x8Str(36+66,6,temp[2]);
}

void OLED_TASK_PID2(float dt)//参数界面1
{
	 static int cx,cy;
	 char sel=0;
	 char temp[10][40]={'\0'};
   OLED_Fill(0x00);
   sel=4;
   OLED_P6x8Str(3+4,0,"P05:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36,0,temp[0]);
   OLED_P6x8Str(3+4,1,"I05:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36,1,temp[1]);
   OLED_P6x8Str(3+4,2,"D05:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36,2,temp[2]);

   sel=5;
   OLED_P6x8Str(3+4,4,"P06:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36,4,temp[0]);
   OLED_P6x8Str(3+4,5,"I06:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36,5,temp[1]);
   OLED_P6x8Str(3+4,6,"D06:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36,6,temp[2]);
	 
	 sel=6;
	 OLED_P6x8Str(3+4+66,0,"P07:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36+66,0,temp[0]);
   OLED_P6x8Str(3+4+66,1,"I07:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36+66,1,temp[1]);
   OLED_P6x8Str(3+4+66,2,"D07:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36+66,2,temp[2]);
	 
	 sel=7;
	 OLED_P6x8Str(3+4+66,4,"P08:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36+66,4,temp[0]);
   OLED_P6x8Str(3+4+66,5,"I08:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36+66,5,temp[1]);
   OLED_P6x8Str(3+4+66,6,"D08:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36+66,6,temp[2]);
}


void OLED_TASK_PID3(float dt)//参数界面1
{
	 static int cx,cy;
	 char sel=0;
	 char temp[10][40]={'\0'};
   OLED_Fill(0x00);
   sel=8;
   OLED_P6x8Str(3+4,0,"P09:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36,0,temp[0]);
   OLED_P6x8Str(3+4,1,"I09:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36,1,temp[1]);
   OLED_P6x8Str(3+4,2,"D09:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36,2,temp[2]);

   sel=9;
   OLED_P6x8Str(3+4,4,"P10:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36,4,temp[0]);
   OLED_P6x8Str(3+4,5,"I10:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36,5,temp[1]);
   OLED_P6x8Str(3+4,6,"D10:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36,6,temp[2]);
	 
	 sel=10;
	 OLED_P6x8Str(3+4+66,0,"P11:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36+66,0,temp[0]);
   OLED_P6x8Str(3+4+66,1,"I11:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36+66,1,temp[1]);
   OLED_P6x8Str(3+4+66,2,"D11:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36+66,2,temp[2]);
	 
	 sel=11;
	 OLED_P6x8Str(3+4+66,4,"P12:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36+66,4,temp[0]);
   OLED_P6x8Str(3+4+66,5,"I12:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36+66,5,temp[1]);
   OLED_P6x8Str(3+4+66,6,"D12:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36+66,6,temp[2]);
}


void OLED_TASK_PID4(float dt)//参数界面1
{
	 static int cx,cy;
	 char sel=0;
	 char temp[10][40]={'\0'};
   OLED_Fill(0x00);
   sel=12;
   OLED_P6x8Str(3+4,0,"P13:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36,0,temp[0]);
   OLED_P6x8Str(3+4,1,"I13:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36,1,temp[1]);
   OLED_P6x8Str(3+4,2,"D13:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36,2,temp[2]);

   sel=13;
   OLED_P6x8Str(3+4,4,"P14:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36,4,temp[0]);
   OLED_P6x8Str(3+4,5,"I14:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36,5,temp[1]);
   OLED_P6x8Str(3+4,6,"D14:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36,6,temp[2]);
	 
	 sel=14;
	 OLED_P6x8Str(3+4+66,0,"P15:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36+66,0,temp[0]);
   OLED_P6x8Str(3+4+66,1,"I15:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36+66,1,temp[1]);
   OLED_P6x8Str(3+4+66,2,"D15:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36+66,2,temp[2]);
	 
	 sel=15;
	 OLED_P6x8Str(3+4+66,4,"P16:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36+66,4,temp[0]);
   OLED_P6x8Str(3+4+66,5,"I16:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36+66,5,temp[1]);
   OLED_P6x8Str(3+4+66,6,"D16:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36+66,6,temp[2]);
}



void OLED_TASK_PID5(float dt)//参数界面1
{
	 static int cx,cy;
	 char sel=0;
	 char temp[10][40]={'\0'};
   OLED_Fill(0x00);
   sel=16;
   OLED_P6x8Str(3+4,0,"P17:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36,0,temp[0]);
   OLED_P6x8Str(3+4,1,"I17:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36,1,temp[1]);
   OLED_P6x8Str(3+4,2,"D17:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36,2,temp[2]);

   sel=17;
   OLED_P6x8Str(3+4,4,"P18:");
   my_itoa(plane.PID_RX[sel][0],temp[0]);
	 OLED_P6x8Str(36,4,temp[0]);
   OLED_P6x8Str(3+4,5,"I18:");
   my_itoa(plane.PID_RX[sel][1],temp[1]);
	 OLED_P6x8Str(36,5,temp[1]);
   OLED_P6x8Str(3+4,6,"D18:");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36,6,temp[2]);
	 
// 	 sel=18;
// 	 OLED_P6x8Str(3+3+66,0,"P70:");
//    my_itoa(plane.PID_RX[sel][0],temp[0]);
// 	 OLED_P6x8Str(36+66,0,temp[0]);
//    OLED_P6x8Str(3+3+66,1,"P71:");
//    my_itoa(plane.PID_RX[sel][1],temp[1]);
// 	 OLED_P6x8Str(36+66,1,temp[1]);
//    OLED_P6x8Str(3+3+66,2,"P72:");
//    my_itoa(plane.PID_RX[sel][2],temp[2]);
// 	 OLED_P6x8Str(36+66,2,temp[2]);
// 	 
	 sel=19;
// 	 OLED_P6x8Str(3+3+66,4,"P80:");
//    my_itoa(plane.PID_RX[sel][0],temp[0]);
// 	 OLED_P6x8Str(36+66,4,temp[0]);
//    OLED_P6x8Str(3+3+66,5,"P81:");
//    my_itoa(plane.PID_RX[sel][1],temp[1]);
// 	 OLED_P6x8Str(36+66,5,temp[1]);
   OLED_P6x8Str(3+3+66,6,"CH :");
   my_itoa(plane.PID_RX[sel][2],temp[2]);
	 OLED_P6x8Str(36+66,6,temp[2]);
}


char mode_rc[3];
int set_rc[3];
void OLED_TASK_MISSION(float dt)//任务界面
{
	int dis,yaw_d[2];
	char temp[10][40]={'\0'},i;
  float draw_err[3];
  float temp_err[3];
  float scale[3];
	float cy,sy;
  float err[3];
  //OLED_Fill(0x00);
		mode_rc[0]=plane.mission.smarts/100;  
		mode_rc[1]=plane.mission.smarts/10-mode_rc[0]*100; 
		mode_rc[2]=plane.mission.smarts-mode_rc[0]*100-mode_rc[1]*10; 
   GUI_RectangleFill(1,1,123,55,0);
	 GUI_Rectangle(3,3,20,46,1);
	 GUI_Rectangle(22,3,123,46,1);

	 if(plane.module.gps)
		GUI_RectangleFill(115,40,120,45,1);
   else
		GUI_Rectangle(115,40,120,45,1); 
	 
	  if(plane.module.flow)
		GUI_RectangleFill(115,38,120,33,1);
   else
		GUI_Rectangle(115,38,120,33,1); 
	 
	  if(plane.module.vision)
		GUI_RectangleFill(115,31,120,26,1);
   else
		GUI_Rectangle(115,31,120,26,1); 
	 
	//mission
	 if(mode_rc[1]==1)//draw spd
	 {
		draw_err[0]=LIMIT(plane.mission.t_spd[0],-1,1)/1*(123-22)/2;
		draw_err[1]=LIMIT(plane.mission.t_spd[1],-1,1)/1*(46-3)/2;
		
    GUI_Circle(70,25,3,1);//now  
    GUI_RectangleFill(70-2, 25, 70+2, 25+draw_err[1], 1);//y
    GUI_RectangleFill(70, 25+2, 70+draw_err[0], 25-2, 1);//x 
	 }else//draw pos
	 {
		err[0]=plane.mission.t_pos[0]-plane.pos[0];
		err[1]=plane.mission.t_pos[1]-plane.pos[1];
		 
		for(i=0;i<2;i++){ 
		if(ABS(err[i])<5)		
		  scale[i]=5; 
		else if(ABS(err[i])<10)		
		  scale[i]=10; 
		else if(ABS(err[i])<20)		
		  scale[i]=20;
	  else if(ABS(err[i])<40)		
		  scale[i]=40;
    else 	
		  scale[i]=40; 		
	 }	 	

		cy=cos(plane.att[2]*0.0173);	
		sy=sin(plane.att[2]*0.0173);	
		temp_err[0]= err[0]*cy - err[1]*sy;
		temp_err[1]= err[0]*sy + err[1]*cy;
		draw_err[0]=LIMIT(err[0],-scale[0],scale[0])/scale[0]*(123-24)/2;
		draw_err[1]=LIMIT(err[1],-scale[1],scale[1])/scale[1]*(46-3)/2;
    yaw_d[0]=8*sin(plane.att[2]*0.0173);
	  yaw_d[1]=8*cos(plane.att[2]*0.0173);
	  //draw gui
    GUI_Circle(70,25,2,1);//now 
    GUI_Line(70,25,70+yaw_d[0],25+yaw_d[1],1);//yaw
    GUI_CircleFill(70+draw_err[0],25+draw_err[1],2,1);//now  
	  GUI_Line(70,25,70+draw_err[0],25+draw_err[1],1);
	 }

	 
	 if(mode_rc[2]==2)//draw height
	 {
		err[2]=plane.mission.t_pos[2]-plane.pos[2];
		 
		if(ABS(err[2])<2)		
		  scale[2]=2; 
		else if(ABS(err[2])<6)		
		  scale[2]=6; 
		else if(ABS(err[2])<12)		
		  scale[2]=12;
	  else if(ABS(err[2])<30)		
		  scale[2]=30;
    else 	
		  scale[2]=30; 			
		draw_err[2]=LIMIT(err[2],-scale[2],scale[2])/scale[2]*(43)/2;
		
		
    GUI_Circle(12,24,3,1);//now  
    GUI_Line(3, 24+draw_err[2], 20, 24+draw_err[2], 1);//set
	 }else if(mode_rc[2]==1)//draw height
	 {
		draw_err[2]=LIMIT(plane.mission.t_spd[2],-1,1)/1*(43)/2;
    GUI_Circle(12,24,3,1);//now  
		GUI_RectangleFill(3, 24, 20, 24+draw_err[2], 1);
	 }
		//draw gui
	GUI_Line(3,24+10,5,24+10,1);
	GUI_Line(3,24-10,5,24-10,1);
  GUI_Line(18,24+10,20,24+10,1);
	GUI_Line(18,24-10,20,24-10,1); 	 
  OLED_Refresh_Gram(); 
	//height scale
	if(mode_rc[2]==2)//draw height
	{ 
  my_itoa(scale[2],temp[0]);
  OLED_P6x8Str(25,3,temp[0]);
	OLED_P6x8Str(25+12,3,"m");
  }else if(mode_rc[2]==1)//draw height spd
	{ 
  my_itoa(plane.mission.t_spd[2]*10,temp[0]);
  OLED_P6x8Str(25,3,temp[0]);
	OLED_P6x8Str(25+12,3,"cm/s");
  }
	
  if(mode_rc[2]==1)//draw pos spd
	{ 
	my_itoa(plane.mission.t_spd[0]*10,temp[0]);
  OLED_P6x8Str(100,4,temp[0]);
	OLED_P6x8Str(100+12,4,"cm/s");
	my_itoa(plane.mission.t_spd[1]*10,temp[0]);
  OLED_P6x8Str(60,3,temp[0]);
	OLED_P6x8Str(60+12,3,"cm");
  }
	else	//pos scale
	{ 
  my_itoa(scale[0],temp[0]);
  OLED_P6x8Str(100-12,4,temp[0]);
	OLED_P6x8Str(100,4,"m");
		
	my_itoa(scale[1],temp[0]);
  OLED_P6x8Str(60,3,temp[0]);
	OLED_P6x8Str(60+12,3,"m");
		
	OLED_P6x8Str(60+22,7,"Dis:");	
	dis=my_sqrt(err[0]*err[0]+err[1]*err[1]);
	my_itoa(dis,temp[0]);
  OLED_P6x8Str(60+12+12+22,7,temp[0]);
	OLED_P6x8Str(60+12+12+12+22,7,"m");	
  } 
	// draw word

  OLED_P6x8Str(3,0,"Main:");
	switch(plane.mission.mains){
	case 66:OLED_P6x8Str(36,0,"Miss");break;
  case 22:OLED_P6x8Str(36,0,"Safe");break;
  case 0:OLED_P6x8Str(36,0,"Idle");break;			
  default:my_itoa(plane.mission.mains,temp[0]);
  OLED_P6x8Str(36,0,temp[0]);
	break;
  }
  OLED_P6x8Str(3,1,"SubS:");
  my_itoa(plane.mission.subs,temp[0]);
  OLED_P6x8Str(36,1,temp[0]);

 

   OLED_P6x8Str(3+3+55+10,0,"RC:");
   if(mode_rc[0]==1)
		 OLED_P6x8Str(36+55+5,0,"A");
	 
	  if(mode_rc[1]==2)
		 OLED_P6x8Str(36+55+6+5,0,"P ");
		else if(mode_rc[1]==1)
		 OLED_P6x8Str(36+55+6+5,0,"V ");
		
		if(mode_rc[2]==2)
		 OLED_P6x8Str(36+55+6*2+5,0,"H ");
		else if(mode_rc[2]==1)
		 OLED_P6x8Str(36+55+6*2+5,0,"R");

   my_itoa(plane.bat,temp[5]);
	 OLED_P6x8Str(36+66+10-6,0,temp[5]);
	 OLED_P6x8Str(36+66+10+6,0,"%");
		
   OLED_P6x8Str(3+3+55+10,1,"Way:");
   my_itoa(plane.mission.wayps,temp[1]);
	 OLED_P6x8Str(36+66+10,1,temp[1]);

	//fly time
	my_itoa(time_fly[0],temp[6]);
	OLED_P6x8Str(45-22,7,temp[6]);
	OLED_P6x8Str(45+3*3+3-22,7,":");
	
	my_itoa(time_fly[1],temp[6]);
	OLED_P6x8Str(45+3*3+3*3-22,7,temp[6]);	

}


void BEEP_TASK(float dt)
{ static u8 beep_rst;
 	Play_Music_Task(RC_ERO_BEEP,dt); 
	if(beep_rst++>20){beep_rst=0;
		Tone(0,0);
	}
}

float dt[10]={0},off_att[2];
u16 key_sel_down=0;
u8 rst_screen=0;
int main(void)
{	u8 i,j;
	static u16 cnt_flag[10];
	char temp[10][20]={'\0'};
  static u8 moniter_reg,fly_reg;
  static int cnt_get_data;
	u8 pid_sel[2],page,sub_page;

//------------------------------------//
	delay_init(72);		//延时初始化
	TIM3_Config();
	Cycle_Time_Init();
	OLED_Init();
	USART_init();	
	Adc_Init();
	SPI1_Init();		
	Parameter_Init();
	Nrf24l01_Init(MODEL_TX2,CHANNAL);// 伪双工  主接收
	Nrf24l01_Check();
	Mpu9250_Init();
  keyInit(); 
	OLED_Fill(0x00);OLED_P6x8Str(3,0,"By Golaced-BIT ");OLED_P8x16Str(43,3,"Welcome!");
	if(module.nrf)
	OLED_P6x8Str(3,7,"Nrf2.4G OK!");
	if(module.acc)
	OLED_P6x8Str(3,6,"Mpu9250 OK!");
	
  OLED_P6x8Str(3,5,"Channle:");
  my_itoa(CHANNAL,temp[0]);
  OLED_P6x8Str(50,5,temp[0]);
	delay_ms(20000);
	OLED_Fill(0x00);
	Beep_Init(0,72-1);
	Play_Music_Direct(START_BEEP);
	usb_vcp_init();
	delay_ms(1000);
	Time4ON();
	__enable_irq();
	while(1){
     if(rst_screen){rst_screen=0;
		 OLED_Fill(0x00);}
		 if(flag_ms[2]==1)
		 {
      flag_ms[2]=0;
			dt[0] = Get_Cycle_T(0)/1000000.0f;	
      if(dt[0]>0.005)
				dt[0]=0.005;
			else if(dt[0]<0.0015)
				dt[0]=0.0015;
			getFlyDataADCValue();
			MPU9250_ReadValue();
			MPU6050_Data_Prepare(dt[0]);
		 } 
		 
     if(flag_ms[5]==1)
		 {
      flag_ms[5]=0;
			dt[1] = Get_Cycle_T(1)/1000000.0f;	  
			if(dt[1]>0.01)
				dt[1]=0.01;
			else if(dt[1]<0.0025)
				dt[1]=0.0025;
			//IMUupdate(0.5f *dt[1],mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,&Rol_fc,&Pit_fc,&Yaw_fc); 
			madgwick_update_new( mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,mpu6050_fc.Gyro_deg.x/57.3, mpu6050_fc.Gyro_deg.y/57.3, mpu6050_fc.Gyro_deg.z/57.3,20,20,20, &Rol_fc,&Pit_fc,&Yaw_fc,dt[1]); 
			adc_rc.pitch=LIMIT(-my_deathzoom1(Pit_fc-off_att[0],0)*1.345*1.234,-45,45)/45.*500+1500;
			adc_rc.roll=LIMIT(my_deathzoom1(Rol_fc-off_att[1],0)*1.345,-45,45)/45.*500+1500; 
			Rc_Get.PIT=LIMIT(adc_rc.pitch,1000,2000);
			Rc_Get.ROL=LIMIT(adc_rc.roll,1000,2000);
			Rc_Get.YA=LIMIT(adc_rc.yaw,1000,2000);
			Rc_Get.THROTTLE=LIMIT(adc_rc.thrust,1000,2000);
			if(key_o[4]&&moniter_sel==0){
           key_sel_down++;
        }
				if(key_sel_down>2/0.005){
					  key_sel_down=0;
					  sub_sel[2]=!sub_sel[2];
					  off_att[0]=Pit_fc;
					  off_att[1]=Rol_fc;
					  WRITE_PARM();
					  Play_Music_Direct(RC_RESET_BEEP);				  
				}
       ANO_DT_Data_Exchange(); 	
		 } 
  
		 if(flag_ms[10]==1)
		 {
			dt[2] = Get_Cycle_T(2)/1000000.0f;	
      Nrf_Check_Event();
			RC_Send_Task();	
			 		 
      flag_ms[10]=0;
		 }
		 
		  if(flag_ms[20]==1)
		 {
			dt[3] = Get_Cycle_T(3)/1000000.0f;	   
      flag_ms[20]=0;
     
		 }
		 
		 if(flag_ms[25]==1)
		 {
			dt[4] = Get_Cycle_T(4)/1000000.0f;	   
      flag_ms[25]=0;
			
		 }

		 if(flag_ms[50]==1)
		 {
			dt[5] = Get_Cycle_T(5)/1000000.0f;	   
      flag_ms[50]=0;
			if(fly_reg==0&&plane.lock)
			{time_fly[3]=1;time_fly[0]=time_fly[1]=time_fly[2]=0;}
			if(fly_reg==1&&plane.lock==0)
				time_fly[3]=0;
			fly_reg=plane.lock;
			if(time_fly[3])
				time_fly[2]+=dt[5]*time_fly[4];
			if(time_fly[2]>1){
				time_fly[1]+=time_fly[2];
			  time_fly[2]-=1;
		  }
			if(time_fly[1]>60){
				time_fly[0]++;time_fly[1]-=60;
			} 
			
			#if USE_AS_MONITER 
			  if(Rc_Get.YA>1800)
				{cnt_flag[0]++;cnt_flag[2]++;}
				else
				{cnt_flag[0]=0;cnt_flag[2]=0;}
				
				if(Rc_Get.YA<1200)
				{cnt_flag[1]++;cnt_flag[3]++;}
				else
				{cnt_flag[1]=0;cnt_flag[3]=0;}
			 
				if(Rc_Get.THROTTLE>1800)
				{cnt_flag[4]++;cnt_flag[6]++;}
				else
				{cnt_flag[4]=0;cnt_flag[6]=0;}
				
				if(Rc_Get.THROTTLE<1200)
				{cnt_flag[5]++;cnt_flag[7]++;}
				else
				{cnt_flag[5]=0;cnt_flag[7]=0;}
				
				if(cnt_flag[1]>1/0.05)
				{cnt_flag[1]=cnt_flag[2]=0;moniter_sel--;}	
				if(cnt_flag[0]>1/0.05)
				{cnt_flag[0]=cnt_flag[3]=0;moniter_sel++;}	
				//---------------------
				if(cnt_flag[2]>0.1/0.05)
				{cnt_flag[1]=cnt_flag[2]=0;if(sub_sel[2]==0)sub_sel[0]--;}	
				if(cnt_flag[3]>0.1/0.05)
				{cnt_flag[0]=cnt_flag[3]=0;if(sub_sel[2]==0)sub_sel[0]++;}	
				
				if(key_o[4])
				{cnt_flag[8]++;cnt_flag[9]++;}
				else
				 cnt_flag[8]=cnt_flag[9]=0;
				if(cnt_flag[8]>0.1/0.05){
					  cnt_flag[8]=0;
					  sub_sel[2]=!sub_sel[2];
				}
				if(cnt_flag[9]>1.25/0.05&&moniter_sel!=0){
					  cnt_flag[9]=0;
					  for(i=0;i<18;i++)
					    for(j=0;j<3;j++)
					         plane.PID[i][j]=plane.PID_RX[i][j];
            Play_Music_Direct(UPLOAD_BEEP);
					  send_pid=1;
					  sub_sel[2]=0;
				}
				
				if(cnt_flag[6]>0.1/0.05)
				{cnt_flag[4]=cnt_flag[6]=0;if(sub_sel[2]==0)sub_sel[1]--;if(sub_sel[1]==3)sub_sel[1]=2;}	
				if(cnt_flag[7]>0.1/0.05)
				{cnt_flag[5]=cnt_flag[7]=0;if(sub_sel[2]==0)sub_sel[1]++;if(sub_sel[1]==3)sub_sel[1]=4;}	
				
				if(sub_sel[0]>1)
					sub_sel[0]=0;
				if(sub_sel[0]<0)
					sub_sel[0]=1;
				if(sub_sel[1]>6)
					sub_sel[1]=0;
				if(sub_sel[1]<0)
					sub_sel[1]=6;
				
				if(moniter_sel>6)
					moniter_sel=0;
				if(moniter_sel<0)
					moniter_sel=6;
				
			//change data
			if(sub_sel[1]<3)	
				sub_page=sub_sel[1];
			else
				sub_page=sub_sel[1]-4;
			
			if(sub_sel[0]==0&&sub_sel[1]<3)
				 page=0;
			if(sub_sel[0]==0&&sub_sel[1]>3)
				 page=1;
			if(sub_sel[0]==1&&sub_sel[1]<3)
				 page=2;
			if(sub_sel[0]==1&&sub_sel[1]>3)
				 page=3;
			
			pid_sel[0]=LIMIT((moniter_sel-1)*4+page,0,20);
			pid_sel[1]=LIMIT(sub_page,0,2);
					
			if(sub_sel[2]){	
				plane.PID_RX[pid_sel[0]][pid_sel[1]]+=my_deathzoom1(Rc_Get.THROTTLE-1500,50)*0.01;
				plane.PID_RX[pid_sel[0]][pid_sel[1]]+=my_deathzoom1(Rc_Get.YA-1500,150)*0.2;
				plane.PID_RX[pid_sel[0]][pid_sel[1]]=LIMIT(plane.PID_RX[pid_sel[0]][pid_sel[1]],0,9999);
			}	
			#endif 
			switch(moniter_sel)
      {
       case 0:				
			   OLED_TASK(0.05);  
			 break;
			 case 1:
				 OLED_TASK_PID1(0.05); OLED_TASK_SUB_SEL(0.05);
         if(cnt_get_data++>2.5/0.05)
				  {cnt_get_data=0;plane.read_pid=1;}					 
			 break;
			 case 2:
				 OLED_TASK_PID2(0.05); OLED_TASK_SUB_SEL(0.05);
         if(cnt_get_data++>2.5/0.05)
				  {cnt_get_data=0;plane.read_pid=1;}					 
			 break;	
			case 3:
				OLED_TASK_PID3(0.05); OLED_TASK_SUB_SEL(0.05);
				if(cnt_get_data++>2.5/0.05)
				{cnt_get_data=0;plane.read_pid=1;}					 
			break;	
			case 4:
				OLED_TASK_PID4(0.05); OLED_TASK_SUB_SEL(0.05);
				if(cnt_get_data++>2.5/0.05)
				{cnt_get_data=0;plane.read_pid=1;}					 
			break;	
			case 5:
				OLED_TASK_PID5(0.05); OLED_TASK_SUB_SEL(0.05);
				if(cnt_get_data++>2.5/0.05)
				{cnt_get_data=0;plane.read_pid=1;}					 
			break;
      case 6:
        OLED_TASK_MISSION(0.05);
      break;			
		  }
			if(sub_sel[2]==1)
				cnt_get_data=0;
			if(moniter_reg==0&&moniter_sel!=0)
				plane.read_pid=1;
			if(moniter_reg!=moniter_sel)
				sub_sel[2]=0;
			moniter_reg=moniter_sel;
			BEEP_TASK(0.05); 
			KEY_Scan(0.05);	
		 }
		 
		 
	}
}
/*********************************************END OF FILE**********************/

