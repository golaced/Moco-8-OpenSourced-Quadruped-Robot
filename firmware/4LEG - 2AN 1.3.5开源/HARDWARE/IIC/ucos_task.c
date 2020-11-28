#include "include.h" 
#include "bat.h"
#include "imu.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "sbus.h"
#include "rc_mine.h"
#include "filter.h"
#include "dog.h"
#include "vmc.h"
#include "iic_vl53.h"
#include "usart_fc.h"
#include "pwm_out.h"
#include "beep.h"
#include "nav.h"
#include "ms5611.h"
#include "eso.h"
#include "mavl.h"
#include "spi.h"
#include "gps.h"
float leg_dt[GET_TIME_NUM];
OS_STK FUSION_TASK_STK[FUSION_STK_SIZE];
u8 en_nav=1,en_hml=1;
float FLT_ACC=10;
void fusion_task(void *pdata)
{
 u8 i;
 static u8 init;	
 static u16 cnt_init;
 static float timer_baro;
 	while(1)
	{
	leg_dt[0] = Get_Cycle_T(GET_T_LEG1); 	
	IMU_Read(); 														
	IMU_Data_Prepare( leg_dt[0] );		

	Read_Ground_Key(leg_dt[0]);
	for(i=0;i<4;i++)
	  vmc[i].ground_s=ls53[i].mode;
		
	if(cnt_init++>1/0.005){cnt_init=65530;
		#if defined(ATT_MAD)
			madgwick_update_new(leg_dt[0],
			mems.Gyro_deg.x/57.3, mems.Gyro_deg.y/57.3,mems.Gyro_deg.z/57.3, 
			mems.Acc.x, mems.Acc.y, mems.Acc.z,
			mems.Mag.x*module.hml_imu*mems.Mag_Have_Param*en_hml,
			mems.Mag.y*module.hml_imu*mems.Mag_Have_Param*en_hml,
			mems.Mag.z*module.hml_imu*mems.Mag_Have_Param*en_hml,
			&Pitch,&Roll,&Yaw);
		#endif
		#if defined(ATT_COM)
			IMUupdate(leg_dt[0]/2,
			mems.Gyro_deg.x, mems.Gyro_deg.y,mems.Gyro_deg.z, 
			mems.Acc.x, mems.Acc.y, mems.Acc.z,
			&Pitch,&Roll,&Yaw);
		#endif
  float a_br[3],acc_temp[3];
	static float acc_flt[3];
	a_br[0] =(float) mems.Acc.x/4096.;
	a_br[1] =(float) mems.Acc.y/4096.;
	a_br[2] =(float) mems.Acc.z/4096.;
	acc_temp[0] = a_br[1]*reference_vr[2]  - a_br[2]*reference_vr[1] ;
	acc_temp[1] = a_br[2]*reference_vr[0]  - a_br[0]*reference_vr[2] ;
	acc_temp[2] = reference_vr[2] *a_br[2] + reference_vr[0] *a_br[0]+ reference_vr[1] *a_br[1] - 1 ;
	
	vmc_all.att[PITr]=Pitch;
	vmc_all.att[ROLr]=-Roll;
	vmc_all.att[YAWr]=Yaw;		
	#if VIR_MODEL
	  vmc_all.att[YAWr]=nav.fake_yaw;
	#endif
	vmc_all.att_rate[PITr]=mems.Gyro_deg.x;
	vmc_all.att_rate[ROLr]=mems.Gyro_deg.y;
	vmc_all.att_rate[YAWr]=mems.Gyro_deg.z;
	DigitalLPF( acc_temp[0]*9.8, &vmc_all.acc[Xr], FLT_ACC, leg_dt[0]);
	DigitalLPF(-acc_temp[1]*9.8, &vmc_all.acc[Yr], FLT_ACC, leg_dt[0]);
	DigitalLPF( acc_temp[2]*9.8, &vmc_all.acc[Zr], FLT_ACC, leg_dt[0]);
		
	MS5611_Update(leg_dt[0]);
  }
	delay_ms(5);
	}
}		

OS_STK POSE_FUSION_TASK_STK[POSE_FUSION_STK_SIZE];
void pose_fusion_task(void *pdata)
{
 u8 i;
 static u8 init;	
 static u16 cnt_init;
 static float timer_pose_fushion;
 	while(1)
	{
	  leg_dt[GET_T_FUSHION] = Get_Cycle_T(GET_T_FUSHION); 			
		if(cnt_init++>1/0.005){cnt_init=65530;	
    if(en_nav){
    baro_fushion(leg_dt[GET_T_FUSHION]);
		vmc_all.pos_n.z=nav.pos_n[Zr];
			
    timer_pose_fushion+=leg_dt[GET_T_FUSHION];	
    if(timer_pose_fushion>0.03)
			{pose_fushion(timer_pose_fushion);	
			 vmc_all.pos_n.x=nav.pos_n[Xr];
			 vmc_all.pos_n.y=nav.pos_n[Yr];
			 timer_pose_fushion=0;}
	  }
	}
		delay_ms(10);
	}
}		


//========================步态  任务函数============================

OS_STK BRAIN_TASK_STK[BRAIN_STK_SIZE];
float k_rc_spd=0.002;//速度遥控增益
float k_z_c= 0.12;//旋转遥控增益
float spd_force=0;//强制给定前进速度
float FLT_RC=0.7;//0.95;//遥控器低通 
int test_power=0;
void brain_task(void *pdata)
{	static u8 cnt,cnt1,cnt2,init,rc_update;	
	static float timer_sdk;
	static float auto_power_off;
	float T;float spd,spdy,spdx,yaw=0,w_rad;
	int i;
	u16 temps;
	float ero[2];
	float ero_r[2];
 	while(1)
	{	
	leg_dt[4] = Get_Cycle_T(GET_T_BRAIN);								//获取外环准确的执行周期
  T=LIMIT(leg_dt[4],0.001,0.05);
	
	if(!init){init=1;	vmc_init();}	
				
	if(vmc_all.param.cal_flag[0]&&module.flash){
	for(i=0;i<4;i++)
		vmc_all.param.ground_force[i][0]=press_leg_end[i+1]*1.068;
		mems.Gyro_CALIBRATE=1;
	vmc_all.param.cal_flag[0]=0;
	}else if(vmc_all.param.cal_flag[1]==1&&module.flash){
		mems.Gyro_CALIBRATE=1;
		vmc_all.param.cal_flag[1]=0;
	}else if(vmc_all.param.cal_flag[1]==2&&module.flash){
		mems.Acc_CALIBRATE=mems.Gyro_CALIBRATE=1;
		vmc_all.param.cal_flag[1]=0;
	}
	
	if(Rc_Get_SBUS.update)//--------------使用SBUS
	{ 
		auto_power_off=0;
	  Rc_Get.THROTTLE=LIMIT(Rc_Get_SBUS.THROTTLE,1000,2000)	;
		Rc_Get.ROLL=my_deathzoom_rc(Rc_Get_SBUS.ROLL,2)	;
		Rc_Get.PITCH=my_deathzoom_rc(Rc_Get_SBUS.PITCH,2)	;
		Rc_Get.YAW=my_deathzoom_rc(Rc_Get_SBUS.YAW,2)	;
		Rc_Get.AUX1=Rc_Get_SBUS.AUX1;
		Rc_Get.AUX2=Rc_Get_SBUS.AUX2;
		Rc_Get.AUX3=Rc_Get_SBUS.AUX3;
		Rc_Get.AUX4=Rc_Get_SBUS.AUX4;

		 spd=LIMIT(my_deathzoom(Rc_Get.PITCH-1500,25)*MAX_SPD/1000.,-MAX_SPD*0.86,MAX_SPD);
		 w_rad=LIMIT(my_deathzoom(my_deathzoom(Rc_Get.ROLL-1500,25)*k_z_c,1.68),-MAX_SPD_RAD,MAX_SPD_RAD);//degree
		 vmc_all.tar_spd.z=-w_rad;vmc_all.tar_spd.y=0;
		 	
		 if(Rc_Get.AUX3<1600&&Rc_Get.AUX3>1400){//车辆模式
			 vmc_all.tar_spd.x+=20*my_deathzoom(spd,0.25*MAX_SPD)*T;
			 vmc_all.tar_spd.x=LIMIT(vmc_all.tar_spd.x,-MAX_SPD*0.95,MAX_SPD*0.95);
		 }else vmc_all.tar_spd.x=spd*2;
 
			if(Rc_Get.AUX4<1500){//姿态模式 

				vmc_all.tar_att[PITr]=-my_deathzoom(Rc_Get.PITCH-1500,25)/1000.*22;
				vmc_all.tar_att[ROLr]=my_deathzoom(Rc_Get.ROLL-1500,25)/1000.*18;
				vmc_all.param.smart_control_mode[2]=MODE_ATT;
				vmc_all.tar_spd.x=spd=vmc_all.tar_spd.z=w_rad=0;
			}else if(ABS(Rc_Get.AUX2-1500)>25){//偏差微调
			  if(Rc_Get.AUX3>1600)
					vmc_all.tar_att[PITr]=my_deathzoom(Rc_Get.AUX2-1500,25)/1000.*15;	
				else
					vmc_all.tar_att[ROLr]=my_deathzoom(Rc_Get.AUX2-1500,25)/1000.*15;
			}
			if(ABS(vmc_all.tar_spd_rc.z)>0.1)
				 vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
			
			//高度调节
			if(Rc_Get.AUX4<1600&&Rc_Get.AUX4>1400){
				vmc_all.tar_pos.z+=-my_deathzoom(Rc_Get.PITCH-1500,25)/1000.*vmc_all.param.max_l*0.0075;
			  vmc_all.tar_pos.z=LIMIT(vmc_all.tar_pos.z,MAX_Z*0.9,MIN_Z*1.1);
				vmc_all.tar_spd.x=spd=0;
		  }		
			
		 if(ABS(vmc_all.tar_spd.x)>MAX_SPD*0.015)	
			vmc_all.param.have_cmd_rc[0]=1;
		 else 
			vmc_all.param.have_cmd_rc[0]=0;	
		 if(ABS(vmc_all.tar_spd.z)>MAX_SPD_RAD*0.015)	
			vmc_all.param.have_cmd_rc[1]=1;
		 else 
			vmc_all.param.have_cmd_rc[1]=0;	
		static int reg_aux;
		 //强制断电判断
		 if(Rc_Get.THROTTLE<1200&&Rc_Get.YAW<1200&&Rc_Get.PITCH<1200&&Rc_Get.ROLL>1800
			 &&vmc_all.sita_test[4]==0){
				vmc_all.tar_spd.x=vmc_all.tar_spd.z=0;
				vmc_all.power_state=vmc_all.leg_power=vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x=0;
				for(int i=0;i<4;i++){
					vmc[i].sita1=0-25;
					vmc[i].sita2=180-(-25);
					#if USE_DEMO
					estimate_end_state_d(&vmc[i],T);
					#else
					estimate_end_state(&vmc[i],T);
					#endif
					vmc[i].ground=1; 
				 }
			}
		 reg_aux=Rc_Get.AUX1;  
	}
	else if(module.nrf==2){//----------------使用手持遥控器	
	 auto_power_off=0;
	 DigitalLPF(LIMIT(my_deathzoom(Rc_Get.THROTTLE-1500,25)*MAX_SPD/1000.,-MAX_SPD*0.86,MAX_SPD),
		&spd,FLT_RC,T);
	 DigitalLPF(LIMIT(my_deathzoom(my_deathzoom(Rc_Get.YAW-1500,25)*k_z_c,1.68),-MAX_SPD_RAD,MAX_SPD_RAD),//degree
		&w_rad,FLT_RC,T);
	 {vmc_all.tar_spd.z=-w_rad;vmc_all.tar_spd.y=0;}
	 
	 vmc_all.tar_att[ROLr]=LIMIT(vmc_all.tar_spd.y*2,-3+vmc_all.tar_att_off[ROLr],3+vmc_all.tar_att_off[ROLr]);
	 
	 #if defined(USE_GIMBAL)
	 if(KEY[1]){//云台控制
		 gimbal.angle_set[PITr]=gimbal.angle[PITr];
		 gimbal.angle_set[YAWr]=gimbal.angle[YAWr];
		 vmc_all.tar_spd.x=vmc_all.tar_spd.z=0;
		 gimbal.angle_cmd[PITr]=my_deathzoom(Rc_Get.THROTTLE-1500,25)*gimbal.rc_k[PITr];
		 gimbal.angle_cmd[YAWr]=my_deathzoom(Rc_Get.YAW-1500,25)*gimbal.rc_k[YAWr];
	 }else vmc_all.tar_spd.x=spd*2;
	 #else
	 if(KEY[1]){
		 gait_test[0]=2;
		 vmc_all.tar_spd.x=0;
	 }else {vmc_all.tar_spd.x=spd*2;gait_test[0]=0;}
		 
//	 if(KEY[1]){//车辆模式
//		 vmc_all.tar_spd.x+=20*my_deathzoom(spd,0.25*MAX_SPD)*T;
//		 vmc_all.tar_spd.x=LIMIT(vmc_all.tar_spd.x,-MAX_SPD*0.95,MAX_SPD*0.95);
//	 }else vmc_all.tar_spd.x=spd*2;
	 #endif
	 
	  if(ABS(vmc_all.tar_spd_rc.z)>0.1)
			 vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
		
	 if(ABS(vmc_all.tar_spd.x)>MAX_SPD*0.015)	
		vmc_all.param.have_cmd_rc[0]=1;
	 else 
		vmc_all.param.have_cmd_rc[0]=0;	
	 if(ABS(vmc_all.tar_spd.z)>MAX_SPD_RAD*0.015)	
		vmc_all.param.have_cmd_rc[1]=1;
	 else 
		vmc_all.param.have_cmd_rc[1]=0;	
	 
	 if(KEY[0])//参数调整保护 机器人静止
	   vmc_all.tar_spd.x=vmc_all.tar_spd.z=0;
		 
		 //强制断电判断
	 if(KEY[1]&&KEY[0]&&vmc_all.sita_test[4]==0){
		  vmc_all.tar_spd.x=vmc_all.tar_spd.z=0;
		  vmc_all.power_state=vmc_all.leg_power=vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x=0;
	    for(int i=0;i<4;i++){
				vmc[i].sita1=0-25;
				vmc[i].sita2=180-(-25);
				#if USE_DEMO
				estimate_end_state_d(&vmc[i],T);
				#else
				estimate_end_state(&vmc[i],T);
				#endif
				vmc[i].ground=1; 
	     }
		}
   }else //无遥控速度清零
	  vmc_all.param.have_cmd_rc[0]=vmc_all.param.have_cmd_rc[1]=vmc_all.tar_spd.x=vmc_all.tar_spd.y=vmc_all.tar_spd.z=vmc_all.tar_att[ROLr]=0;
 
	 	//强制断电判断PI CMD外部控制
	 if(pi.connect&&pi.cmd_mode==99){
		  vmc_all.power_state=vmc_all.leg_power=vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x=0;
	    for(int i=0;i<4;i++){
				vmc[i].sita1=0-25;
				vmc[i].sita2=180-(-25);
				#if USE_DEMO
				estimate_end_state_d(&vmc[i],T);
				#else
				estimate_end_state(&vmc[i],T);
				#endif
				vmc[i].ground=1; 
	     }
		}
	 
	//超时自动断电
	 auto_power_off+=T;
   if(auto_power_off>20&&!pi.connect){		
		 vmc_all.power_state=vmc_all.leg_power=vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x=0;
	    for(int i=0;i<4;i++){
				vmc[i].sita1=0-25;
				vmc[i].sita2=180-(-25);
				#if USE_DEMO
				estimate_end_state_d(&vmc[i],T);
				#else
				estimate_end_state(&vmc[i],T);
				#endif
				vmc[i].ground=1; 
	     }
	 }
	 //---------SDK  程序------------
	 static float cnt_param_saving=0;
	 if(Rc_Get_SBUS.update)
			if(Rc_Get.AUX1>1800)
				vmc_all.param.en_sdk=1;
			else{
				vmc_all.param.smart_control_mode[0]=vmc_all.param.smart_control_mode[1]=vmc_all.param.smart_control_mode[2]=0;
				mission_flag=vmc_all.param.en_sdk=0;}
	 else if(module.nrf==2)
	 {
		if(KEY[0]==0)
			vmc_all.param.en_sdk=KEY[4];
		else if(KEY[0]==1)
			if(module.flash==1&&KEY[4])
				cnt_param_saving+=T;
		
		if(cnt_param_saving>2.5){
			cnt_param_saving=0;
			WRITE_PARM();
			module.flash=2;
		 }
	 }
	 else
		vmc_all.param.en_sdk=cnt_param_saving=0;
	 vmc_all.param.en_sdk*=nav.init[Xr]; 
	 
	 timer_sdk+=T;
	 if(timer_sdk>0.02||0){
		 smart_control(timer_sdk);
		 timer_sdk=0;
	 }
	//-------------------------------------
	#if DEBUG_MODE
		//spd_force=0.2;
	#endif
  if(spd_force!=0)//强制速度测试
	 vmc_all.tar_spd.x=spd_force;
	
	#if !USE_FALL_RECOVER
	if(vmc_all.err==1){
		vmc_all.err=2;
	#else
	if(vmc_all.fall==2){	
		vmc_all.fall=0;
	#endif
	  IWDG_Init(4,500*1);
	}

	#if DEBUG_MODE
	  vmc_all.leg_power=test_power;
	#endif
	fly_ready=LIMIT(vmc_all.leg_power,0,1);
	#if USE_DEMO
	if(power_task_d(T)!=2&&vmc_all.err==0)
	#else
	if(power_task(T)!=2&&vmc_all.err==0)
	#endif
	
	IWDG_Feed();
  leg_power_control(vmc_all.leg_power);
	
	#if DEBUG_MODE
	if(cnt2++>5||0)
	#endif
	{cnt2=0;
	#if USE_DEMO
  	VMC_DEMO(T);
	#else
		VMC_OLDX_VER1(T);
		//VMC_OLDX_TEST1(T);
		//VMC_OLDX_TEST2(T);
  #endif
	}
	
	Bat_protect(T);
	#if defined(USE_GIMBAL)
	  gimbal_control(T);
	#endif 
	Set_DJ_PWM();//舵机PWM输出
	#if defined(USE_OLDX_REMOTER)
	  delay_ms(5);
	#else
		delay_ms(5);
	#endif
	}
}		

//=======================串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=4;//<------------------------------UART UPLOAD DATA SEL
float ground_check_show=0;
void uart_task(void *pdata)
{	static u16 cnt[3];
	static float cnt_mavlink_data,cnt_rc;
 	while(1)
	{		leg_dt[1] = Get_Cycle_T(GET_T_LEG2); 		
			switch(UART_UP_LOAD_SEL)
			{
			case 0:
			data_per_uart_rc(
			ls53[0].mode*100,-ls53[1].mode*100,ls53[2].mode*200,
			-ls53[3].mode*200,ls53[0].distance,ls53[0].ambient/10,
			press_leg_end[2]*100,press_leg_end[3]*100,0,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;		
			case 1:
			data_per_uart_rc(
			vmc_all.body_spd[Zr]*1000,vmc[0].epos.z*1000,vmc[1].epos.z*1000,
			vmc[0].tar_epos.z*1000,vmc[0].spd.x*1000,vmc[0].tar_spd.x*1000,
			(vmc_all.deng_all*0.0001)*1000,vmc_all.tar_pos.z*1000,vmc_all.pos.z*1000,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;	
		  case 2:
			data_per_uart_rc(
			ls53[0].distance,ls53[1].distance,ls53[2].distance,
			ls53[3].distance,ls53[0].ambient/10,ls53[1].ambient/10,
			ls53[2].ambient/10,ls53[3].ambient/10,0,
			(int16_t)(Pitch*10),(int16_t)(Roll*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;			
			case 3:
			data_per_uart_rc(
			vmc_all.tar_pos.z*1000,vmc_all.pos.z*1000,0,
		  vmc_all.tar_att[YAWr],vmc_all.att_ctrl[YAWr],0,
			vmc_all.tar_spd.z,vmc_all.att_rate_ctrl[YAWr],0,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;	
		  case 4:
			data_per_uart_rc(
			vmc_all.att_ctrl[ROLr]*10,vmc_all.att[ROLr]*10,(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr])*10,
		  vmc_all.att_rate_ctrl[ROLr]*10,vmc_all.att_rate[ROLr]*10,att_pid_inner_all[ROLr].exp*10,
			vmc_all.att_rate_vm[ROLr]*10,att_rate_eso[ROLr].z[0]*10,att_rate_eso[ROLr].z[1]*10,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 5:
			data_per_uart_rc(
			nav.acc_b_o[Xr]*100,nav.acc_b_o[Yr]*100,0,
		  nav.acc_n_o[Xr]*100,nav.acc_n_o[Yr]*100,0,
			vmc_all.tar_pos.z*1000,vmc_all.pos.z*1000,vmc_all.param.w_t[ROLr]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
      case 6:
			data_per_uart_rc(
			vmc[0].ground_force[0]*100,vmc[1].ground_force[0]*100,vmc[2].ground_force[0]*100,
		  vmc[3].ground_force[0]*100,ground_check_show*100,vmc_all.ground[1][0]*100,
			-vmc_all.ground[1][1]*100,-vmc_all.ground[1][2]*100,vmc_all.ground[1][3]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 7:
			data_per_uart_rc(
			press_leg_end[1]*100,press_leg_end[2]*100,press_leg_end[3]*100,
		  press_leg_end[4]*100,ground_check_show*100,vmc[0].ground_force[1]*100,
			vmc[1].ground_force[1]*100,vmc[2].ground_force[1]*100,vmc[3].ground_force[1]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 8:
			data_per_uart_rc(
			vmc_all.param.line_z[0]*1000,vmc_all.param.line_z[1]*1000,press_leg_end[3]*100,
		  vmc[0].epos.x*100,vmc[1].epos.x*100,vmc[0].ground_force[1]*100,
			vmc[1].ground_force[1]*100,vmc[2].ground_force[1]*100,vmc[3].ground_force[1]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 9:
			data_per_uart_rc(
			vmc_all.acc[Zr]*100,vmc_all.body_spd[Zr]*100,nav.spd_n[Zr]*100,
			ms5611.baroAlt_flt,ms5611.baroAlt-ms5611.baroAlt_off,nav.pos_n[Zr]*100,
			nav.acc_n[Zr]*10,ms5611.baroAlt_flt1*100,0,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 10:
			data_per_uart_rc(
			nav.spd_b[Xr]*100,nav.spd_b_o[Yr]*100,nav.spd_b[Yr]*100,
			nav.pos_n[Xr]*100,nav.pos_n[Yr]*100,nav.att[YAWr]*10,
			vmc_all.att_rate[Zr],0,nav.spd_b_o[Zr],
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 11:
			data_per_uart_rc(
			yaw_mag_view[0]*10,yaw_mag_view[1]*10,yaw_mag_view[2]*10,
			nav.acc_b_o[Xr]*100,nav.acc_b_o[Yr]*100,nav.acc_b_o[Zr]*100,
			Pitch*10,Roll*10,Yaw*10,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;

			default:break;
			}

			#if defined(USE_OLDX_REMOTER)
			cnt_rc+=leg_dt[1];
			if(cnt_rc>0.02){
				RC_Send_Task();	
				Nrf_Check_Event(cnt_rc);	
				cnt_rc=0;				
			}
			#endif
			
			static float timer_pose_fushion;
			if(cnt[0]++>1){cnt[0]=0;		
			#if defined (LEG_USE_VLX)	
				READ_VL53(0);
				READ_VL53(1);
				READ_VL53(2);
				READ_VL53(3);
			#endif
			#if defined (LEG_USE_VL6)
				RangePollingRead();
			#endif
						
				
			#if EN_DMA_UART1 
			#if defined(USE_MAVLINK)
			cnt_mavlink_data+=leg_dt[1];
			if(cnt_mavlink_data>0.05||1){cnt_mavlink_data=0;
			update_mavlink();//解析Mavlink接受数据			
			}
			static u8 mav_state;
			if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)
			{ 	
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
			for	(SendBuff1_cnt=0;SendBuff1_cnt<SEND_BUF_SIZE1;SendBuff1_cnt++)
			SendBuff1[SendBuff1_cnt]=0;
			SendBuff1_cnt=0;
			if(cnt[0]++>1/0.02){cnt[0]=0;			
			mavlink_send_message(0, MSG_HEARTBEAT, 0);
			mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 1000-mavlinkData.idlePercent,
			bat.average * 1000, -1, bat.percent*100, 0, mavlinkData.packetDrops, 0, 0, 0, 0);
			u8 RADIO_QUALITY;
			//if()
			//mavlink_msg_radio_status_send(MAVLINK_COMM_0, RADIO_QUALITY, 0, 0, 0, 0, 0, 0);	
			}		
			if(cnt[1]++>0.5/0.02){cnt[1]=0;			
			mavlink_send_message(0, MSG_RADIO_IN, 0);
			mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, GetSysTime_us(), mems.Acc.x,  mems.Acc.y,  mems.Acc.z, 
			mems.Gyro_deg.x*10.0f, mems.Gyro_deg.y*10.0f, mems.Gyro_deg.z*10.0f,0,0,0);
			}						

			if(cnt[2]++>2/0.02){cnt[2]=0;			
			uint8_t satellites_visible=0,fix_tpy=0;
			if(Gps_information.fix_type==3){
			if(Gps_information.satellite_num>=8)
				fix_tpy=3;
			else
				fix_tpy=2;
			satellites_visible=Gps_information.fix_type;}
			mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, GetSysTime_us(), fix_tpy, nav.lat*(double)1e7, nav.lon*(double)1e7, 
			0,0,0,0,0, satellites_visible);
			}

			mavlink_send_message(0, MSG_ATTITUDE, 0);
			mavlink_send_message(0, MSG_LOCATION, 0);

			if (mavlinkData.wpCurrent < mavlinkData.wpCount && mavlinkData.wpAttempt <= AQMAVLINK_WP_MAX_ATTEMPTS && mavlinkData.wpNext < GetSysTime_us() ) {
			mavlinkData.wpAttempt++;
			mavlink_msg_mission_request_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, mavlinkData.wpCurrent);
			mavlinkData.wpNext = GetSysTime_us() + AQMAVLINK_WP_TIMEOUT;
			}
			else if (mavlinkData.wpCurrent == mavlinkData.wpCount) {
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, 0);
			mavlinkData.wpCurrent++;
			} else if (mavlinkData.wpAttempt > AQMAVLINK_WP_MAX_ATTEMPTS) {
			}

			USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);    
			MYDMA_Enable(DMA2_Stream7,SendBuff1_cnt+2);  	  
			}		
			#else
				if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)
					{ 
						DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		
						clear_leg_uart();
						data_per_uart1();
						USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);      
						MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);    
					}	
			#endif
			#endif	

			#if EN_DMA_UART3 
				static u8 sel_pi,cnt_pi;
				if(cnt_pi++>3){cnt_pi=0;
				if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)
				{ 
					DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);		
					clear_pi_uart();
					
					if(sel_pi++>5){
					sel_pi=0;
					data_per_uart3(1);	
					}else
					data_per_uart3(0);
					USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);      
					MYDMA_Enable(DMA1_Stream3,leg_pi_cnt+2);    
				}
			  }
			#endif
		}
			delay_ms(10);  
	}
}	


//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}

//软件定时器2的回调函数				  50ms	 
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
	u8 i;	
	static u16 cnt_1,cnt_2;	
	static u8 cnt;
	LEDRGB_STATE(0.05);
	if(Rc_Get_SBUS.lose_cnt++>2/0.05)Rc_Get_SBUS.connect=0;
	if(module.nrf>0&&module.nrf_loss_cnt++>2/0.05)module.nrf=1;
	if(o_cmd.lost_cnt++>125)o_cmd.connect=0;
	if(pi.lost_cnt++>3/0.05)pi.connect=0;
	if(flow.lost_cnt++>3/0.05)flow.connect=0;
	
	//判断航点写入
	  static u8 way_point_update,count_reg=1;
	  static u16 cnt_way_point=0;
	  switch(way_point_update)
		{
			case 0:
				 if(mavlinkData.wpCurrent!=count_reg)
					 way_point_update=1;
			break;
			case 1:
				 if(mavlinkData.wpCount==mavlinkData.wpCurrent-1)
					 way_point_update=2;
			break;
			case 2:
				 if(cnt_way_point++>2/0.05){
					 cnt_way_point=0;
				   way_point_update=3;
				 }
			case 3:			
			if(!fly_ready){	
			way_point_update=0;
			navGetWaypointCount();	
			#if !FLASH_USE_STM32
				WRITE_PARM_WAY_POINTS();
			#endif
			mavlinkData.wpNext=1;	
			Play_Music_Direct(MEMS_WAY_UPDATE);
			}				
			break;
		}
	  count_reg=mavlinkData.wpCurrent;	
}
