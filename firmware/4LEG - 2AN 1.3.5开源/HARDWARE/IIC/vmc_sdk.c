#include "vmc.h"
#include "usart_fc.h"
#include "my_math.h"
#include "filter.h"
#include "nav.h"
#include "mavl.h"
int mission_sel_lock=1;//选择DEMO
int mission_state;
u8 mission_flag;
float timer_sdk[10];
char flag_sdk[10];
u32 cnt_mission[MAX_CNT_NUM];
u16 cnt_task[MAX_CNT_NUM];
//demo测试程序
u8 Cube_Track(float dt);
u8 Trajectory_Test(float dt);
//api
u8 delay_api(float time,float dt);
u8 set_spd_api(float x,float y,float z,char mode,float time,float dt);
u8 set_pos_api(float x,float y,float z,char mode,float dt);
u8 track_pixy_api(u8 mode,float dt);
u8 set_att_api(float pit,float rol,float yaw,float dt);
//SDK开发

void smart_control(float dt)
{
	static float cnt,timer_rst;
	u8 mission_finish=0;
	//reset
	timer_rst+=dt;
	if(timer_rst>0&&vmc_all.param.smart_control_mode[0]!=MODE_POS){
		timer_rst=0;
		vmc_all.param.tar_spd_on.x=vmc_all.param.tar_spd_on.z=0;
		vmc_all.param.tar_att_on[PITr]=vmc_all.param.tar_att_on[ROLr]=0;
		vmc_all.param.tar_att_on[YAWr]=vmc_all.att_ctrl[YAWr];
		vmc_all.param.tar_pos_on.x=vmc_all.pos_n.x;
		vmc_all.param.tar_pos_on.y=vmc_all.pos_n.y;
		vmc_all.param.tar_pos_on.z=vmc_all.pos.z;
		vmc_all.param.smart_control_mode[0]=vmc_all.param.smart_control_mode[1]=vmc_all.param.smart_control_mode[2]=0;
	}
	
	switch(mission_flag)
	{
	  case 0:
			if(vmc_all.param.en_sdk&&vmc_all.power_state>2)
				cnt+=dt;
			else
				cnt=0;
			if(cnt>1)
			{cnt=0;mission_flag=1;mission_state=0;}
		break;
		case 1://选择DEMO
			switch(mission_sel_lock)
			{
				case 0:
					mission_finish=Cube_Track(dt);
				break;
				case 1:
					mission_finish=Trajectory_Test(dt);
				break;
			}	
			if(vmc_all.param.en_sdk&&!Rc_Get_SBUS.update)//长按退出
				cnt+=dt;
			else
				cnt=0;
			if(cnt>2||mission_finish==1)
			{cnt=0;
			 vmc_all.param.smart_control_mode[0]=vmc_all.param.smart_control_mode[1]=vmc_all.param.smart_control_mode[2]=0;
			 mission_flag=0;}
		break;
	}
	if(mission_flag==1)
		vmc_all.param.control_mode_all=2;//sdk模式
	else
		vmc_all.param.control_mode_all=1;//遥控模式
//**-------------------------------------------------------------------------------**//
	if(o_cmd.connect)//外部控制命令
	{ vmc_all.param.control_mode_all=2;
		if(ABS(o_cmd.pos[Xr])>0.005||ABS(o_cmd.pos[Yr])>0.005||ABS(o_cmd.pos[Zr])>0.005)
			set_pos_api(o_cmd.pos[Xr],o_cmd.pos[Yr],o_cmd.pos[Zr],MODE_POS,dt);
		else{
			set_spd_api(o_cmd.spd[Xr],o_cmd.spd[Yr],o_cmd.spd[Zr],MODE_BODY,0,dt);
			set_att_api(o_cmd.att[PITr],o_cmd.att[ROLr],o_cmd.att[YAWr],dt);
		}
	}
	else if(pi.connect==MODE_CMD)//树莓派ROS控制命令
	{ vmc_all.param.control_mode_all=2;
		if(ABS(pi.cmd_pos[Xr])>0.005||ABS(pi.cmd_pos[Yr])>0.005||ABS(pi.cmd_pos[Zr])>0.005)
			set_pos_api(pi.cmd_pos[Xr],pi.cmd_pos[Yr],pi.cmd_pos[Zr],MODE_POS,dt);
		else{
			set_spd_api(pi.cmd_spd[Xr],pi.cmd_spd[Yr],pi.cmd_spd[Zr],MODE_BODY,0,dt);
			set_att_api(pi.cmd_att[PITr],pi.cmd_att[ROLr],pi.cmd_att[YAWr],dt);
		}
	}
}

void timer_init(void)
{
	u8 i;
  for(i=0;i<10;i++)
    timer_sdk[i]=flag_sdk[i]=0;
	for(i=0;i<MAX_CNT_NUM;i++)
   cnt_mission[i]=0;
}

u8 Cube_Track(float dt){
u8 flag=0,mission_finish=0;
	switch(mission_state){
	case 0:
		     if(vmc_all.power_state>2)
				   {flag=1;timer_init();}
		     break;	
	case 1:flag=delay_api(2,dt);break;		 
	case 2:
		     track_pixy_api(0,dt);
	      break;	
  case 3:mission_finish=1; 	
	default:break;
	}
	if(flag)
		mission_state++;
	
	if(mission_finish){
		mission_state=0;
	  return 1;}		
  else
    return 0;		
}	

u8 Trajectory_Test(float dt){
u8 flag=0,mission_finish=0;
	switch(mission_state){
	case 0:
		     if(vmc_all.power_state>2)
				   {flag=1;timer_init();}
		     break;	
	case 1:flag=delay_api(0.5,dt);break;		 
	case 2:
		     flag=set_pos_api(0,0,0,MODE_POS,dt);
	      break;	
	case 3:
		     flag=set_pos_api(0,0.68,0,MODE_POS,dt);
	      break;	
  case 4:mission_state=1; break;	
	default:break;
	}
	if(flag)
		mission_state++;
	
	if(mission_finish){
		mission_state=0;
	  return 1;}		
  else
    return 0;		
}	


//--------------------------------------API-----------------------------
u8 delay_api(float time,float dt)
{
	if(timer_sdk[0]>time)
	{
	  timer_sdk[0]=0;
		return 1;
	}else{
    timer_sdk[0]+=dt;
		return 0;
	}
}
// 								 俯仰  左右  前后  设定距离
float param_track[5]={30,0.25,0.03,25};
u8 track_pixy_api(u8 mode,float dt)
{
 float err[3];
 static float err_flt[3];
 static u8 hold;
 static float hold_timer,reset_timer;
	
 err[Yr]=LIMIT(pi.cube.y,-160,160);//俯仰
 err[Xr]=LIMIT(-pi.cube.x,-120,120);//左右
 err[Zr]=LIMIT(param_track[3]-pi.cube.s,-20,35);//前后
 if(pi.cube.check){
 Low_Fass_Filter(my_deathzoom(err[Yr],10),&err_flt[Yr], 1.5,  dt);//俯仰
 Low_Fass_Filter(my_deathzoom(err[Xr],10),&err_flt[Xr], 1.5,  dt);//左右 
 Low_Fass_Filter(my_deathzoom(err[Zr],10), &err_flt[Zr], 1.5,  dt);//前后
 }
 if(pi.cube.check){
	 hold_timer=0;hold=1;
	 reset_timer=0;
 }else
   reset_timer+=dt;
 if(hold)//pix
 {
	hold_timer+=dt; 
  vmc_all.gait_on=1;
	vmc_all.param.tar_spd_on.x=err_flt[Zr]*param_track[2];
  vmc_all.param.tar_spd_on.z=err_flt[Xr]*param_track[1];
	vmc_all.param.smart_control_mode[0]=MODE_SPD;
	 
	vmc_all.param.tar_att_on[PITr]+=err_flt[Yr]*param_track[0]*dt;
	vmc_all.param.tar_att_on[PITr]=LIMIT(vmc_all.param.tar_att_on[PITr],-12,15);
	vmc_all.param.smart_control_mode[2]=MODE_ATT;
 }
 if(reset_timer>2)
 {
	reset_timer=0; 
 	vmc_all.param.tar_spd_on.x=vmc_all.param.tar_spd_on.z=0;
	vmc_all.param.smart_control_mode[0]=MODE_SPD;
	 
	vmc_all.param.tar_att_on[PITr]=0;
	vmc_all.param.smart_control_mode[2]=MODE_ATT;
 }
 
 if(hold_timer>0.5)
	  hold_timer=hold=0;
 
 return 0;
}


u8 set_spd_api(float x,float y,float z,char mode,float time,float dt)
{
   if(mode==MODE_BODY)//body
	 {
	   vmc_all.param.tar_spd_on.x=x;
		 vmc_all.param.tar_spd_on.z=z;
		 vmc_all.param.smart_control_mode[0]=MODE_SPD;
	 }
   return delay_api(time,dt);
}


u8 set_att_api(float pit,float rol,float yaw,float dt)
{
			if(ABS(yaw)>0.5){
			 vmc_all.param.tar_att_on[YAWr]=yaw;
			 vmc_all.param.smart_control_mode[2]=MODE_ATT_YAW_ONLY;
			}				
			if(ABS(pit)>0.5){
			vmc_all.param.tar_att_on[0]=pit;
			if(vmc_all.param.smart_control_mode[2]==MODE_ATT_YAW_ONLY)
				vmc_all.param.smart_control_mode[2]=MODE_ATT;
			else
				vmc_all.param.smart_control_mode[2]=MODE_ATT_PR_ONLY;
			}
			if(ABS(rol)>0.5){
			vmc_all.param.tar_att_on[1]=rol;
			if(vmc_all.param.smart_control_mode[2]==MODE_ATT_YAW_ONLY)
				vmc_all.param.smart_control_mode[2]=MODE_ATT;
			else
				vmc_all.param.smart_control_mode[2]=MODE_ATT_PR_ONLY;
			}
   return 0;
}

float k_traj=0.368;
u8 traj_init_task(float ps[3],float pe[3],float T,u8 sel)
{		
			traj[sel].ps[Xr]=ps[0];//m
			traj[sel].ps[Yr]=ps[1];//m
			traj[sel].ps[Zr]=ps[2];//m
			traj[sel].vs[Xr]=MAX_SPD*sind(vmc_all.att_ctrl[YAWr]);//m
			traj[sel].vs[Yr]=MAX_SPD*cosd(vmc_all.att_ctrl[YAWr]);//m
			traj[sel].vs[Zr]=0;//m
			traj[sel].pe[Xr]=pe[0];//m
			traj[sel].pe[Yr]=pe[1];//m
			traj[sel].pe[Zr]=pe[2];//m
			traj[sel].time_now=0;
			traj[sel].Dis=my_sqrt(my_pow(traj[sel].ps[Xr]-traj[sel].pe[Xr])
													 +my_pow(traj[sel].ps[Yr]-traj[sel].pe[Yr]));

			traj[sel].Time=traj[sel].Dis/(MAX_SPD)*k_traj;
			traj[sel].defined[0]=1;traj[sel].defined[1]=1;traj[sel].defined[2]=0;
			if(T!=0)
				traj[sel].Time=T;//s
			plan_tra(&traj[sel]);
	    return 1;
}

//mode 1->pos  2->spd
u8 follow_traj(u8 sel,u8 mode,float dt)
{
	   if(mode==MODE_SPD){
				traj[sel].time_now+=dt;
				traj[sel].time_now=LIMIT(traj[sel].time_now,0,traj[sel].Time);
				get_tra(&traj[sel],traj[sel].time_now);
				float angle=fast_atan2(traj[sel].vt[0],traj[sel].vt[1])*57.3;
				vmc_all.param.smart_control_mode[0]=MODE_SPD;
				float body_spd[2];
				float yaw=vmc_all.att_ctrl[YAWr];
				body_spd[Yr]= traj[sel].vt[Yr]*cosd(yaw)+traj[sel].vt[Xr]*sind(yaw); 
				body_spd[Xr]=-traj[sel].vt[Yr]*sind(yaw)+traj[sel].vt[Xr]*cosd(yaw);
				vmc_all.param.tar_spd_on.x=LIMIT(body_spd[Yr],-MAX_SPD,MAX_SPD);
				vmc_all.param.tar_spd_on.z=0;
			 
				vmc_all.param.smart_control_mode[2]=MODE_ATT;
				vmc_all.param.tar_att_on[2]=angle;
					
				if(traj[sel].time_now>=traj[sel].Time)
					 return 1;
				else 
					 return 0;
			}else if(mode==MODE_POS){
				vmc_all.param.smart_control_mode[0]=MODE_POS;
			
				float dis=my_sqrt(my_pow(vmc_all.param.tar_pos_on.x-vmc_all.pos_n.x)
												 +my_pow(vmc_all.param.tar_pos_on.y-vmc_all.pos_n.y));	
				
				float dis_final=my_sqrt(my_pow(traj[sel].pe[Xr]-vmc_all.pos_n.x)
												       +my_pow(traj[sel].pe[Yr]-vmc_all.pos_n.y));	
				
				float Per_t=LIMIT(traj[sel].Dis/(vmc_all.H*1.75),0.025,0.1);
				traj[sel].traj_pre_d=Per_t;
				if(dis<vmc_all.H*3.5||flag_sdk[1]==0)
				{
					traj[sel].time_now+=traj[sel].Time*Per_t;
					traj[sel].time_now=LIMIT(traj[sel].time_now,0,traj[sel].Time);
					get_tra(&traj[sel],traj[sel].time_now);
					flag_sdk[1]=1;
				}
				vmc_all.param.tar_pos_on.x=traj[sel].pt[Xr];		
				vmc_all.param.tar_pos_on.y=traj[sel].pt[Yr];		
					
				 if(dis_final<POS_DEAD)//reach check
				 {flag_sdk[1]=0;return 1;}
				 else
					 return 0;
			}
}


u8 set_pos_api(float x,float y,float z,char mode,float dt)
{
	 float ps[3];
	 float pe[3];
	 char finish;
	 switch(flag_sdk[0]){
		 case 0:
		 ps[0]=nav.pos_n[Xr]+0.01;ps[1]=nav.pos_n[Yr]+0.01;
		 pe[0]=x+0.01;pe[1]=y+0.01;
		 traj_init_task(ps,pe,0,0);
		 flag_sdk[0]=1;
		 flag_sdk[1]=0;
		 break;
		 case 1:
		 finish=follow_traj(0,mode,dt);
		 if(finish){
			 flag_sdk[0]=flag_sdk[1]=0;
		   return 1;
			}	
		 break;
	 }
    return 0;
}

u8 way_point_task(double lat,double lon, float height,float loter_time,u8 mode,float dt)
{ 
	static float pos_reg[3];
	double tar_gps[3];
	if(cnt_mission[WAY_INIT]==0){
		pos_reg[0]=nav.lat;			
		pos_reg[1]=nav.lon;					
		pos_reg[2]=nav.pos_n[Zr];			
		cnt_mission[WAY_INIT]=1;
  }
	if(ABS(nav.local_lat)<5|| ABS(nav.local_lon)<5)
		return 2;
	
  if(ABS(lat)>5)
		tar_gps[0]=lat;
	else
	  tar_gps[0]=pos_reg[0];
	if(ABS(lon)>5)
		tar_gps[1]=lon;
	else
	  tar_gps[1]=pos_reg[1];
	if(height>0)
		tar_gps[2]=height;
	else
	  tar_gps[2]=pos_reg[2];
	
	float tar_Y,tar_X,tar_Yaw;			
	CalcGlobalDistance(tar_gps[0], tar_gps[1], nav.local_lat, nav.local_lon, &tar_Y,&tar_X );
	tar_Yaw=navCalcBearing(nav.lat, nav.lon,tar_gps[0], tar_gps[1]);	
  if(ABS(lat)>5&&ABS(lon)>5){
		set_pos_api(tar_X,tar_Y,0,MODE_POS,dt);
	}

	float dead_use=WAY_POINT_DEAD1;
	float check_time=MISSION_CHECK_TIME;
	if(mode==MODE_FAST_WAY){
		dead_use=WAY_POINT_DEAD1*6.6;
		check_time=MISSION_CHECK_TIME;
	}
	if(loter_time>1)
		check_time=loter_time;
  if(ABS(tar_X - nav.pos_n[Xr])<dead_use
		&&ABS(tar_Y - nav.pos_n[Yr])<dead_use){
		 flag_sdk[0]=flag_sdk[1]=0;
		 cnt_mission[WAY_CNT]++;
		}
	else
		 cnt_mission[WAY_CNT]=0;
	
	if(cnt_mission[WAY_CNT]>LIMIT(check_time,0.05,3600)/dt){
		cnt_mission[WAY_INIT]=cnt_mission[WAY_CNT]=0;
		return 1;
		}
	else
    return 0;			
}	

u8 way_point_mission(float spd_limit,u8 head_to_waypoint,float dt)
{
	u8 flag;
  double lat,lon,alt;
	lat=navData.missionLegs[cnt_mission[MISSION_CNT]].targetLat;
	lon=navData.missionLegs[cnt_mission[MISSION_CNT]].targetLon;
	alt=LIMIT(navData.missionLegs[cnt_mission[MISSION_CNT]].targetAlt,0,MAX_WAYP_Z);
	
	float tar_Yaw=To_180_degrees(navCalcBearing(nav.lat, nav.lon,lat,lon));	

	if(navData.Leg_num<1)
		 return 2;

  float dis_posNorth = (lat - nav.lat) * nav.r1;
  float dis_posEast =  (lon - nav.lon) * nav.r2;
	float dis=my_sqrt((dis_posNorth*dis_posNorth)+(dis_posEast*dis_posEast));
	if(ABS(lat)>5&&ABS(lon)>5&&alt>0&&navData.missionLegs[cnt_mission[MISSION_CNT]].type!=0){
//		if(navData.missionLegs[cnt_mission[MISSION_CNT]].maxHorizSpeed!=0)
//		nav_spd_pid.max_exp=LIMIT(navData.missionLegs[cnt_mission[MISSION_CNT]].maxHorizSpeed,0,3);
//		else
//		nav_spd_pid.max_exp=spd_limit;
//					
//		if(navData.missionLegs[cnt_mission[MISSION_CNT]].poiHeading!=0)
//		  set_drone_yaw_task(To_180_degrees(navData.missionLegs[cnt_mission[MISSION_CNT]].poiHeading*57.3)
//		  ,YAW_LIMIT_RATE,dt);
//    else if(head_to_waypoint&&dis>3){
//			set_drone_yaw_task(tar_Yaw,YAW_LIMIT_RATE,dt);
//    }	
	  float loiterTime;
		if(navData.missionLegs[cnt_mission[MISSION_CNT]].loiterTime>1)
			loiterTime=navData.missionLegs[cnt_mission[MISSION_CNT]].loiterTime;
		if(way_point_task(lat,lon,alt,loiterTime,MODE_FAST_WAY,dt))
		  {cnt_mission[F_YAW_INIT]=0;cnt_mission[MISSION_CNT]++;}
	}
	else
		cnt_mission[MISSION_CNT]++;
	
	if(cnt_mission[MISSION_CNT]>navData.Leg_num){
		cnt_mission[MISSION_CNT]=0;
		return 1;}
	else 
		return 0;
}

u8 return_home(float set_z, float spd_limit, u8 en_land, u8 head_to_home, float dt)
{
	 u8 flag=0;
		float tar_Yaw=To_180_degrees(navCalcBearing(nav.lat, nav.lon,nav.home_lat,nav.home_lon));	
		float dis_posNorth = (nav.home_lat - nav.lat) * nav.r1;
		float dis_posEast =  (nav.home_lon - nav.lon) * nav.r2;
		float dis=my_sqrt((dis_posNorth*dis_posNorth)+(dis_posEast*dis_posEast));
	 switch(cnt_mission[HOME_CNT]){
			case 0:
				if(way_point_task(nav.home_lat,nav.home_lon,0,0,NMODE_FAST_WAY,dt))
					cnt_mission[HOME_CNT]++;
			break;
			case 1:
				cnt_mission[HOME_CNT]=0;
				return 1;
			break;	
   }
	 
		return 0;
}