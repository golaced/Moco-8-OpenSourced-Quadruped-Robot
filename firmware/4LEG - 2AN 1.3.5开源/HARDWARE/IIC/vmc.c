#include "vmc.h"
#include "my_math.h"
#include "math.h"
#include "eso.h"
//mems
VMC vmc[4];
VMC_ALL vmc_all;
PID h_pid[4],h_pid_all, att_pid[2][4],att_pid_all[3],pos_pid[2],pos_pid_all,extend_pid[4],extend_pid_all;
PID att_pid_outter_all[2],att_pid_inner_all[2];
ESO att_rate_eso[3];
GAIT_SHEC gait;
float MIN_Z=-0.1;
float MAX_Z=-0.19;
float MIN_X=-0.15;
float MAX_X=0.15;
float MAX_SPD=0;
float MAX_SPD_RAD=40;

char deng_sel=1;
float end_tirg_rate=0.68;//跨腿中点比例
float MAX_DENG_ANGLE=180;
float T_GAIN=1;
float FLT_ATT_CTRL[3]={1.68,0.68,0.68};//姿态角  角速度  机械角速度
float FLT_ATT_TRIG=5;
float FLT_ATT_CTRL_Y=0.35;

float FLT_SPD_END=6;
float FLT_SPD_END_Z=6;
float FLT_POS_Z=6;
float FLT_AUTO_T=0.5;
float FLT_TORQUE=25;
float FLT_COG_OFF=0.5;
float FLT_ATT_SPD_ERR=6;//0.368;
float FLT_ENCODER=2;
float FLT_POLAR=1.68;

//回缩高度 ， 回缩时间比例   ， 伸长时间终止比例 ,    姿态倾斜蹬腿补偿,  时间加速倍率
float deng_param[5]={0.5,0.5,1,0,1};
//时间动态调节参数
#if MINI_TO_BIG
float gait_change_param[6]={16.8,0.00123,4,0.003,0.0732,0.45};
#else
float gait_change_param[6]={16.8,0.00123,4,0.003,0.0732,0.36};
#endif
float en_fix[5]={0,1,1,1,0.85};
int flag_att_force[3]={0,0,0};
static char ukey_state=0,block_lisence=0;
char block_yaw=0;//屏蔽航向控制
char  out_flag=0;
void get_license(void)
{
		int mcuID[3];	
    mcuID[0] = *(__IO uint32_t*)(0x1FFF7A10);
    mcuID[1] = *(__IO uint32_t*)(0x1FFF7A14);
    mcuID[2] = *(__IO uint32_t*)(0x1FFF7A18); 
	  vmc_all.board_id[0]=(mcuID[0])%100+1;
		vmc_all.board_id[1]=(mcuID[1])%100+2;
		vmc_all.board_id[2]=(mcuID[2])%100+3;
}

static int board_id_test[3]={44,16,55};
static int board_license_test[3];
static void check_lisence(void)
{
		int mcuID[3];	
		int board_id[3];

		mcuID[0] = *(__IO uint32_t*)(0x1FFF7A10);
		mcuID[1] = *(__IO uint32_t*)(0x1FFF7A14);
		mcuID[2] = *(__IO uint32_t*)(0x1FFF7A18); 

	  ukey_state=96;vmc_all.key_right=1;

}

void DigitalLPF(float in, float* out, float cutoff_freq, float dt) {
	  float input_reg=in;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        *out = input_reg;
    }
    float rc = 1.0f/(2*PI*cutoff_freq);
    float alpha = LIMIT(dt/(dt+rc), 0.0f, 1.0f);
    *out += (input_reg - *out) * alpha;
}


float fast_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}

int sign(float in)
{
  if(in>0)
		return 1;
	else
		return -1;
}
float cosd(double in)
{
return cos(in*DEG_TO_RAD);
}

float tand(double in)
{
return tan(in*DEG_TO_RAD);
}

float sind(float in)
{
return sin(in*DEG_TO_RAD);
}

float dead(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}

float cal_bear(float x2,float  y2,float  x1,float  y1)//计算1 到 2的朝向
{
float angle=0;
float y_se= y1-y2;
float x_se= x1-x2;
if (x_se==0 && y_se>0)
angle = 360;
if (x_se==0 && y_se<0)
angle = 180;
if (y_se==0 && x_se>0)
angle = 90;
if (y_se==0 && x_se<0)
angle = 270;
if (x_se>0 && y_se>0)
angle = atan(x_se/y_se)*57.3;
else if (x_se<0 && y_se>0)
angle = 360 + atan(x_se/y_se)*57.3;
else if (x_se<0 && y_se<0)
angle = 180 + atan(x_se/y_se)*57.3;
else if (x_se>0 && y_se<0)
angle = 180 + atan(x_se/y_se)*57.3;
return angle;
}

float att_to_weight(float in, float dead_in,char mode, float max)//权重计算
{
	float temp=dead(in,dead_in);
	float out;
	switch(mode)
	{
		case 0:
			out=sind(LIMIT(ABS(temp)/max,0,1)*90);
		break;
		case 1:
			out=1-sind(LIMIT(ABS(temp)/max,0,1)*90);
		break;
		default:
			out=LIMIT(ABS(temp)/max,0,1);
		break;
	}
  return out;
}

void line_function_from_two_point(float x1,float y1,float x2,float y2,float *k,float *b)
{ 
	float k_temp=0;
  *k=k_temp=(y1-y2)/(x1-x2+0.000001);
  *b=y1-k_temp*x1;
}	

void line_function_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw;
	float k_temp=0;
	if(ABS(tyaw)<0.1)
		 tyaw=0.1;
  *k=k_temp=tand(tyaw);
  *b=y-k_temp*x;
}	

void line_function90_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw;
	float k_temp=0;
  if(ABS(tyaw)<0.1)
		tyaw=0.1;
  *k=k_temp=-1/tand(tyaw);
  *b=y-k_temp*x;
}	


u8 cross_point_of_lines(float k1,float b1,float k2,float b2,float *x,float *y)
{ 
	if(ABS(k1-k2)<0.001){
		*x=*y=0;
		return 0;}
	float x_temp;
	*x=x_temp=(b1-b2)/(k2-k1+0.00001);
	*y=k1*x_temp+b1;
	return 1;
}
#if !USE_DEMO
void vcal_pwm_from_sita(VMC *in)
{ char i=0;
	for(i=0;i<2;i++){
	in->param.PWM_OUT[i]=LIMIT(in->param.PWM_OFF[i]+in->param.sita_flag[i]*in->param.sita[i]*in->param.PWM_PER_DEGREE[i],
	in->param.PWM_MIN[i],in->param.PWM_MAX[i]);
	}
}	

//舵机角度坐标系转换

void convert_mine_to_vmc_test(VMC *vmc)	
{ float limit=vmc->param.sita_limit;
	vmc->sita1=LIMIT(vmc->sita1,-limit,180-limit);
	vmc->sita2=LIMIT(vmc->sita2,limit,180+limit);
	vmc->param.sita[0]=vmc->sita1;
  vmc->param.sita[1]=(vmc->sita2);
	vcal_pwm_from_sita(vmc);
}

void convert_mine_to_vmc(VMC *vmc)	
{ float limit=vmc->param.sita_limit;
	vmc->sita1=LIMIT(vmc->sita1,-limit,180-limit);
	vmc->sita2=LIMIT(vmc->sita2,limit,180+limit);
	vmc->param.sita[0]=vmc->sita1+90;
  vmc->param.sita[1]=360-(vmc->sita2+90);
	vcal_pwm_from_sita(vmc);
}

//运动学逆解
char inv_end_state(float x,float z,float *sita1,float *sita2)
{
  float sita=-(fast_atan2(-z,x+0.00001)*RAD_TO_DEG-90);
	float r=fast_sqrt(z*z+x*x)+0.00001;
	*sita1=90-sita-acos((r*r+vmc_all.l1*vmc_all.l1-vmc_all.l2*vmc_all.l2)/(2*vmc_all.l1*r))*RAD_TO_DEG;
  *sita2=90-sita+acos((r*r+vmc_all.l1*vmc_all.l1-vmc_all.l2*vmc_all.l2)/(2*vmc_all.l1*r))*RAD_TO_DEG;
}

#endif
//计算雅克比矩阵  三角形
char cal_jacobi(VMC *in)
{
	float alfa=(in->sita2-in->sita1)/2;
	float beta=(in->sita2+in->sita1)/2;
	
	float r=in->l1*cosd(beta)+fast_sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001;
	//x
  in->jacobi[0]=-sind(beta)*r*0.5+cosd(beta)*(in->l1*sind(alfa)*0.5+0.5*in->l1*in->l1*sind(alfa)*cosd(alfa)/
	fast_sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001);
	in->jacobi[1]=-sind(beta)*r*0.5+cosd(beta)*(-in->l1*sind(alfa)*0.5-0.5*in->l1*in->l1*sind(alfa)*cosd(alfa)/
	fast_sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001);
	//z
	in->jacobi[2]=cosd(beta)*r*0.5+sind(beta)*(in->l1*sind(alfa)*0.5+0.5*in->l1*in->l1*sind(alfa)*cosd(alfa)/
	fast_sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001);
	in->jacobi[3]=cosd(beta)*r*0.5+sind(beta)*(-in->l1*sind(alfa)*0.5-0.5*in->l1*in->l1*sind(alfa)*cosd(alfa)/
	fast_sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001);
}

#if !USE_DEMO
//由力求取力矩
void cal_torque(VMC *in)
{
  in->torque[Xr]=(-in->jacobi[0]*in->force[Xr]+(-in->jacobi[2]*in->force[Zr]))*in->param.flt_toqrue+(1-in->param.flt_toqrue)*in->torque[Xr];
  in->torque[Zr]=(-in->jacobi[1]*in->force[Xr]+(-in->jacobi[3]*in->force[Zr]))*in->param.flt_toqrue+(1-in->param.flt_toqrue)*in->torque[Zr];
	
	in->param.spd_dj[0]=in->torque[Xr]*in->param.gain_torque;
  in->param.spd_dj[1]=in->torque[Zr]*in->param.gain_torque;
}

void cal_torque1(VMC *in,float flt,float dt)
{
  DigitalLPF((-in->jacobi[0]*in->force[Xr]+(-in->jacobi[2]*in->force[Zr])),&in->torque[Xr],flt,dt);
	DigitalLPF((-in->jacobi[1]*in->force[Xr]+(-in->jacobi[3]*in->force[Zr])),&in->torque[Zr],flt,dt);
	in->param.spd_dj[0]=in->torque[Xr]*in->param.gain_torque;
  in->param.spd_dj[1]=in->torque[Zr]*in->param.gain_torque;
}

static void leg_off_publish(void)
{
	if(vmc[0].param.PWM_OFF[0]>2500||vmc[0].param.PWM_OFF[0]<50||force_dj_off_reset)
	{
	vmc[0].param.PWM_OFF[0]=600;	
	vmc[0].param.PWM_OFF[1]=600;	
	vmc[1].param.PWM_OFF[0]=2400;	
	vmc[1].param.PWM_OFF[1]=2420;	
	vmc[2].param.PWM_OFF[0]=2360;	
	vmc[2].param.PWM_OFF[1]=2380;	
	vmc[3].param.PWM_OFF[0]=600;	
	vmc[3].param.PWM_OFF[1]=600;	
  force_dj_off_reset=0;
	}
}

void out_range_protect(void)
{ int i=0;
	char out=0,g_num=0;
	float err[2];
	static int cnt_out[4];
	for(i=0;i<4;i++){
	  if(vmc[i].ground==1)
			 g_num++;
	}
	for(i=0;i<4;i++){
   if(fabs(vmc[i].epos.z)<fabs(MIN_Z)*0.95){out=1;
	  vmc[i].force_cmd[Zr]=-vmc_all.out_range_force;}
   if(fabs(vmc[i].epos.z)>fabs(MAX_Z)*1.05){out=1;
    vmc[i].force_cmd[Zr]=vmc_all.out_range_force;}
	 
	 if(vmc[i].epos.x>MAX_X*1.05){out=1;
    vmc[i].force_cmd[Xr]=-vmc_all.out_range_force;}
	 if(vmc[i].epos.x<MIN_X*1.05){out=1;
    vmc[i].force_cmd[Xr]=vmc_all.out_range_force;}
	
  }
	
	for(i=0;i<4;i++){//错误超出着地复位
	 if(out==1&&vmc[i].ground==0&&g_num>2)
		   cnt_out[i]++;
	 
	 if(cnt_out[i]>5)
			vmc[i].ground=1;
	 
	 if(cnt_out[i]>0&&vmc[i].ground==1)
		  cnt_out[i]=0;
	 }
	
	out_flag=out;
	char flag_nan=0;
	for(i=0;i<4;i++)
	{
		if( isnan(vmc[i].sita1)||isnan(vmc[i].sita2))
	   flag_nan++;
	}
	#if !DEBUG_MODE		
	static float err_timer=0;
	if(flag_nan>0||
	   fabs(vmc_all.att_ctrl[PITr])>75||fabs(vmc_all.att_ctrl[ROLr])>75)
	  err_timer+=0.005;
	else
		err_timer=0;
	if(err_timer>0.05){
		err_timer=0;
		vmc_all.leg_power=0;
	  vmc_all.err=1;
	}
	#endif
}

//状态复位 和  数据滤波
float w_r_vm[2]={0.321,1};
float k_rate_vm=0.567;

static void state_rst(float dt)
{ static float st;
	static char init;
	static float time[5];
	float att_use[3],err[3],cog_off_hover,temp_end_dis;
	char i;
	if(!init)
	{
	 vmc_all.param.dt_size[1]=dt/0.005;
	}
	
	if(vmc_all.k_auto_time){
		vmc_all.gait_time[2]=vmc_all.gait_time[0]
						-dead(fabs(vmc_all.tar_spd.z),gait_change_param[0])*gait_change_param[1]//旋转
						-dead(fabs(2*(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr])-vmc_all.att_ctrl[PITr]),gait_change_param[2])*gait_change_param[3]
						-dead(fabs(vmc_all.tar_spd.x)/MAX_SPD,0.5)*gait_change_param[4];
		vmc_all.gait_time[2]=LIMIT(vmc_all.gait_time[2],gait_change_param[5],99);	
		//步频率自动调整
		DigitalLPF(vmc_all.gait_time[2],&vmc_all.gait_time[3],FLT_AUTO_T,dt);
	}else
		vmc_all.gait_time[3]=vmc_all.gait_time[0];
//	DigitalLPF(LIMIT(cosd(dead(vmc_all.att_ctrl[PITr],4)*90/16.8)*vmc_all.delay_time[0],0,vmc_all.delay_time[0]),
//	&vmc_all.delay_time[1],
//	FLT_AUTO_T,dt);	
////跨腿高度自动调整
//	DigitalLPF(LIMIT(vmc_all.delta_ht[0]*0.5-cosd(dead(vmc_all.att_ctrl[PITr],4)*90/16.8)*vmc_all.delta_ht[0]*0.25,0,vmc_all.delta_ht[0]),
//	&vmc_all.delta_h_att_off,
//	FLT_AUTO_T,dt);	
	
	
	vmc_all.delay_time[1]=vmc_all.delay_time[0];
	vmc_all.delta_h_att_off=0;

 	st=vmc_all.gait_time[1]*vmc_all.gait_alfa;
	vmc_all.stance_time_auto=st;
	vmc_all.ground_num=0;
	leg_off_publish();
	
	for(i=0;i<4;i++){
	 vmc[i].tar_spd.x=vmc_all.param.tar_spd_use_rc.x;
	 vmc[i].param.delta_h=vmc_all.delta_ht[0];
	 vmc[i].param.pid[Xr][0]=vmc_all.pid[Xr][0];
	 vmc[i].param.pid[Xr][1]=vmc_all.pid[Xr][1];
	 vmc[i].param.pid[Xr][2]=vmc_all.pid[Xr][2];	
	 vmc[i].param.pid[Xr][3]=vmc_all.pid[Xr][3];
	 vmc[i].param.pid[Xr][4]=vmc_all.pid[Xr][4];
		
	 vmc[i].param.pid[Zr][0]=vmc_all.pid[Zr][0];
	 vmc[i].param.pid[Zr][1]=vmc_all.pid[Zr][1];
	 vmc[i].param.pid[Zr][2]=vmc_all.pid[Zr][2];	
	 vmc[i].param.pid[Zr][3]=vmc_all.pid[Zr][3];	
	 vmc[i].param.pid[Zr][4]=vmc_all.pid[Zr][4];	
		
	 h_pid[i].kp=h_pid_all.kp;	  h_pid[i].ki=h_pid_all.ki;		 h_pid[i].kd=h_pid_all.kd;
	 att_pid[PITr][i].kp=att_pid_all[PITr].kp;att_pid[PITr][i].ki=att_pid_all[PITr].ki;att_pid[PITr][i].kd=att_pid_all[PITr].kd;
	 att_pid[ROLr][i].kp=att_pid_all[ROLr].kp;att_pid[ROLr][i].ki=att_pid_all[ROLr].ki;att_pid[ROLr][i].kd=att_pid_all[ROLr].kd;
	 vmc_all.kp_deng[3]=vmc_all.kp_deng[2]=vmc_all.kp_deng[1]=vmc_all.kp_deng[0];
		
	 vmc[i].param.flt_toqrue=vmc_all.flt_toqrue;
	 vmc[i].param.gain_torque=vmc_all.gain_torque;
   vmc_all.ground[0][i]=vmc[i].ground;
	 vmc_all.ground[1][i]=vmc[i].ground_s;
	 if(vmc[i].ground)
		  vmc_all.ground_num++;
  }
	
		att_use[PITr]=vmc_all.att_vm[PITr];//使用机械计算角度
		att_use[ROLr]=vmc_all.att_ctrl[ROLr];

		err[PITr]=LIMIT(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr]-att_use[PITr],-8,8);
		err[ROLr]=LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr],-8,8);
   	vmc_all.param.max_l=(fabs(MAX_Z)-fabs(MIN_Z));//最大腿长	
	  vmc_all.param.max_to_end=99;
	  for(i=0;i<4;i++){
	    temp_end_dis=ABS(MAX_Z-vmc[i].epos.z);
	  	if(temp_end_dis<vmc_all.param.max_to_end)  //最大腿长至MAX_Z
				 vmc_all.param.max_to_end=temp_end_dis;
		}
		vmc_all.param.w[Xr] = LIMIT(ABS(err[PITr])/vmc_all.param.att_limit_for_w,0,1);
		vmc_all.param.w[Yr] = LIMIT(ABS(err[ROLr])/vmc_all.param.att_limit_for_w,0,1);
		vmc_all.param.w[Zr] = LIMIT(ABS(vmc_all.tar_pos.z-vmc_all.pos.z)/(vmc_all.param.max_l*0.3),0,1);
		vmc_all.param.w[3]= LIMIT(1-MAX(vmc_all.param.w[Xr],vmc_all.param.w[Yr]),0.5,1);
	
	  vmc_all.param.dt_size[0]=0.375/(vmc_all.gait_time[1]+0.0000001);
		
		//------------------------------------机械角度解算--------------------------------
		float temp[2]={0};
		if((vmc[0].ground+vmc[2].ground!=0) &&  (vmc[1].ground+vmc[3].ground!=0)){
		temp[0]=(vmc[0].epos.z*vmc[0].ground+vmc[2].epos.z*vmc[2].ground)/(vmc[0].ground+vmc[2].ground);
		temp[1]=(vmc[1].epos.z*vmc[1].ground+vmc[3].epos.z*vmc[3].ground)/(vmc[1].ground+vmc[3].ground);	
		DigitalLPF(-fast_atan2(temp[0]-temp[1], vmc_all.H)*57.3,&vmc_all.att_vm[PITr],
		 FLT_ATT_CTRL[0],dt);
			
		temp[0]=(vmc[0].spd.z*vmc[0].ground+vmc[2].spd.z*vmc[2].ground)/(vmc[0].ground+vmc[2].ground);
		temp[1]=(vmc[1].spd.z*vmc[1].ground+vmc[3].spd.z*vmc[3].ground)/(vmc[1].ground+vmc[3].ground);	
		DigitalLPF(-(temp[0]-temp[1])/vmc_all.H*57.3*k_rate_vm,&vmc_all.att_rate_vm[PITr],
		 FLT_ATT_CTRL[2],dt);//角速度
			
		vmc_all.param.att_limit[PITr]=ABS(fast_atan2(vmc_all.param.max_l-2*1.05*vmc_all.delta_ht[0], vmc_all.H/2)*57.3);//限制幅度	
		}
		
		if((vmc[0].ground+vmc[1].ground!=0) &&  (vmc[2].ground+vmc[3].ground!=0)){
		temp[0]=(vmc[0].epos.z*vmc[0].ground+vmc[1].epos.z*vmc[1].ground)/(vmc[0].ground+vmc[1].ground);
		temp[1]=(vmc[2].epos.z*vmc[2].ground+vmc[3].epos.z*vmc[3].ground)/(vmc[2].ground+vmc[3].ground);
	  DigitalLPF(fast_atan2(temp[0]-temp[1], vmc_all.W)*57.3,&vmc_all.att_vm[ROLr],
		 FLT_ATT_CTRL[0],dt);		
			
		temp[0]=(vmc[0].spd.z*vmc[0].ground+vmc[1].spd.z*vmc[1].ground)/(vmc[0].ground+vmc[1].ground);
		temp[1]=(vmc[2].spd.z*vmc[2].ground+vmc[3].spd.z*vmc[3].ground)/(vmc[2].ground+vmc[3].ground);	
	  DigitalLPF((temp[0]-temp[1])/vmc_all.W*57.3*k_rate_vm,&vmc_all.att_rate_vm[ROLr],
		 FLT_ATT_CTRL[2],dt);//角速度	

		temp[0]=MIN_Z-vmc_all.delta_ht[0];
		temp[1]=MAX_Z+vmc_all.delta_ht[0];	
		vmc_all.param.att_limit[ROLr]=ABS(fast_atan2(vmc_all.param.max_l-2*1.05*vmc_all.delta_ht[0], vmc_all.W/2)*57.3);//限制幅度				
		}
		DigitalLPF((vmc_all.param.encoder_spd[R]-vmc_all.param.encoder_spd[L])/(vmc_all.W/2)*RAD_TO_DEG*vmc_all.param.k_mb,
		&vmc_all.att_rate_vm[YAWr],FLT_ATT_CTRL[2],dt);
		 
		//跨腿用  低通滤波
		DigitalLPF(vmc_all.att[PITr], &vmc_all.att_trig[PITr], FLT_ATT_TRIG, dt); 
		DigitalLPF(vmc_all.att[ROLr], &vmc_all.att_trig[ROLr], FLT_ATT_TRIG, dt);
		vmc_all.att_trig[YAWr]=vmc_all.att[YAWr];
		DigitalLPF(vmc_all.att_rate[PITr], &vmc_all.att_rate_trig[PITr], FLT_ATT_TRIG, dt);
		DigitalLPF(vmc_all.att_rate[ROLr], &vmc_all.att_rate_trig[ROLr], FLT_ATT_TRIG, dt);
		DigitalLPF(vmc_all.att_rate[YAWr], &vmc_all.att_rate_trig[YAWr], FLT_ATT_TRIG, dt);
		
		vmc_all.att_trig[PITr]=LIMIT(vmc_all.att_trig[PITr],-16,16);
	  vmc_all.att_trig[ROLr]=LIMIT(vmc_all.att_trig[ROLr],-16,16);
	  //姿态控制用  低通滤波 	 
		static float att_rate_flt[3];
		DigitalLPF(vmc_all.att[PITr], &vmc_all.att_ctrl[PITr], FLT_ATT_CTRL[0], dt); 
		DigitalLPF(vmc_all.att[ROLr], &vmc_all.att_ctrl[ROLr], FLT_ATT_CTRL[0], dt);
	  vmc_all.att_ctrl[YAWr]=vmc_all.att[YAWr];
		DigitalLPF(vmc_all.att_rate[PITr], &att_rate_flt[PITr], FLT_ATT_CTRL[1], dt);
		DigitalLPF(vmc_all.att_rate[ROLr], &att_rate_flt[ROLr], FLT_ATT_CTRL[1], dt);
		DigitalLPF(vmc_all.att_rate[YAWr], &att_rate_flt[YAWr], FLT_ATT_CTRL_Y, dt);
		//机体角速度观测
		#if USE_MODEL_FOR_ATT_CAL
			#if USE_ESO_OBSEVER
			ESO_2(&att_rate_eso[ROLr],att_rate_flt[ROLr],vmc_all.att_rate_vm[ROLr],dt);    
			vmc_all.att_rate_ctrl[ROLr]=att_rate_eso[ROLr].z[0];
			ESO_2(&att_rate_eso[PITr],att_rate_flt[PITr],vmc_all.att_rate_vm[PITr],dt);    
			vmc_all.att_rate_ctrl[PITr]=att_rate_eso[PITr].z[0];
			ESO_2(&att_rate_eso[YAWr],att_rate_flt[YAWr],vmc_all.att_rate_vm[YAWr],dt);    
			vmc_all.att_rate_ctrl[YAWr]=att_rate_eso[YAWr].z[0];
			#else
			vmc_all.att_rate_ctrl[PITr]=w_r_vm[0]*att_rate_flt[PITr]+(1-w_r_vm[0])*vmc_all.att_rate_vm[PITr];
			vmc_all.att_rate_ctrl[ROLr]=w_r_vm[0]*att_rate_flt[ROLr]+(1-w_r_vm[0])*vmc_all.att_rate_vm[ROLr];
			vmc_all.att_rate_ctrl[YAWr]=w_r_vm[0]*att_rate_flt[YAWr]+(1-w_r_vm[0])*vmc_all.att_rate_vm[YAWr];
			#endif				
		#else
			vmc_all.att_rate_ctrl[PITr]=att_rate_flt[PITr];vmc_all.att_rate_ctrl[ROLr]=att_rate_flt[ROLr]; vmc_all.att_rate_ctrl[YAWr]=att_rate_flt[YAWr];
		#endif
		 
			
		//姿态地形跟随残差+极坐标补偿（机头抬角度应该为+）
		 //DigitalLPF(LIMIT(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr],-20,20),
		 DigitalLPF(LIMIT(vmc_all.att_vm[PITr],-20,20)*en_fix[2],
		 //DigitalLPF(LIMIT(vmc_all.att_vm[PITr]+(vmc_all.att_vm[PITr]-vmc_all.att_ctrl[PITr]),-20,20)*en_fix[2],
		 &vmc_all.att_trig[3],FLT_POLAR,dt);
		 
		//高度估计
		if((vmc[0].ground+vmc[1].ground+vmc[2].ground+vmc[3].ground>=3)||(vmc[0].ground&&vmc[3].ground)||(vmc[1].ground&&vmc[2].ground)){
		 DigitalLPF((vmc[0].epos.z*vmc[0].ground+vmc[1].epos.z*vmc[1].ground+
								 vmc[2].epos.z*vmc[2].ground+vmc[3].epos.z*vmc[3].ground)/vmc_all.ground_num,
								&vmc_all.pos.z,
								FLT_POS_Z,dt);
	  vmc_all.pos.z=LIMIT(vmc_all.pos.z,MAX_Z*0.95,MIN_Z*1.05);
		
		 DigitalLPF(( vmc[0].force_deng[0]*vmc[0].ground+vmc[1].force_deng[0]*vmc[1].ground+
								 vmc[2].force_deng[0]*vmc[2].ground+vmc[3].force_deng[0]*vmc[3].ground)/vmc_all.ground_num,
								&vmc_all.deng_all,
								0,dt);
		}
		
		if(vmc[0].ground&&vmc[3].ground==1)
			DigitalLPF(vmc[0].epos.z/2+vmc[3].epos.z/2,&vmc_all.param.line_z[0],5,dt);
		if(vmc[1].ground&&vmc[2].ground==1)
			DigitalLPF(vmc[1].epos.z/2+vmc[2].epos.z/2,&vmc_all.param.line_z[1],5,dt);
		
		//机体速度估计  里程计
		//机械速度 x 前后
    if(vmc_all.ground_num>1){
			temp[0]=(vmc[0].spd.x*vmc[0].ground+vmc[1].spd.x*vmc[1].ground+
							 vmc[2].spd.x*vmc[2].ground+vmc[3].spd.x*vmc[3].ground)/vmc_all.ground_num;
			
		 	vmc_all.body_spd[Xr]=-temp[0];	
		//机械速度 z
			temp[0]=(vmc[0].spd.z*vmc[0].ground+vmc[1].spd.z*vmc[1].ground+
				 vmc[2].spd.z*vmc[2].ground+vmc[3].spd.z*vmc[3].ground)/vmc_all.ground_num;
			
			DigitalLPF(-temp[0], &vmc_all.body_spd[Zr], FLT_SPD_END_Z, dt);
		}

		//机械速度  左右
		DigitalLPF(vmc[0].spd.x*vmc[0].ground+vmc[1].spd.x*vmc[1].ground, &vmc_all.param.encoder_spd[R], FLT_ENCODER, dt);
		DigitalLPF(vmc[2].spd.x*vmc[2].ground+vmc[3].spd.x*vmc[3].ground, &vmc_all.param.encoder_spd[L], FLT_ENCODER, dt);
		//机械速度 rad
		vmc_all.body_spd[YAWrr]=vmc_all.att_ctrl[YAWr];
		
		if(vmc_all.param.tar_spd_use_rc.x>0)//机体偏差
		 cog_off_hover=vmc_all.cog_off[F];
		else
		 cog_off_hover=vmc_all.cog_off[B];	
		DigitalLPF(cog_off_hover, &vmc_all.param.cog_off_use[3], FLT_COG_OFF, dt) ;
//----------------------------------------------------------------------------------------	
		
  vmc[FL1].force[Xr]= vmc[FL2].force[Xr]= vmc[BL1].force[Xr]= vmc[BL2].force[Xr]=0;
  vmc[FL1].force[Zr]= vmc[FL2].force[Zr]= vmc[BL1].force[Zr]= vmc[BL2].force[Zr]=0; 
}
#endif
//估计末端状态和 速度估计
char estimate_end_state(VMC *in,float dt)//每个腿的
{
	float alfa=(in->sita2-in->sita1)/2;
	float beta=(in->sita2+in->sita1)/2;
	//逆解
	in->r=in->l1*cosd(alfa)+fast_sqrt(pow(in->l2,2)-pow(in->l1,2)*pow(sind(alfa),2));
	in->sita=beta;
	in->epos.x=cosd(in->sita)*in->r;
	in->epos.z=-sind(in->sita)*in->r;
	
	float d_sita=0; 
	float d_alfa=0;

	in->param.spd_est_cnt+=dt;
	if(in->param.spd_est_cnt>vmc_all.param.end_sample_dt){
		in->epos_reg.x=in->epos.x;
		in->epos_reg.z=in->epos.z;
		
		//---------------------------角度微分-------------------------
		d_sita=(beta-in->sita_reg[0])/in->param.spd_est_cnt/57.3;
		d_alfa=(alfa-in->sita_reg[1])/in->param.spd_est_cnt/57.3;
		//d_sita=(in->param.spd_dj[1]+in->param.spd_dj[0])/2/57.3;
		//d_alfa=(in->param.spd_dj[1]-in->param.spd_dj[0])/2/57.3;
		float temp=fast_sqrt(pow(in->l2,2)-pow(in->l1,2)*pow(sind(alfa),2));
		float dr_dt=(-in->l1*sind(alfa)*d_alfa-(pow(in->l1,2)*sind(alfa)*cosd(alfa)*d_alfa)/(temp+0.000001));
		float d_x=-sind(in->sita)*d_sita*in->r+cosd(in->sita)*dr_dt;
		float d_z=-cosd(in->sita)*d_sita*in->r-sind(in->sita)*dr_dt;
    in->sita_reg[0]=beta;
		in->sita_reg[1]=alfa;
		
		if(in->ground){
			in->spd_o.x=-LIMIT(d_x,-1,1);
			in->spd_o.z= LIMIT(d_z,-1,1);
		}
		in->param.spd_est_cnt=0;	
	}	
	DigitalLPF(in->spd_o.x, &in->spd.x, FLT_SPD_END, dt) ;
  DigitalLPF(in->spd_o.z, &in->spd.z, FLT_SPD_END, dt) ;
}
#if !USE_DEMO
//计算落足点
static void cal_tar_end_pos(VMC *in)
{ 
	float att_off=0;
	float cog_off=0;
	float spd_off=0,acc_off=0;
	float size_off=0;
	float tar_spd;
	float cog_off_hover=0;
	
	cog_off_hover=vmc_all.param.cog_off_use[3];//机体偏差
	
	if(in->flag_rl==1)
		tar_spd=vmc_all.param.tar_spd_use[L].x;
	else
		tar_spd=vmc_all.param.tar_spd_use[R].x;	

	if(in->param.id==0||in->param.id==2)//前后对外扩偏差
		size_off=vmc_all.off_leg_dis[0];
	if(in->param.id==1||in->param.id==3)
		size_off=vmc_all.off_leg_dis[1];
	
	cog_off=LIMIT(0.1*vmc_all.mess*9.81*cog_off_hover
	        ,MIN_X*0.35,MAX_X*0.35);//重心补偿
	
	att_off=LIMIT(tan((vmc_all.att_trig[PITr])*DEG_TO_RAD)*vmc_all.pos.z
					,MIN_X*0.6,MAX_X*0.6);//姿态地形跟随
	
	spd_off=vmc_all.kp_trig[0]*(tar_spd-in->spd.x);
	acc_off=vmc_all.kp_trig[1]*vmc_all.acc[Xr];
  in->tar_epos.x=LIMIT(in->spd.x*(vmc_all.stance_time+vmc_all.delay_time[2]+vmc_all.gait_delay_time)/2+
	                     spd_off+//速度前馈
											 acc_off+
	                     att_off+//地形跟随偏差
	                     cog_off+
											 size_off//前后设置的固定偏差
	                     ,MIN_X,MAX_X);
	
  in->tar_epos.z=LIMIT(in->tar_epos.z,MAX_Z,MIN_Z);
	#if USE_ATT_POLAR
		float pit_err=vmc_all.att_trig[3];//姿态补偿
		float k1,b1;
		line_function_from_arrow(in->epos.x,in->epos.z,90-pit_err,&k1,&b1);
		float k2,b2;
		line_function_from_two_point(0,0,in->tar_epos.x,in->tar_epos.z,&k2,&b2);
		//line_function_from_arrow(in->tar_epos.x,in->tar_epos.z,90-(90-pit_err),&k2,&b2);
		char good=0;
		float tempx,tempz;
		good=cross_point_of_lines(k1,b1,k2,b2,&tempx,&tempz);
		if(good&&en_fix[0]){
			in->tar_epos.x=LIMIT(in->tar_epos.x*en_fix[4]+(1-en_fix[4])*tempx,MIN_X,MAX_X);
			in->tar_epos.z=LIMIT(in->tar_epos.z*en_fix[4]+(1-en_fix[4])*tempz,MAX_Z,MIN_Z);
		}
	#endif
	in->ground=0;
	vmc_all.param.cog_off_use[0]=cog_off;
	vmc_all.param.cog_off_use[1]=att_off;
}

//---------------------------------------------------------------------------------------------------
//-------------------------------------------控制器-------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------站立时的控制器-----------------------------------

void base_reset(VMC *in,float dt)
{
	char id=in->param.id;
  static char reg[4];
  if(reg[id]==0&&in->ground==1){//着地
    in->e_pos_base[0].z=in->epos.z;
		h_pid[id].p=h_pid[id].d=h_pid[in->param.id].i=0;
		extend_pid[id].i=0;
		att_pid[ROLr][id].p=att_pid[ROLr][id].d=att_pid[ROLr][id].i=0;
		att_pid[PITr][id].p=att_pid[PITr][id].d=att_pid[PITr][id].i=0;
	}
	if(reg[id]==1&&in->ground==0){//抬腿   仅对真实传感器发布偏差使用
    in->e_pos_base[1].z=in->epos.z;
		in->check_epos.z=0;
	}	
	reg[id]=in->ground;
}
	
//姿态控制
ESO att_eso[4];
float ESO_PARAM[2]={4000,1};
float attitude_control1(VMC *in,float dt)
{
	char id=in->param.id;
	float att_use[3],rate_use[3];
	float err[2];
	float t_size=vmc_all.param.dt_size[1];
	
	att_eso[id].r0=ESO_PARAM[0];//跟踪速度
  att_eso[id].h0=dt*ESO_PARAM[1];//滤波因子
	
 //控制反馈选择
	if(vmc_all.unmove==0||DEBUG_MODE)
		att_use[PITr]=vmc_all.att_vm[PITr];//使用机械计算角度
	else
		att_use[PITr]=vmc_all.att_ctrl[PITr];//静止时进入姿态平滑模式
	rate_use[PITr]=LIMIT(vmc_all.att_rate_ctrl[PITr],-22,22);
	
	#if DEBUG_MODE
	att_use[ROLr]=vmc_all.att_vm[ROLr];
	#else
	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	#endif
	rate_use[ROLr]=-LIMIT(vmc_all.att_rate_ctrl[ROLr],-22,22);
 
	err[PITr]= LIMIT(LIMIT(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr],-vmc_all.param.att_limit[PITr],vmc_all.param.att_limit[PITr])
						-att_use[PITr],-16,16);
	err[ROLr]=-LIMIT(LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr],-vmc_all.param.att_limit[ROLr],vmc_all.param.att_limit[ROLr])
						-att_use[ROLr],-16,16);
	
  //Pit
	att_pid[PITr][id].err=err[PITr];
	att_pid[PITr][id].i+=-in->flag_fb*0.001*sind(vmc_all.H/2*(att_pid[PITr][id].err)*att_pid[PITr][id].ki)*t_size;
	att_pid[PITr][id].d+=in->flag_fb*0.001*rate_use[PITr]*att_pid[PITr][id].kd*t_size;
	att_pid[PITr][id].i=LIMIT(att_pid[PITr][id].i,-0.6*vmc_all.param.max_l,0.6*vmc_all.param.max_l);
	att_pid[PITr][id].d=LIMIT(att_pid[PITr][id].d,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);
	att_pid[PITr][id].out=att_pid[PITr][id].i+att_pid[PITr][id].d;
	att_pid[PITr][id].out=LIMIT(att_pid[PITr][id].out,-0.6*vmc_all.param.max_l,0.6*vmc_all.param.max_l);
	//Rol
	att_pid[ROLr][id].err=err[ROLr];
	att_pid[ROLr][id].i+=-in->flag_rl*0.001*sind(vmc_all.W/2*(att_pid[ROLr][id].err)*att_pid[ROLr][id].ki)*t_size;
	att_pid[ROLr][id].d+=in->flag_rl*0.001*rate_use[ROLr]*att_pid[ROLr][id].kd*t_size;
	att_pid[ROLr][id].i=LIMIT(att_pid[ROLr][id].i, -0.6*vmc_all.param.max_l,0.6*vmc_all.param.max_l);
	att_pid[ROLr][id].d=LIMIT(att_pid[ROLr][id].d,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);
	att_pid[ROLr][id].out=att_pid[ROLr][id].i+att_pid[ROLr][id].d;
	att_pid[ROLr][id].out=LIMIT(att_pid[ROLr][id].out,-0.6*vmc_all.param.max_l,0.6*vmc_all.param.max_l);
}

float max_rate1=0.35;
float test_att_pid[5]={0  ,0,  40,3.5  , 0};
float attitude_control_all(float dt)//全局姿态控制
{
	float att_use[3],rate_use[3];
	float err[2];
	float t_size=vmc_all.param.dt_size[1];
	static float cnt_outterr,cnt_outterp;
	char i;
 //控制反馈选择
	if(vmc_all.unmove==0)
		att_use[PITr]=vmc_all.att_vm[PITr];//使用机械计算角度
	else
		att_use[PITr]=vmc_all.att_ctrl[PITr];//静止时进入姿态平滑模式
	rate_use[PITr]=LIMIT(vmc_all.att_rate_ctrl[PITr],-22,22);
	 
	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	rate_use[ROLr]=LIMIT(vmc_all.att_rate_ctrl[ROLr],-22,22);
 
	err[PITr]=LIMIT(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr]-att_use[PITr],-16,16);
	err[ROLr]=LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr],-16,16);
	
  //Pit
	cnt_outterp+=dt;
	if(cnt_outterp>0.01)
	{
		att_pid_outter_all[PITr].err=sind(err[PITr]);
		att_pid_outter_all[PITr].p=att_pid_outter_all[PITr].err*att_pid_outter_all[PITr].kp;
		att_pid_outter_all[PITr].d=(att_pid_outter_all[PITr].err-att_pid_outter_all[PITr].err_reg)*att_pid_outter_all[PITr].kd;
		att_pid_outter_all[PITr].out=att_pid_outter_all[PITr].p+att_pid_outter_all[PITr].d;
		att_pid_outter_all[PITr].out=LIMIT(att_pid_outter_all[PITr].out,-MAX_SPD_RAD*0.5,MAX_SPD_RAD*0.5);
	  att_pid_outter_all[PITr].err_reg=att_pid_outter_all[PITr].err;
		cnt_outterp=0;
	}
	att_pid_inner_all[PITr].exp=att_pid_outter_all[PITr].out;
	att_pid_inner_all[PITr].err=att_pid_inner_all[PITr].exp-rate_use[PITr];
	att_pid_inner_all[PITr].p= 	 0.001*vmc_all.H/2*(att_pid_inner_all[PITr].err)*att_pid_inner_all[PITr].kp;
	att_pid_inner_all[PITr].fpp= 0.001*vmc_all.H/2*(att_pid_inner_all[PITr].exp)*att_pid_inner_all[PITr].fp;
	att_pid_inner_all[PITr].fpp=LIMIT(att_pid_inner_all[PITr].fpp, -0.1*vmc_all.param.max_l,0.1*vmc_all.param.max_l);
	att_pid_inner_all[PITr].d=-0.001*vmc_all.H/2*0.0005*((rate_use[PITr]-att_pid_inner_all[PITr].err_reg)/dt)
																	 *att_pid_inner_all[PITr].kd;
	att_pid_inner_all[PITr].p=LIMIT(att_pid_inner_all[PITr].p, -0.1*vmc_all.param.max_l,0.1*vmc_all.param.max_l);
	att_pid_inner_all[PITr].d=LIMIT(att_pid_inner_all[PITr].d,-0.05*vmc_all.param.max_l,0.05*vmc_all.param.max_l);
	att_pid_inner_all[PITr].out=att_pid_inner_all[PITr].fpp+att_pid_inner_all[PITr].p+att_pid_inner_all[ROLr].d;
	att_pid_inner_all[PITr].out=LIMIT(att_pid_inner_all[PITr].out,-0.1*vmc_all.param.max_l,0.1*vmc_all.param.max_l);
	att_pid_inner_all[PITr].err_reg=rate_use[PITr];

	//Rol
	cnt_outterr+=dt;
	if(cnt_outterr>0.01)
	{
		att_pid_outter_all[ROLr].err=sind(err[ROLr]);
		att_pid_outter_all[ROLr].p=att_pid_outter_all[ROLr].err*att_pid_outter_all[ROLr].kp;
		att_pid_outter_all[ROLr].d=(att_pid_outter_all[ROLr].err-att_pid_outter_all[ROLr].err_reg)/cnt_outterr*att_pid_outter_all[ROLr].kd;
		//att_pid_outter_all[ROLr].d=(-rate_use[ROLr])*att_pid_outter_all[ROLr].kd;
		att_pid_outter_all[ROLr].out=att_pid_outter_all[ROLr].p+att_pid_outter_all[ROLr].d;
		att_pid_outter_all[ROLr].out=LIMIT(att_pid_outter_all[ROLr].out,-MAX_SPD_RAD*max_rate1,MAX_SPD_RAD*max_rate1);
	  att_pid_outter_all[ROLr].err_reg=att_pid_outter_all[ROLr].err;
		cnt_outterr=0;
	}
	//Double
	if(test_att_pid[0]){
	test_att_pid[1]+=dt*test_att_pid[2];
	test_att_pid[4]=sind(test_att_pid[1])*test_att_pid[3];
	  if(test_att_pid[1]>360)
			test_att_pid[1]=0;
	att_pid_inner_all[ROLr].exp=test_att_pid[4];	
	}else
	att_pid_inner_all[ROLr].exp=att_pid_outter_all[ROLr].out;
	
#if DOUBLE_LOOP	
	att_pid_inner_all[ROLr].err=att_pid_inner_all[ROLr].exp-rate_use[ROLr];
	att_pid_inner_all[ROLr].p= 	 0.001*vmc_all.W/2*(att_pid_inner_all[ROLr].err)*att_pid_inner_all[ROLr].kp;
	att_pid_inner_all[ROLr].fpp= 0.001*vmc_all.W/2*(att_pid_inner_all[ROLr].exp)*att_pid_inner_all[ROLr].fp;
	
	att_pid_inner_all[ROLr].i+=0.001*att_pid_inner_all[ROLr].err*vmc_all.W/2*att_pid_inner_all[ROLr].ki*dt;
	att_pid_inner_all[ROLr].i=LIMIT(att_pid_inner_all[ROLr].i,-0.05*vmc_all.param.max_l,0.05*vmc_all.param.max_l);
	
	att_pid_inner_all[ROLr].d=-0.001*vmc_all.W/2*0.0005*((rate_use[ROLr]-att_pid_inner_all[ROLr].err_reg)/dt)
																	 *att_pid_inner_all[ROLr].kd;
	att_pid_inner_all[ROLr].p=LIMIT(att_pid_inner_all[ROLr].p, -0.2*vmc_all.param.max_l,0.2*vmc_all.param.max_l);
	att_pid_inner_all[ROLr].fpp=LIMIT(att_pid_inner_all[ROLr].fpp, -0.1*vmc_all.param.max_l,0.1*vmc_all.param.max_l);
	att_pid_inner_all[ROLr].d=LIMIT(att_pid_inner_all[ROLr].d,-0.1*vmc_all.param.max_l,0.1*vmc_all.param.max_l);
	att_pid_inner_all[ROLr].out=att_pid_inner_all[ROLr].fpp+att_pid_inner_all[ROLr].p+att_pid_inner_all[ROLr].i+att_pid_inner_all[ROLr].d;
	att_pid_inner_all[ROLr].out=LIMIT(att_pid_inner_all[ROLr].out,-0.25*vmc_all.param.max_l,0.25*vmc_all.param.max_l);
	att_pid_inner_all[ROLr].err_reg=rate_use[ROLr];
#else
	//Single
	att_pid_inner_all[ROLr].err=err[ROLr];
	att_pid_inner_all[ROLr].p=0.0001*vmc_all.W/2*(att_pid_inner_all[ROLr].err)*att_pid_inner_all[ROLr].kp;
	att_pid_inner_all[ROLr].d=-0.001*rate_use[ROLr]*att_pid_inner_all[ROLr].kd;
	att_pid_inner_all[ROLr].p=LIMIT(att_pid_inner_all[ROLr].p, -0.26*vmc_all.param.max_l,0.26*vmc_all.param.max_l);
	att_pid_inner_all[ROLr].d=LIMIT(att_pid_inner_all[ROLr].d,-0.1*vmc_all.param.max_l,0.1*vmc_all.param.max_l);
	att_pid_inner_all[ROLr].out=att_pid_inner_all[ROLr].p+att_pid_inner_all[ROLr].d;
	att_pid_inner_all[ROLr].out=LIMIT(att_pid_inner_all[ROLr].out,-0.26*vmc_all.param.max_l,0.26*vmc_all.param.max_l);
#endif
}

void height_control1(VMC *in,float dt)
{
	char id=in->param.id;
	float out;
	float t_size=vmc_all.param.dt_size[1];
	h_pid[id].err=LIMIT(vmc_all.tar_pos.z+vmc_all.deng_all*0.0001-vmc_all.pos.z,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);
	h_pid[id].i+=h_pid[id].err*h_pid[id].ki*0.001*t_size;
	h_pid[id].d+=vmc_all.body_spd[Zr]*h_pid[id].kd*0.001*t_size;
	h_pid[id].i=LIMIT(h_pid[id].i,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);
	h_pid[id].d=LIMIT(h_pid[id].d,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);	
	h_pid[id].out=h_pid[id].i+h_pid[id].d;
	h_pid[id].out=LIMIT(h_pid[id].out,-0.4*vmc_all.param.max_l,0.4*vmc_all.param.max_l);

	if(id==0||id==3)
		h_pid[id].p+=-(vmc_all.param.line_z[0]-vmc_all.param.line_z[1])*h_pid[id].kp*0.001*t_size;
	if(id==2||id==1)
		h_pid[id].p+= (vmc_all.param.line_z[0]-vmc_all.param.line_z[1])*h_pid[id].kp*0.001*t_size;
  h_pid[id].p=LIMIT(h_pid[id].p,-0.4*vmc_all.param.max_l,0.4*vmc_all.param.max_l);
}

void height_control_all(float dt)//全局高度控制
{
	float out;
	float t_size=vmc_all.param.dt_size[1];

	h_pid_all.err=LIMIT(vmc_all.tar_pos.z+vmc_all.deng_all*0.0001-vmc_all.pos.z,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);
	h_pid_all.i=h_pid_all.err*h_pid_all.ki*0.001;
	h_pid_all.d=vmc_all.body_spd[Zr]*h_pid_all.kd*0.001;
	h_pid_all.i=LIMIT(h_pid_all.i,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);
	h_pid_all.d=LIMIT(h_pid_all.d,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);

	h_pid_all.out=h_pid_all.i+h_pid_all.d;
	h_pid_all.out=LIMIT(h_pid_all.out,-0.5*vmc_all.param.max_l,0.5*vmc_all.param.max_l);
	
	h_pid_all.p=(vmc_all.param.line_z[0]-vmc_all.param.line_z[1])*h_pid_all.kp*0.001;
	h_pid_all.p=LIMIT(h_pid_all.p,-0.5*vmc_all.param.max_l,0.5*vmc_all.param.max_l);
}

float k_nground=0.6;
float k_att=6;
void leg_tar_publish(VMC *in,float dt)
{
	char id=in->param.id;
	float base_z=in->e_pos_base[0].z;
	vmc_all.param.weight[ROLr]=LIMIT(1-ABS(LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-vmc_all.att_ctrl[ROLr],
	-k_att,k_att))/k_att+k_nground,0,1);//横滚轴权重分离
	vmc_all.param.weight[PITr]=LIMIT(1-ABS(LIMIT(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr]-vmc_all.att_ctrl[PITr],
	-k_att,k_att))/k_att+k_nground,0,1);//横滚轴权重分离	

	#if USE_ATT_POLAR
		float pit_err=-vmc_all.att_trig[3];//姿态补偿
	  float tempx,tempz; 
	  tempx=tand(pit_err)*in->e_pos_base[0].z;
		float k1,b1;
		line_function_from_arrow(tempx,in->e_pos_base[0].z,90-pit_err,&k1,&b1);
		float k2,b2;
		//line_function_from_two_point(0,0,tempx,in->e_pos_base[0].z,&k2,&b2);
		line_function_from_arrow(in->tar_epos.x,in->e_pos_base[0].z,90-(90-pit_err),&k2,&b2);
		char good=0;
		good=cross_point_of_lines(k1,b1,k2,b2,&tempx,&tempz);
		if(good&&en_fix[1])
			base_z=LIMIT(tempz,MAX_Z,MIN_Z);
	#endif
		
	#if GLOBAL_CONTROL
		att_pid[PITr][id].out+=-in->flag_fb*att_pid_inner_all[PITr].out*dt/0.005;
		att_pid[ROLr][id].out+=in->flag_rl*att_pid_inner_all[ROLr].out*dt/0.005;
		h_pid[id].out+=h_pid_all.out*dt/0.005;
		if(id==0||id==3)
			h_pid[id].p+=-h_pid_all.p*dt/0.005;
		if(id==2||id==1)
			h_pid[id].p+= h_pid_all.p*dt/0.005;
		
		h_pid[id].out=LIMIT(h_pid[id].out,-0.6*vmc_all.param.max_l,0.6*vmc_all.param.max_l);
		h_pid[id].p=LIMIT(h_pid[id].p,-0.6*vmc_all.param.max_l,0.6*vmc_all.param.max_l);
		att_pid[PITr][id].out=LIMIT(att_pid[PITr][id].out,-0.6*vmc_all.param.max_l,0.6*vmc_all.param.max_l);
		att_pid[ROLr][id].out=LIMIT(att_pid[ROLr][id].out,-0.6*vmc_all.param.max_l,0.6*vmc_all.param.max_l);
		
		in->tar_epos.z= LIMIT(
										base_z
										+att_pid[PITr][id].out
										+att_pid[ROLr][id].out
										+h_pid[id].out*vmc_all.param.weight[ROLr]
										+h_pid[id].p
										,MAX_Z+vmc_all.delta_ht[0]*0.68,MIN_Z-vmc_all.delta_ht[0]*1.345)
										+LIMIT(in->force_deng[0]*0.0001,-vmc_all.param.max_to_end,99)
										;
	#else
		in->tar_epos.z= LIMIT(
										base_z
										+att_pid[PITr][id].out
										+att_pid[ROLr][id].out
										+h_pid[id].out*vmc_all.param.weight[ROLr]
										+h_pid[id].p	
										+extend_pid[id].i
										,MAX_Z+vmc_all.delta_ht[0]*0.68,MIN_Z-vmc_all.delta_ht[0]*1)
										
										+LIMIT(in->force_deng[0]*0.0001,-vmc_all.param.max_to_end,99)
										;
	#endif
	in->tar_epos.z=LIMIT(in->tar_epos.z,MAX_Z*0.95,MIN_Z*1.05);
}
//真实姿态参数修正着地腿
float gain_check[5]={1,0,0,0,0};
void leg_check_publish(VMC *in,float dt)
{
	char id=in->param.id;
	in->check_epos.z+=-vmc_all.body_spd[Zr]*dt*gain_check[0]
									  -gain_check[1]*in->flag_fb*vmc_all.att_rate_ctrl[PITr]*vmc_all.H/4*DEG_TO_RAD*dt;
	
	in->tar_epos.z=in->e_pos_base[1].z+in->check_epos.z;
	in->tar_epos.z=LIMIT(in->tar_epos.z,MAX_Z*0.95,MIN_Z*1.05);
}

void yaw_control(float dt){
	//---------------YAW control
	static float rst_yaw[2];
	static float timer;
	if(vmc_all.param.have_cmd==0)
		rst_yaw[0]+=dt;
	else
		rst_yaw[0]=rst_yaw[1]=0;
	
	if(rst_yaw[0]>3)
		rst_yaw[1]=1;
	
	if(rst_yaw[1]||vmc_all.unmove||vmc_all.hand_hold||
		 vmc_all.param.have_cmd_rc[1]||vmc_all.param.have_cmd_sdk[1][YAWrr])//无控制复位
	  vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
	
	if(vmc_all.param.have_cmd_sdk[1][YAWr])//SDK输出
	  vmc_all.tar_att[YAWr]=vmc_all.param.tar_att_on[2];
	
	static float err_yaw,yaw_out;
	DigitalLPF(LIMIT(To_180_degrees(dead(vmc_all.tar_att[YAWr]-vmc_all.att_ctrl[YAWr],0.6)),-20,20), 
		&err_yaw, FLT_ATT_CTRL_Y, dt);

	vmc_all.param.w_cmd[YAWr]=LIMIT(err_yaw/25,0,1);//计算权重
	//外环
	static float exp_yaw_rate=0;
	timer+=dt;
	if(timer>0.05){
		exp_yaw_rate=LIMIT(-att_pid_all[YAWr].kp*err_yaw,-MAX_SPD_RAD,MAX_SPD_RAD);
		timer=0;
	}		

	//内环
	if(vmc_all.param.have_cmd_rc[1]||vmc_all.param.have_cmd_sdk[1][YAWrr])//外部遥控介入
	  exp_yaw_rate=vmc_all.tar_spd.z;
		
	exp_yaw_rate*=LIMIT(1-vmc_all.param.w_cmd[ROLr],0.5,1);
	
	static float rate_reg;
	float damp = (vmc_all.att_rate_ctrl[YAWr]- rate_reg) *( 0.005f/dt );
	yaw_out=LIMIT(dead(exp_yaw_rate-vmc_all.att_rate_ctrl[YAWr],0.5),-MAX_SPD_RAD,MAX_SPD_RAD)*att_pid_all[YAWr].kp_i+
								 -damp*att_pid_all[YAWr].kd_i+
	               exp_yaw_rate*att_pid_all[YAWr].fp_i;//前馈
	rate_reg=vmc_all.att_rate_ctrl[YAWr];

	if(vmc_all.hand_hold||block_yaw||DEBUG_MODE)
		yaw_out=0;
	
	//控制差速输出
	vmc_all.param.tar_spd_use[L].x=vmc_all.param.tar_spd_use_rc.x+yaw_out;
	vmc_all.param.tar_spd_use[R].x=vmc_all.param.tar_spd_use_rc.x-yaw_out;
	
	//旋转造成的姿态倾斜
	if(fabs(vmc_all.param.tar_spd_use_rc.x)>MAX_SPD*0.2&&fabs(vmc_all.tar_spd.z)>MAX_SPD_RAD*0.1)
		if(vmc_all.param.tar_spd_use_rc.x>0)
		vmc_all.tar_att_off[ROLr]=vmc_all.cog_off[3]*sign(vmc_all.param.tar_spd_use_rc.x)*LIMIT(exp_yaw_rate,-MAX_SPD_RAD*0.35,MAX_SPD_RAD*0.35)/(MAX_SPD_RAD*0.35+0.00001);
	  else
		vmc_all.tar_att_off[ROLr]=vmc_all.cog_off[4]*sign(vmc_all.param.tar_spd_use_rc.x)*LIMIT(exp_yaw_rate,-MAX_SPD_RAD*0.35,MAX_SPD_RAD*0.35)/(MAX_SPD_RAD*0.35+0.00001);
	else
		vmc_all.tar_att_off[ROLr]=0;
}	
	
float k_att_spd=25;
void spd_control(float dt)//机体速度控制
{	
	float att_use[3];
	att_use[PITr]=vmc_all.att_vm[PITr];
	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	vmc_all.param.w_cmd[ROLr]=LIMIT(ABS(LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr],
	-k_att_spd,k_att_spd))/k_att_spd,0,1);
	vmc_all.param.w_cmd[PITr]=LIMIT(ABS(LIMIT(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr]-att_use[PITr],
	-k_att_spd,k_att_spd))/k_att_spd,0,1);
	
	vmc_all.param.w_cmd[4]=LIMIT((ABS(vmc_all.pos.z)-ABS(MIN_Z))/vmc_all.param.max_l+0.35,0.5,1);//高度对速度的限制幅度
	//速度权重分配输出
	vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x//总期望
													*LIMIT(1-vmc_all.param.w_cmd[YAWr],0.35,1)
													*LIMIT(1-vmc_all.param.w_cmd[ROLr],0.5,1)//?
													*vmc_all.param.w_cmd[4];//高度
	
	//vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x;//不使用权重分配速度
	vmc_all.param.tar_spd_use_rc.z=vmc_all.tar_spd.z;
	//移动时俯仰角设定
	vmc_all.tar_att_off[PITr]=LIMIT(vmc_all.param.tar_spd_use_rc.x,-MAX_SPD,MAX_SPD)/(MAX_SPD+0.00001)*vmc_all.cog_off[2];
}	

		
void pos_control(float dt)
{
	if(vmc_all.param.smart_control_mode[0]==MODE_POS&&vmc_all.param.smart_control_mode[0]!=MODE_SPD){//位置
			if(vmc_all.param.tar_pos_on.x!=0)
					vmc_all.tar_pos.x=vmc_all.param.tar_pos_on.x;
			if(vmc_all.param.tar_pos_on.y!=0)
					vmc_all.tar_pos.y=vmc_all.param.tar_pos_on.y;

			//yaw
			float tar_yaw=cal_bear(vmc_all.pos_n.x,vmc_all.pos_n.y,vmc_all.tar_pos.x,vmc_all.tar_pos.y);
			tar_yaw=To_180_degrees(tar_yaw);	
			float err_yaw=To_180_degrees(tar_yaw-vmc_all.att_ctrl[YAWr]);
			vmc_all.param.smart_control_mode[2]=MODE_ATT_YAW_ONLY;//姿态角
			vmc_all.param.tar_att_on[YAWr]=tar_yaw;		
					
			float w_yaw=1-LIMIT(fabs(dead(err_yaw,6)),-YAW_POS_MAX,YAW_POS_MAX)/YAW_POS_MAX;	
			int flag_spd=0;
			if(w_yaw<90&&w_yaw>-90)
				flag_spd=1;
			else 
				flag_spd=-1;
			//spd
			pos_pid[Xr].err=dead(vmc_all.tar_pos.x-vmc_all.pos_n.x,POS_DEAD*0.8);
			pos_pid[Yr].err=dead(vmc_all.tar_pos.y-vmc_all.pos_n.y,POS_DEAD*0.8);
			pos_pid_all.err=my_sqrt(my_pow(pos_pid[Xr].err)
								 +my_pow(pos_pid[Yr].err));	
			
			pos_pid_all.p=pos_pid_all.err*pos_pid_all.kp;
			
			pos_pid_all.i+=pos_pid_all.err*pos_pid_all.ki;
			pos_pid_all.i=LIMIT(pos_pid_all.i,-0.3*MAX_SPD,0.3*MAX_SPD);
			
			pos_pid_all.out=(pos_pid_all.p+pos_pid_all.i)*w_yaw*flag_spd;		
			pos_pid_all.out=LIMIT(pos_pid_all.out,-MAX_SPD,MAX_SPD);
			DigitalLPF(pos_pid_all.out,&vmc_all.param.tar_spd_on.x,3,dt);
			vmc_all.param.tar_spd_on.z=0;
			
			if(pos_pid_all.err>POS_DEAD*0.8||fabs(err_yaw)>2){
				vmc_all.gait_on=1;
				vmc_all.unmove=0;
			}
	}
	else
	{
			vmc_all.tar_pos.x=vmc_all.pos_n.x;
			vmc_all.tar_pos.y=vmc_all.pos_n.y;
		  pos_pid_all.out=0;
	}
}


//----------------------------------------------VMC Force Controll--------------------
//触地力控制
void deng_control(VMC *in,float dt)
{
	char i;
	float out;
	float att_use[3],rate_use[3];
	float err[2];
	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	rate_use[ROLr]=vmc_all.att_rate_ctrl[ROLr];

	err[ROLr]=LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr],-8,8);
	
	out=-in->flag_rl*(vmc_all.kd_deng[P]*vmc_all.kp_deng[FL1]*err[ROLr]+
		               vmc_all.kd_deng[D]*LIMIT(0-rate_use[ROLr],-MAX_SPD_RAD*0.68,MAX_SPD_RAD*0.68));//Rol
	
	vmc_all.kp_deng_ctrl[0][in->param.id]=LIMIT(out,0,vmc_all.kp_deng[in->param.id]*1.25);
//vmc_all.kp_deng_ctrl[0][in->param.id]=LIMIT(out,-vmc_all.kp_deng[in->param.id]*1.25,vmc_all.kp_deng[in->param.id]*1.25);
}	

//计算Y轴虚拟力
float cal_force_y(VMC *in,float dt)//前后
{
 static float interge;
 float cog_off[2],tar_spd;	
 static float err[4][2];
	//旋转差速
	if(in->flag_rl==1)
		tar_spd=vmc_all.param.tar_spd_use[L].x;
	else
		tar_spd=vmc_all.param.tar_spd_use[R].x;	
	
 cog_off[Yr]=-sind(dead(-vmc_all.att_trig[PITr],2))
   *vmc_all.mess*9.81*vmc_all.kp_g[Yr];//倾斜重力补偿
 
 err[in->param.id][0]=tar_spd-in->spd.x;
 DigitalLPF( err[in->param.id][0], &err[in->param.id][1], FLT_ATT_SPD_ERR, dt);
	
 interge+=(err[in->param.id][1])*in->param.pid[Xr][I]*dt;
 interge=LIMIT(interge,-100,100);
 if(fabs(tar_spd)<0.05*MAX_SPD)
	 interge=0;
	 
 if(vmc_all.hand_hold||vmc_all.unmove)//静止下足尖归中
	 in->force_cmd[Xr]=-vmc_all.param.kp_pose_reset[0]*(sind(vmc_all.att_trig[PITr])*in->epos.z
																																									-vmc_all.param.jump_out[Yr]
																																									-in->epos.x);
 else
	 in->force_cmd[Xr]=in->param.pid[Xr][P]*(err[in->param.id][1])//速度前馈
			+interge//速度误差积分
			+cog_off[Xr]
			+cog_off[Yr]//姿态修正
			-vmc_all.param.kp_pose_reset[1]*(sind(vmc_all.att_trig[PITr])*in->epos.z-in->epos.x);
 
 if(fabs(in->epos.z)<fabs(MIN_Z)||fabs(in->epos.z)>fabs(MAX_Z)||fabs(in->epos.x)>MAX_X)
	 in->force_cmd[Xr]*=0.1;
}

//计算Z轴虚拟力
float cal_force_z(VMC *in,float dt)//高度
{
 char i;
 float att_off[2],cog_off[2],ground_off=0;
 float z_out;
 cog_off[Xr]=sind(dead(vmc_all.att_trig[ROLr],2))
   *vmc_all.mess*9.81*vmc_all.kp_g[Yr];//倾斜重力补偿
	
 in->force_cmd[Zr]=cog_off[Xr]+
							 in->param.pid[Zr][P]*(in->tar_epos.z-in->epos.z)+in->param.pid[Zr][D]*(0-in->spd.z);
 
 if(fabs(in->epos.z)<fabs(MIN_Z)||fabs(in->epos.z)>fabs(MAX_Z)||fabs(in->epos.x)>MAX_X)
	 in->force_cmd[Zr]*=0.1;
}


void cal_force_deng(VMC *in,float dt){
	char id=in->param.id;
	float divid_time_check=deng_param[1]*vmc_all.stance_time;
		
	//姿态蹬腿控制
	deng_control(in,dt);
	vmc_all.kp_deng_ctrl[1][id]=vmc_all.kp_deng_ctrl[0][id];//
	
	float deng_control_out=vmc_all.kp_deng[id]//Gravity 
												 +vmc_all.kp_deng_ctrl[1][id]//ROLL Control
													;
	float pit_err=(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr])-vmc_all.att_ctrl[3];//姿态补偿
	if(flag_att_force[Yr])
		deng_control_out+=deng_control_out*cosd(fabs(pit_err))*deng_param[3];//test	
	
	float deng_use=0;
	if(in->force_deng[1]<divid_time_check)
		deng_use=LIMIT(deng_control_out*deng_param[0]//弹簧收缩
									 ,0,0.68*vmc_all.delta_ht[0]/0.0001);
	else
		deng_use=deng_control_out;	 //弹簧伸长
	
  float temp_t=vmc_all.stance_time-divid_time_check;
  if(in->force_deng[1]<divid_time_check)//弹簧收缩
	{
	    in->force_deng[0]=sind(180*in->force_deng[1]/divid_time_check)*
											  LIMIT(deng_use,-2*vmc_all.kp_deng[id],2*vmc_all.kp_deng[id]); 	
	}else	if(in->force_deng[1]<divid_time_check+temp_t*deng_param[2])						//弹簧伸长
  {
			if(in->force_deng[2]==1)//正常
				 in->force_deng[0]=-sind(180*(in->force_deng[1]-divid_time_check)/temp_t)*
					 LIMIT(deng_use,-2*vmc_all.kp_deng[id],2*vmc_all.kp_deng[id]); 	
			if(in->force_deng[2]==2)//继续蹬腿 或回蹬腿
				 in->force_deng[0]=-sind(180*(in->force_deng[1]-divid_time_check)/temp_t)*
					 LIMIT(deng_use,-2*vmc_all.kp_deng[id],2*vmc_all.kp_deng[id]); 		
	}
	
	in->force_deng[1]+=dt*deng_param[4];//时间增量

	if(in->force_deng[1]>vmc_all.stance_time)//超出时间复位
		 in->force_deng[2]=in->force_deng[1]=0;
}
//-------------------------跨腿------------------------//----------------------------------------------
//-------------------------跨腿---------------------------------//----------------------------------
//-------------------------跨腿-------------------------------//-----------------------------------------
//-------------------------跨腿------------------------------//----------------------------------------------

//计算二次曲线系数
static float c0[4][3],c3[4][3],c4[4][3],c5[4][3],c6[4][3];
static void cal_curve_from_pos_new(VMC *in, END_POS t_pos, float desire_time)
{
char id=in->param.id;	
float pos_now[3],pos_tar[3];
pos_now[Xr]=in->epos.x;pos_now[Yr]=in->epos.y;pos_now[Zr]=in->epos.z;
pos_tar[Xr]=t_pos.x;pos_tar[Yr]=t_pos.y;pos_tar[Zr]=t_pos.z;
float t1=desire_time/2,t2=desire_time;
float p0[3], p1[3], p2[3];
//----------start
p0[0]=pos_now[Xr];p0[1]=pos_now[Yr];p0[2]=pos_now[Zr];
//--------------end
p2[0]=pos_tar[Xr];p2[1]=pos_tar[Yr];p2[2]=pos_tar[Zr];//z
//-------------middle
float k,b;
float end_weight;
float att_off=-sind(vmc_all.att_rate_trig[PITr]);
end_weight=end_tirg_rate;
if(vmc_all.param.en_att_tirg)
	end_weight=LIMIT(dead(ABS(vmc_all.att_rate_trig[PITr]),4.5),0,16.8)/16.8*0.5+0.5;

p1[0]=p0[0]*(1-end_weight)+p2[0]*end_weight;	//3
p1[1]=(p0[1]+p2[1])/2;
p1[0]=LIMIT(p1[0],(p0[0]+p2[0])/2-0.5*MAX_X,(p0[0]+p2[0])/2+0.5*MAX_X);
p1[2]=LIMIT((p0[2]+p2[2])/2+vmc_all.delta_ht[0],MAX_Z,MIN_Z);//wait

float p1_p0[3],p0_p2[3];
int i;
for(i=0;i<3;i++)
{c0[id][i]=p0[i];p1_p0[i]=p1[i]-p0[i];p0_p2[i]=p0[i]-p2[i];}
float t1_3=pow(t1,3),t1_4=pow(t1,4),t1_5=pow(t1,5),t1_6=pow(t1,6);
double t2_2=pow(t2,2),t2_3=pow(t2,3),t2_5=pow(t2,5),t2_6=pow(t2,6);
float temp1=0;temp1=1/(t1_3*pow((t1-t2),3)*t2_3);
for(i=0;i<3;i++)
{	
 c3[id][i]=-1*temp1*(t2_6*(p1_p0[i])+5*t1_4*t2_2*3*(p0_p2[i])+2*t1_6*5*(p0_p2[i])-3*t1_5*t2*8*(p0_p2[i]));
 c4[id][i]=temp1/t2*(3*t2_6*(p1_p0[i])+15*t1_3*t2_3*(p0_p2[i])-27*t1_5*t2*(p0_p2[i])+t1_6*15*(p0_p2[i]));
 c5[id][i]=-temp1/t2_2*3*(t2_6*(p1_p0[i])+2*t1_6*(p0_p2[i])+t1_3*t2_3*8*(p0_p2[i])-t1_4*t2_2*9*(p0_p2[i]));
 c6[id][i]=temp1/t2_2*(t2_5*(p1_p0[i])+6*t1_5*(p0_p2[i])+10*t1_3*t2_2*(p0_p2[i])-t1_4*t2*15*(p0_p2[i]));}
}

//读出规划曲线各采样三维位置
static END_POS cal_pos_tar_from_curve(VMC *in,float dt)
{
char id=in->param.id;	
END_POS epos;
float cal_curve[3];	
float time_now=LIMIT(in->param.time_trig,0,vmc_all.gait_time[1]);
char i;
for(i=0;i<3;i++)
	cal_curve[i]=c0[id][i]+c3[id][i]*pow(time_now,3)+c4[id][i]*pow(time_now,4)+c5[id][i]*pow(time_now,5)+c6[id][i]*pow(time_now,6);
epos.x=cal_curve[Xr];
epos.y=cal_curve[Yr];
epos.z=cal_curve[Zr];	
return epos;
}	

//跟踪跨腿轨迹  多项式
char test1=1;
static char trig_curve(VMC *in,float *x,float *z,float dt,char sel)
{ END_POS epos; 
  float alfa=0;
	float err[3],att_use[3];
  int id=in->param.id,temp=0;;
	att_use[PITr]=vmc_all.att_vm[PITr];//使用机械计算角度
	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	err[PITr]=LIMIT(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr]-att_use[PITr],-24,24);
	err[ROLr]=LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr],-24,24);
	err[2]=MAX(ABS(err[PITr]),ABS(err[ROLr]));
	switch(in->param.trig_state)
	{
		case 0:
			if(!sel){//摆线
				 alfa=2*PI*LIMIT(in->param.time_trig,0,vmc_all.gait_time[1]-vmc_all.stance_time)/(vmc_all.gait_time[1]-vmc_all.stance_time);
				if(in->param.time_trig<vmc_all.gait_time[1]-vmc_all.stance_time)
				{ 
				 *x=(in->tar_pos.x-in->st_pos.x)*(alfa-sin(alfa))/(2*PI)+in->st_pos.x;
				 *z=(in->param.delta_h+in->param.delta_h_att_off)*(1-cos(alfa))/2+in->st_pos.z+(in->tar_pos.z-in->st_pos.z)*in->param.time_trig/(vmc_all.gait_time[1]-vmc_all.stance_time);	
				}//else
				 //{*x=in->epos.x;*z=in->epos.z;}
		  }else{//多项式		
				if(in->param.time_trig<vmc_all.gait_time[1]-vmc_all.stance_time)
				{
				 epos=cal_pos_tar_from_curve(in,dt);
				 *x=epos.x;
				 *z=epos.z;
				}//else
				 //{*x=in->epos.x;*z=in->epos.z;}
		  }

			if(in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)&&in->epos.z>in->tar_epos.z)//&&ABS(in->epos.z-in->tar_epos.z)>0.005)
				in->param.trig_state=1;//到达步态周期还未满足设定高度  跳转到延伸状态
			else if(in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)*0.5
			  &&fabs(in->epos.z)>fabs(in->tar_epos.z)){//还未到达周期却已满足高度
				if(vmc_all.use_ground_sensor==0||vmc_all.use_ground_sensor==1)//未使用着地传感器默认结束
					{in->ground=1;
					 in->tar_epos.z=in->epos.z;
					 in->param.ground_state=1;//未传感器触地退出
					 in->param.ground_state_cnt=0;
					 in->param.trig_state=99;
					 return 1;}
					else
					 in->param.trig_state=1;//否则继续触地
			}else if(in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)){//超出跨腿周期
				 if(vmc_all.use_ground_sensor<=1){//不使用着地传感器直接退出
					 in->ground=1;
					 in->tar_epos.z=in->epos.z;
					 in->param.ground_state=1;//未传感器触地退出
					 in->param.ground_state_cnt=0;
					 in->param.trig_state=99;
					 return 1;}
				 else
					 in->param.trig_state=1;//否则继续触地
			 }
			
			in->param.time_trig+=dt;		
		break;
			 
		case 1://倾斜延迟跨腿+ 未着地继续触地
	  	if(fabs(in->epos.z)>fabs(in->tar_epos.z)&&vmc_all.use_ground_sensor<=1){//未使用着地传感器
					in->ground=1;
					in->param.trig_state=99;
					in->param.ground_state=1;//未传感器触地退出
				  in->param.ground_state_cnt=0;
					in->tar_epos.z=in->epos.z;//test
					return 1;
				}
			if(fabs(in->epos.z)>fabs(in->tar_epos.z)&&vmc_all.use_ground_sensor==2){//着地传感器			
					if(in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)*1.5||in->ground_s){
					in->ground=1;
					in->param.trig_state=99;
					in->param.ground_state=1;//未传感器触地退出
					in->param.ground_state_cnt=0;
					in->tar_epos.z=in->epos.z*(1-vmc_all.param.ground_dump);//主动柔性？	
					return 1;
					}
				}
			in->param.time_trig+=dt;
			//*x=epos.x;	
			//*z=epos.z-vmc_all.kp_touch*dt;
		  *z-=vmc_all.kp_touch*dt;
		break;
	}
	*x=LIMIT(*x,-MAX_X,MAX_X);
	*z=LIMIT(*z,MAX_Z,MIN_Z);
	//--------------------------着地判断-----------------------------
	if(vmc_all.use_ground_sensor>0
		&&in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)*vmc_all.param.trig_ground_st_rate//保证一定跨腿过程
	  &&in->ground_s){//&&err[2]<vmc_all.param.angle_set_trig){
		in->ground=1;
		in->param.trig_state=99;
		in->param.ground_state=11;//触地退出
		in->param.ground_state_cnt=0;
		in->tar_epos.z=in->epos.z*(1-vmc_all.param.ground_dump);// 主动柔性

		if(id==0||id==3)
			temp=1;
		else
			temp=0;
		if(test1){
					in->force_deng[1]=vmc[temp].force_deng[1];//复制角度
					in->force_deng[3]=vmc[temp].force_deng[0];//复制值
					in->force_deng[2]=2;//一起回缩 继续蹬腿
		}
		return 1;
	}	
	return 0;
}
//--------------------判断机器人状态------------------------//----------------------------------------------
//--------------------判断机器人状态-----------------------//----------------------------------
//--------------------判断机器人状态----------------------//-----------------------------------------
//--------------------判断机器人状态----------------------//----------------------------------------------
void state_check(float dt)
{
 static char state_hold,state_unmove;
 static float timer[5];
 static float ground_reg[2][4];
 float att_use[3],err[2];
 char i;	
 char ground_num[2]={0,0};
 float acc_norm=fast_sqrt(vmc_all.acc[Xr]*vmc_all.acc[Xr]+vmc_all.acc[Yr]*vmc_all.acc[Yr]+vmc_all.acc[Zr]*vmc_all.acc[Zr]);
 vmc_all.acc[3]=acc_norm;	

 att_use[ROLr]=vmc_all.att_ctrl[ROLr];
 err[ROLr]=vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr];
 
 for(i=0;i<4;i++){
   ground_num[0]+=vmc[i].ground;
	 ground_num[1]+=vmc[i].ground_s;
 } 
 
 if((ABS(vmc_all.tar_spd.x)>MAX_SPD*0.005||ABS(vmc_all.tar_spd.z)>MAX_SPD_RAD*0.005)||
   (vmc_all.param.smart_control_mode[2]==MODE_ATT_YAW_ONLY||vmc_all.param.smart_control_mode[2]==MODE_ATT))	 
		vmc_all.param.have_cmd=1;
 else
	  vmc_all.param.have_cmd=0;
 
 
	switch(state_hold)
	 {
	  case 0:	  
			 if(ground_num[1]>=1||vmc_all.param.have_cmd)
			  {timer[0]=0;vmc_all.hand_hold=0;}
			 else
				 timer[0]+=dt;
			 
			 if(timer[0]>1&&
				 (ABS(vmc_all.att_rate_trig[PITr])>MAX_SPD_RAD*0.12||
					ABS(vmc_all.att_rate_trig[ROLr])>MAX_SPD_RAD*0.12||
					ABS(vmc_all.att_rate_trig[YAWr])>MAX_SPD_RAD*0.12||
			    vmc_all.acc[Zr]>0.2*9.81))
			 {state_hold=1;timer[0]=0;vmc_all.hand_hold=1;}
		break;
		case 1:
			if(vmc_all.param.have_cmd)
				 timer[1]+=dt;
			else
				 timer[1]=0;
			
			if( timer[1]>2||ground_num[1]>=2){
		    state_hold=0; timer[1]=0;vmc_all.hand_hold=0;
			}
		break;
	 }	
	 if(vmc_all.use_ground_sensor==0)
		 state_hold=0;
//---------------------静止判断-------------------------
	switch(state_unmove)
	 {
	  case 0:	  
			vmc_all.unmove=0;
			if(vmc_all.param.have_cmd)
				timer[3]=0;
			else
				timer[3]+=dt;
			
			if(timer[3]>0.25){
				if(vmc_all.use_ground_sensor>0){//使用着地传感器
				state_unmove=1;
				ground_reg[0][0]=ground_reg[0][1]=ground_reg[0][2]=ground_reg[10][3]=0;
				ground_reg[1][0]=ground_reg[1][1]=ground_reg[1][2]=ground_reg[1][3]=0;
			}else 
			  {state_unmove=11;vmc_all.unmove=1;}
			}
		break;
		case 1:
			for(i=0;i<4;i++)
		  {
				if(vmc[i].ground_s)
				{ground_reg[0][i]=0;ground_reg[1][i]=1;}
				else
				 ground_reg[0][i]+=dt;
				
				if(ground_reg[0][i]>10)
					{ground_reg[0][i]=ground_reg[1][i]=0;}
			}
		
			if((ground_reg[1][0]+ground_reg[1][1]+ground_reg[1][2]+ground_reg[1][3]>=3))
			  {state_unmove=2;vmc_all.unmove=1;}
			
			if(vmc_all.param.have_cmd)
				state_unmove=0;
		break;
		case 2:
			if(vmc_all.param.have_cmd||ABS(err[ROLr])>16.8||ABS(vmc_all.att_rate_ctrl[ROLr])>1.2*MAX_SPD_RAD)
				state_unmove=0;
		  vmc_all.unmove=1;
		break;
			
		case 11:	
			if(vmc_all.param.have_cmd)
				timer[3]=state_unmove=0;
		break;
	 }	
	 
	 if(vmc_all.hand_hold)
    vmc_all.unmove=state_unmove=0;
	 
	 vmc_all.hand_hold=0;
	 static char hand_reg=0;
	 if(hand_reg==1&&vmc_all.hand_hold==0&&vmc_all.unmove==1)
	   vmc_all.unmove=state_unmove=0;
	 hand_reg=vmc_all.hand_hold;
	 if(!vmc_all.param.en_hold_on||vmc_all.use_ground_sensor==0)
		 vmc_all.hand_hold=0;
}
#endif
//机器人启动和保护
float att_trig_reset_dead=2.56;
float soft_param[2]={0.75,0.4};//启动速度  启动比例
char power_task(float dt)
{
	char i;
	static float t,t_rst[3],timer[5];	
	static char state,rst_state,soft_start_state;
	float cog_off,att_off;
	float end_dis[4];
	float att_use[3],err[2];

	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	err[ROLr]=vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr];
	
	switch(vmc_all.power_state)
	{
		case 0:
			if(ABS(vmc_all.param.tar_spd_use_rc.x)>MIN_SPD_ST||vmc_all.sita_test[4]) 
				 t+=dt;
			else
				 t=0;
			
			if(t>1)
			{vmc_all.leg_power=vmc_all.power_state=1;t=0;}
    break;		
	  case 1://启动时缩腿保护
			 vmc_all.gait_on=0;
			 for(i=0;i<4;i++){
				vmc[i].sita1=0-30;
				vmc[i].sita2=180-(-30);
				estimate_end_state(&vmc[i],dt); 
				vmc[i].ground=1; 
	     }
			 t+=dt;
			 if(t>2)
			 {
				t=0;vmc_all.power_state=2;
				vmc[FL1].e_pos_base[0].z=vmc[FL1].e_pos_base[1].z=vmc_all.tar_pos.z;
				vmc[BL1].e_pos_base[0].z=vmc[BL1].e_pos_base[1].z=vmc_all.tar_pos.z;
				vmc[FL2].e_pos_base[0].z=vmc[FL2].e_pos_base[1].z=vmc_all.tar_pos.z;
				vmc[BL2].e_pos_base[0].z=vmc[BL2].e_pos_base[1].z=vmc_all.tar_pos.z;
			 }
		break;	
		case 2://站立
			vmc_all.param.tar_spd_use_rc.x=vmc_all.param.tar_spd_use_rc.z=0;
		  vmc_all.tar_spd.x=vmc_all.tar_spd.y=vmc_all.tar_spd.z=0;
		  vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
		  vmc_all.unmove=1;
		  for(i=0;i<4;i++){
			 vmc_all.tar_pos.z=-(sind(35)*vmc_all.l1+sind(50)*vmc_all.l2);
		   vmc[i].tar_pos.z=vmc_all.tar_pos.z;
			 vmc[i].tar_epos.z=vmc_all.tar_pos.z;
			}
		   t+=dt;
			 if(t>2)
			 {t=0;vmc_all.power_state=3;}
		break;
    case 3://正常工作
				cog_off=vmc_all.param.cog_off_use[0];//重心补偿
				att_off=vmc_all.param.cog_off_use[1];//姿态地形跟随

		 //------------------------自动复位跨腿--------------------------
			for(i=0;i<4;i++)
			  vmc_all.end_dis[i]=end_dis[i]=ABS(vmc[i].epos.x-(cog_off+att_off));//每个腿里中心偏差

			switch(rst_state)
			{	
				case 0:
					if(vmc_all.param.have_cmd)
						 rst_state=1;
					
					if(((end_dis[0]+end_dis[3])>2*vmc_all.rst_dead //腿超出自复位
						||(end_dis[1]+end_dis[2])>2*vmc_all.rst_dead)
					  &&vmc_all.hand_hold==0&&vmc_all.unmove==0&&fabs(err[ROLr])<att_trig_reset_dead)
					 {rst_state=2;vmc_all.delta_ht[0]=0.6*vmc_all.delta_ht[1];}
				break;
				case 1://一段时间无遥控
			   if(vmc_all.param.have_cmd==0)
					t_rst[0]+=dt;
				 else
				  t_rst[0]=0;
				 
				  vmc_all.delta_ht[0]+=(vmc_all.delta_ht[1]-vmc_all.delta_ht[0])*soft_param[0]*dt;

					if(t_rst[0]>T_RST&&fabs(err[ROLr])<att_trig_reset_dead)
					{rst_state=t_rst[0]=0;vmc_all.delta_ht[0]=soft_param[1]*vmc_all.delta_ht[1];}
				break;
				case 2://主动复位
						vmc_all.gait_on=1;
				   if(vmc_all.param.have_cmd==1)
					 {rst_state=t_rst[0]=0;}
					 else
						t_rst[0]+=dt; 
				    
					 if(t_rst[0]>vmc_all.stance_time*2)
						{rst_state=t_rst[0]=0;}
			  break;	
		  }
    break;
	}
	if(vmc_all.param.soft_start==0)
	 vmc_all.delta_ht[0]= vmc_all.delta_ht[1];
	if(vmc_all.power_state<=2)
	 vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
	return vmc_all.leg_power;
}	

#if !USE_DEMO
//跨腿延伸
float k_ex1=1;
char trig_extend1(VMC *in,float dt)
{
	char id=in->param.id;
  switch(in->param.ground_state)
	{
		case 0://空闲
			if(vmc_all.unmove)
		     in->param.ground_state_cnt+=dt;
			if(in->param.ground_state_cnt>vmc_all.param.ground_rst)
				{in->param.ground_state_cnt=0;in->param.ground_state=1;}
		break;
	  case 1://正常结束
			if(extend_pid_all.ki==0)
				extend_pid[id].i=0;
			else
				extend_pid[id].i-=vmc_all.param.max_l*extend_pid_all.ki*dt;
		  in->param.ground_state_cnt+=dt;
		  if(in->ground_s||in->param.ground_state_cnt>(vmc_all.gait_time[1]-vmc_all.stance_time)*15){
				in->param.ground_state=99;
		  }
		break;
		case 11://直接触地
			if(vmc_all.unmove)
		     in->param.ground_state_cnt+=dt;
			if(in->param.ground_state_cnt>vmc_all.param.ground_rst)
				{in->param.ground_state_cnt=0;in->param.ground_state=1;}
		break;	
		case 99://下压完毕
			if(vmc_all.unmove)
		     in->param.ground_state_cnt+=dt;
			if(in->param.ground_state_cnt>vmc_all.param.ground_rst)
				{in->param.ground_state_cnt=0;in->param.ground_state=1;}
		break;
	}
	
	if(in->ground_s&&in->ground)//&&in->param.ground_state==99)
			extend_pid[id].i+=vmc_all.param.max_l*extend_pid_all.ki*dt*k_ex1;
	extend_pid[id].i=LIMIT(extend_pid[id].i,-ABS(in->epos.z-MAX_Z),0);
}

char trig_extend(VMC *in,float dt)
{ 
	char id=in->param.id,i;
	float ground_off_use;
	static float cnt_rst[4];
	static char gait_flag=1,state_extend,good_cnt[4];
	static float ground_reg[2][4],ground_cnt[4],extend_gain[4],check_time[4];
	if(in->param.ground_state==0)
		cnt_rst[id]+=dt;
	else
		cnt_rst[id]=0;
	
	if(fabs(vmc_all.param.tar_spd_use_rc.x)>0.01*MAX_SPD||fabs(vmc_all.tar_spd.z)>0.01*MAX_SPD_RAD)//移动FLAG
	{cnt_rst[id]=0;gait_flag=1;}
	if(cnt_rst[id]>vmc_all.param.ground_rst)//静止FLAG
	{cnt_rst[id]=0;in->param.ground_state=1;gait_flag=0;}
	
	if(gait_flag){
		//复位
		ground_reg[0][id]=0;ground_reg[1][id]=0;
		extend_gain[id]=1;ground_cnt[id]=0;good_cnt[id]=0;
		check_time[id]=vmc_all.param.ground_rst;
		
		ground_off_use=vmc_all.ground_off[0];//移动着地
	}
	else
		ground_off_use=vmc_all.ground_off[1];//静止着地
	
	if(gait_flag==0){//静止
		    switch(state_extend)
				{
				  case 0:
						ground_cnt[id]+=dt;
						if(ground_cnt[id]>check_time[id]&&good_cnt[id]<3*2)
							{
							  ground_reg[1][id]=1;//触发伸腿
								state_extend=1;
								ground_cnt[id]=0;
							}	
					break;
					case 1:
						ground_cnt[id]+=dt;
						if(ground_cnt[id]>(vmc_all.gait_time[1]-vmc_all.stance_time)*4)//失败
						{ 
							ground_reg[1][id]=0;
							state_extend=2;
							ground_cnt[id]=0;
						}
						if(in->ground_s){//成功
						  ground_reg[1][id]=0;
							state_extend=3;
							ground_cnt[id]=0;
						}
					break;
				  case 2://触地失败
						check_time[id]-=vmc_all.param.ground_rst;	
						extend_gain[id]+=0.2;	
					  good_cnt[id]--;
					  state_extend=0;
					break;
				  case 3://触地成功
						check_time[id]+=vmc_all.param.ground_rst*2;	
					  good_cnt[id]+=2;
					  state_extend=0;
					break;
				}	
				//倾倒复位
				if((ABS(vmc_all.att_rate_trig[PITr])>MAX_SPD_RAD*0.35||
					ABS(vmc_all.att_rate_trig[ROLr])>MAX_SPD_RAD*0.35||
					ABS(vmc_all.att_rate_trig[YAWr])>MAX_SPD_RAD*0.35||
			    vmc_all.acc[3]>0.2*9.81)){
					ground_reg[0][id]=0;ground_reg[1][id]=0;
					extend_gain[id]=1;ground_cnt[id]=0;good_cnt[id]=0;
					check_time[id]=vmc_all.param.ground_rst;
					}
						
				good_cnt[id]=LIMIT(good_cnt[id],0,127);
        extend_gain[id]=LIMIT(extend_gain[id],0.5,3.5);					
				check_time[id]=LIMIT(check_time[id],vmc_all.param.ground_rst,10);	
	}
	
	//触地操作
	switch(in->param.ground_state){
		case 1:
			if(in->ground&&!in->ground_s&&gait_flag==1)
			{in->param.ground_state=2;in->param.time_trig=0;}
			if(in->ground&&ground_reg[1][id]==1&&gait_flag==0)//静止
			{in->param.ground_state=2;in->param.time_trig=0;}
		break;
	  case 2:
			  if(gait_flag)
					in->tar_epos.z-=ground_off_use*dt*vmc_all.param.dt_size[0];
				else//静止
					in->tar_epos.z-=ground_off_use*dt*vmc_all.param.dt_size[0]*extend_gain[id];
			  in->param.time_trig+=dt;
		   if(in->ground_s||in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)*4)//着地或者超时强制停止腿延伸
			 {
				ground_reg[1][id]=0;
			  in->param.ground_state=0;
			  gait_flag=1;
			 }
		break;
  }
		
	in->tar_epos.z=LIMIT(in->tar_epos.z,MAX_Z*0.95,MIN_Z*1.05);
}



//--------------------------跳跃-----------------------//----------------------------------------------
//--------------------------跳跃--------------------------------//----------------------------------
//--------------------------跳跃-----------------------------//-----------------------------------------
//--------------------------跳跃----------------------------//--------------------------------------------
float k_gain=1;
void jump_4leg(float dt)
{
 static float cnt[3];
 char i;
 float temp[3];
 float E[3];
  switch(vmc_all.param.jump_state)
	{
		//float vmc_all.param.jump_tar_param[2];//高度 距离
		case 1://计算相关参数
			
		  temp[0]=-99;
		  for(i=0;i<4;i++)
		    if(vmc[i].epos.z>temp[0])
					temp[0]=vmc[i].epos.z;
	    vmc_all.param.jump_param_use[0]=temp[0]*1.25;//收缩腿时机体高度
		  //
		  temp[1]=(vmc_all.param.max_l-(fabs(vmc_all.param.jump_param_use[0])-fabs(MIN_Z)))*0.86;//当前可用腿长
			
			vmc_all.param.jump_param_use[1]=sqrt(vmc_all.param.jump_tar_param[0]*2/9.8)*2;//时间消耗
				
			vmc_all.param.jump_param_use[2]=vmc_all.param.jump_tar_param[1]/LIMIT(vmc_all.param.jump_param_use[1],0.0001,5);//水平速度	
			
			E[0]=vmc_all.mess*9.8*vmc_all.param.jump_tar_param[0];//重力势能
			E[1]=0.5*vmc_all.mess*pow(vmc_all.param.jump_param_use[2],2);//前进能量	
			E[2]=(E[0]+E[1]);//总能量	
			
      vmc_all.param.jump_param_use[3]=1.25*RAD_TO_DEG*sign(vmc_all.param.jump_tar_param[1])
																	   *ABS(fast_atan2(0.5*vmc_all.param.jump_tar_param[1]
																			,vmc_all.param.jump_tar_param[0]));//伸腿角度				
		  vmc_all.param.jump_param_use[4]=ABS(LIMIT(fast_sqrt(2*E[2]),0,temp[1]));//伸腿长度
	  	
		  vmc_all.param.jump_out[Xr]=-vmc_all.param.jump_param_use[4]*sind(vmc_all.param.jump_param_use[3])*k_gain;
			vmc_all.param.jump_out[Zr]=vmc_all.param.jump_param_use[4]*cosd(vmc_all.param.jump_param_use[3])*k_gain;	
			vmc_all.param.jump_out[Yr]=LIMIT(vmc_all.param.jump_out[Xr],MIN_X,MAX_X);
		  cnt[0]=cnt[1]=cnt[2]=0;
			if(vmc_all.param.jump_tar_param[0]>0)
				vmc_all.param.jump_state++;
			else
				vmc_all.param.jump_state=0;
	  break;
		
		case 2://收缩
	    vmc_all.tar_pos.z=LIMIT(MIN_Z-(vmc_all.param.jump_param_use[0]-ABS(MIN_Z))*0.35,MAX_Z,MIN_Z);
		  cnt[0]+=dt;
		  if(cnt[0]>4)
				{cnt[0]=0;vmc_all.param.jump_state++;}
	  break;
	
		case 3://伸长
			vmc_all.fly=1;
	    for(i=0;i<4;i++)
		   {
			   vmc[i].tar_epos_force.x=LIMIT(vmc[i].epos.x+vmc_all.param.jump_out[Xr],MIN_X,MAX_X);
				 vmc[i].tar_epos_force.z=LIMIT(vmc[i].epos.z-vmc_all.param.jump_out[Zr],MAX_Z,MIN_Z);
			 }
		  cnt[0]+=dt;
		  if(cnt[0]>vmc_all.stance_time/2)
				{cnt[0]=0;vmc_all.param.jump_state++;}
	  break;
		
		case 4://空中
	     for(i=0;i<4;i++){
				vmc[i].tar_epos_force.x=vmc[i].tar_epos_force.z=0; 
				vmc[i].sita1=0-30;
				vmc[i].sita2=180-(-30);
				estimate_end_state(&vmc[i],dt);
			  cal_jacobi(&vmc[i]);	 
				vmc[i].ground=1; 
	     }		 
		  cnt[0]+=dt;
		  if(cnt[0]>vmc_all.stance_time/2)
				{cnt[0]=0;vmc_all.param.jump_state++;
				 vmc_all.power_state=0;
				 vmc_all.leg_power=0;	
				 vmc_all.param.jump_out[Yr]=0;
				}
	  break;
		
		case 5://空中缩腿触地检测
		   vmc_all.tar_pos.z=LIMIT(MIN_Z-(vmc_all.param.jump_param_use[0]-ABS(MIN_Z))*0.35,MAX_Z,MIN_Z);
		   cnt[0]+=dt;
		   if(cnt[0]>vmc_all.param.jump_param_use[1])//总飞行时间判断
				{cnt[0]=0;vmc_all.param.jump_state++;
				 vmc_all.leg_power=vmc_all.power_state=1;
				}
	     if(vmc_all.acc_norm>9.8)//重力加速度
				{cnt[0]=0;vmc_all.param.jump_state++;
				 vmc_all.leg_power=vmc_all.power_state=1;
				}
	  break;
		
		case 6://重新站立
	    vmc_all.tar_pos.z=MIN_Z-vmc_all.param.max_l*0.5;
			cnt[0]+=dt;
		   if(cnt[0]>3)
				{ cnt[0]=0;
					vmc_all.fly=0;
					vmc_all.param.jump_state=0;}
	  break;
	}
}	

//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//
//------------------------------------状态机-------------------------------------//
//-------------------------------------------------------------------------------//
void onboard_control(float dt)
{
	static char state;
	static float cnt;
	vmc_all.param.have_cmd_sdk[0][0]=vmc_all.param.have_cmd_sdk[0][1]=vmc_all.param.have_cmd_sdk[0][2]=vmc_all.param.have_cmd_sdk[0][3]=0;
	vmc_all.param.have_cmd_sdk[1][0]=vmc_all.param.have_cmd_sdk[1][1]=vmc_all.param.have_cmd_sdk[1][2]=vmc_all.param.have_cmd_sdk[1][3]=0;
	switch(state)
	{
	  case 0:
		if(vmc_all.param.have_cmd_rc[0]+vmc_all.param.have_cmd_rc[1]==0){//无遥控下
			if(vmc_all.param.smart_control_mode[0]==MODE_SPD||
				 vmc_all.param.smart_control_mode[0]==MODE_POS){//移动速度
				if(vmc_all.param.tar_spd_on.x!=0){
						vmc_all.tar_spd.x=LIMIT(vmc_all.param.tar_spd_on.x,-MAX_SPD,MAX_SPD);
				    vmc_all.param.have_cmd_sdk[0][Xr]=1;
				}
				if(vmc_all.param.tar_spd_on.z!=0){
						vmc_all.tar_spd.z=LIMIT(vmc_all.param.tar_spd_on.z,-MAX_SPD_RAD,MAX_SPD_RAD);
					  vmc_all.param.have_cmd_sdk[1][YAWrr]=1;
				}
			}
				
			if(vmc_all.param.smart_control_mode[1]==MODE_POS){//高度
			 vmc_all.tar_pos.z=LIMIT(vmc_all.param.tar_pos_on.z,MAX_Z,MIN_Z);
			 vmc_all.param.have_cmd_sdk[0][Zr]=1;
			}

			if(vmc_all.param.smart_control_mode[2]==MODE_ATT_PR_ONLY||vmc_all.param.smart_control_mode[2]==MODE_ATT){//姿态角
			 vmc_all.tar_att[0]=vmc_all.param.tar_att_on[0];
			 vmc_all.tar_att[1]=vmc_all.param.tar_att_on[1];
			 vmc_all.param.have_cmd_sdk[1][PITr]=1;	
			 vmc_all.param.have_cmd_sdk[1][ROLr]=1;		
			}
			
			if(vmc_all.param.smart_control_mode[2]==MODE_ATT_YAW_ONLY||vmc_all.param.smart_control_mode[2]==MODE_ATT){//航向角
			 vmc_all.tar_att[YAWr]=vmc_all.param.tar_att_on[YAWr];
			 vmc_all.param.have_cmd_sdk[1][YAWr]=1;
			}				

	  }else if(vmc_all.param.control_mode_all==2)//SDK模式时的外部遥控
		{
			state=1;cnt=0;
		}
		break;
		case 1://RESET
			vmc_all.tar_att[PITr]=vmc_all.tar_att[ROLr]=0;
		  vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
		  vmc_all.tar_spd.x=vmc_all.tar_spd.z=0;
			vmc_all.tar_pos.x=vmc_all.pos_n.x;
			vmc_all.tar_pos.y=vmc_all.pos_n.y;
		  vmc_all.tar_pos.z=vmc_all.pos.z;
		  state=2;
		break;
	  case 2:		
			if(vmc_all.param.have_cmd_rc[0]+vmc_all.param.have_cmd_rc[1]>0)//有遥控
		   cnt=0;
		  else
			 cnt+=dt;
			
			if(cnt>3)
			{state=cnt=0;}
		break;
	}		  
}
//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//
//基础步态测试  USE
float test_epos[3]={0,0,-0.07};
void  VMC_OLDX_VER1(float dt)
{
 static char init=0,state=0,switch_flag=0,cnt_time_change=0;
 static uint16_t time,time1;
 float out[2],kp_deng_use;	
 char i;
 static float timer_task[10],time_delay;
 static char front_leg,back_leg;
	if(!init)
	{
	  init=1;
		check_lisence();
	}
	
	if(ukey_state==96||block_lisence){
  
	state_rst(dt);

		
	//速度分配
	
	if(timer_task[1]> 0){
	pos_control(timer_task[1]);
	timer_task[1]=0;
 }
	
  onboard_control(dt);
 
 if(timer_task[2]>0.025){
	spd_control(timer_task[2]);
	timer_task[2]=0;
 }

 
 if(vmc_all.power_state>=2&&timer_task[3]>0.025){
	yaw_control(timer_task[3]);
	timer_task[3]=0;
 }

	if(timer_task[0]>0.1){
		state_check(timer_task[0]);
		timer_task[0]=0;
	}
	
	if(gait_test[0]>0){
		gait_test[1]+=dt*(gait_test[2]);if(gait_test[1]>360)gait_test[1]=0;
		gait_test[4]=sind(gait_test[1])*vmc_all.param.max_l*gait_test[3];

	 if(gait_test[0]==1)
		vmc_all.tar_pos.z=LIMIT(MIN_Z-vmc_all.param.max_l*0.6+gait_test[4],MAX_Z,MIN_Z);
   else if(gait_test[0]==2)
	  vmc_all.tar_att_off[PITr]=LIMIT(sind(gait_test[1])*11,-22,22);
	 else if(gait_test[0]==3)
		 for(i=0;i<4;i++)
				vmc[i].tar_epos.z=LIMIT(MIN_Z-vmc_all.param.max_l*0.6+gait_test[4],MAX_Z,MIN_Z);
	 else if(gait_test[0]==4)
		 vmc_all.tar_att_off[PITr]=11;
	 }
	
	
	 //估计足末状态
	for(i=0;i<4;i++)
		estimate_end_state(&vmc[i],dt);
	 
	switch(state)
	{
		case 0:
		if(vmc_all.unmove==0&&vmc_all.hand_hold==0&&vmc_all.fly==0&&
			 (ABS(vmc_all.tar_spd.x)>MIN_SPD_ST			
			||ABS(vmc_all.tar_spd.z)>MIN_SPD_ST
			||ABS(vmc_all.tar_spd.y)>MIN_SPD_ST
			||vmc_all.gait_on)){
	
			if(cnt_time_change++>2){
				vmc_all.gait_time[1]=vmc_all.gait_time[3];	
				vmc_all.stance_time=vmc_all.stance_time_auto;		
				vmc_all.delay_time[2]=vmc_all.delay_time[1];			
				cnt_time_change=0;
			}
			//规划落足点
			if(switch_flag)	
			{
				cal_tar_end_pos(&vmc[FL1]);cal_tar_end_pos(&vmc[BL2]);
				vmc[FL1].st_pos.x=vmc[FL1].epos.x;
				vmc[FL1].st_pos.z=vmc[FL1].epos.z;
				vmc[FL1].tar_pos.x=vmc[FL1].tar_epos.x;
				vmc[FL1].tar_pos.z=vmc[FL1].tar_epos.z;
				vmc[FL1].param.time_trig=0;
				vmc[BL2].st_pos.x=vmc[BL2].epos.x;
				vmc[BL2].st_pos.z=vmc[BL2].epos.z;
				vmc[BL2].tar_pos.x=vmc[BL2].tar_epos.x;
				vmc[BL2].tar_pos.z=vmc[BL2].tar_epos.z;
				vmc[BL2].param.time_trig=0;
				vmc[BL2].param.trig_state=vmc[FL1].param.trig_state=0;
				vmc[BL2].param.ground_state=vmc[FL1].param.ground_state=0;
				vmc[FL2].force_deng[1]=vmc[BL1].force_deng[1]=0;
				vmc[FL2].force_deng[2]=vmc[BL1].force_deng[2]=1;
				vmc_all.kp_deng_ctrl[1][BL1]=vmc_all.kp_deng_ctrl[0][BL1];
				vmc_all.kp_deng_ctrl[1][FL2]=vmc_all.kp_deng_ctrl[0][FL2];
				vmc[FL1].param.delta_h_att_off=vmc_all.delta_h_att_off;
				vmc[BL2].param.delta_h_att_off=vmc_all.delta_h_att_off;
				
				cal_curve_from_pos_new(&vmc[FL1],vmc[FL1].tar_pos, vmc_all.gait_time[1]-vmc_all.stance_time);
			  cal_curve_from_pos_new(&vmc[BL2],vmc[BL2].tar_pos, vmc_all.gait_time[1]-vmc_all.stance_time);
			}
			else
			{
				cal_tar_end_pos(&vmc[FL2]);cal_tar_end_pos(&vmc[BL1]);
				vmc[FL2].st_pos.x=vmc[FL2].epos.x;
				vmc[FL2].st_pos.z=vmc[FL2].epos.z;
				vmc[FL2].tar_pos.x=vmc[FL2].tar_epos.x;
				vmc[FL2].tar_pos.z=vmc[FL2].tar_epos.z;
				vmc[FL2].param.time_trig=0;
				vmc[BL1].st_pos.x=vmc[BL1].epos.x;
				vmc[BL1].st_pos.z=vmc[BL1].epos.z;
				vmc[BL1].tar_pos.x=vmc[BL1].tar_epos.x;
				vmc[BL1].tar_pos.z=vmc[BL1].tar_epos.z;
				vmc[BL1].param.time_trig=0;
				vmc[BL1].param.trig_state=vmc[FL2].param.trig_state=0;
				vmc[BL1].param.ground_state=vmc[FL2].param.ground_state=0;
				vmc[FL1].force_deng[1]=vmc[BL2].force_deng[1]=0;
				vmc[FL1].force_deng[2]=vmc[BL2].force_deng[2]=1;
				vmc_all.kp_deng_ctrl[1][BL2]=vmc_all.kp_deng_ctrl[0][BL2];
				vmc_all.kp_deng_ctrl[1][FL1]=vmc_all.kp_deng_ctrl[0][FL1];
				vmc[FL2].param.delta_h_att_off=vmc_all.delta_h_att_off;
				vmc[BL1].param.delta_h_att_off=vmc_all.delta_h_att_off;
				  
				cal_curve_from_pos_new(&vmc[FL2],vmc[FL2].tar_pos, vmc_all.gait_time[1]-vmc_all.stance_time);
			  cal_curve_from_pos_new(&vmc[BL1],vmc[BL1].tar_pos, vmc_all.gait_time[1]-vmc_all.stance_time);
			}	
			state++;
	  }
		break;
		case 1:
			//跨腿
			if(switch_flag)	
			{
				trig_curve(&vmc[FL1],&vmc[FL1].param.tar_epos.x,&vmc[FL1].param.tar_epos.z,dt,vmc_all.trig_mode);
        vmc[FL1].param.tar_epos.z=LIMIT(vmc[FL1].param.tar_epos.z,MAX_Z,MIN_Z);vmc[FL1].param.tar_epos.x=LIMIT(vmc[FL1].param.tar_epos.x,MIN_X,MAX_X);
				trig_curve(&vmc[BL2],&vmc[BL2].param.tar_epos.x,&vmc[BL2].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[BL2].param.tar_epos.z=LIMIT(vmc[BL2].param.tar_epos.z,MAX_Z,MIN_Z);vmc[BL2].param.tar_epos.x=LIMIT(vmc[BL2].param.tar_epos.x,MIN_X,MAX_X);
				inv_end_state(vmc[FL1].param.tar_epos.x,vmc[FL1].param.tar_epos.z,&vmc[FL1].sita1,&vmc[FL1].sita2);
				inv_end_state(vmc[BL2].param.tar_epos.x,vmc[BL2].param.tar_epos.z,&vmc[BL2].sita1,&vmc[BL2].sita2);
				if(vmc[FL1].ground&&vmc[FL2].ground&&vmc[BL1].ground&&vmc[BL2].ground)
				{	if(vmc_all.delay_time[2]>0) 
					state++;
					else
					{time_delay=state=vmc_all.gait_on=0;switch_flag=!switch_flag;}	
					vmc[FL1].param.trig_state=vmc[BL2].param.trig_state=0;
				  //vmc[FL1].param.ground_state=vmc[BL2].param.ground_state=1;
				  vmc[FL1].force_deng[1]=vmc[BL2].force_deng[1]=0;
				  vmc[FL1].force_deng[2]=vmc[BL2].force_deng[2]=0;
				}
			}
			else
			{
				trig_curve(&vmc[FL2],&vmc[FL2].param.tar_epos.x,&vmc[FL2].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[FL1].param.tar_epos.z=LIMIT(vmc[FL1].param.tar_epos.z,MAX_Z,MIN_Z);vmc[FL1].param.tar_epos.x=LIMIT(vmc[FL1].param.tar_epos.x,MIN_X,MAX_X);
				trig_curve(&vmc[BL1],&vmc[BL1].param.tar_epos.x,&vmc[BL1].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[BL1].param.tar_epos.z=LIMIT(vmc[BL1].param.tar_epos.z,MAX_Z,MIN_Z);vmc[BL1].param.tar_epos.x=LIMIT(vmc[BL1].param.tar_epos.x,MIN_X,MAX_X);
				inv_end_state(vmc[FL2].param.tar_epos.x,vmc[FL2].param.tar_epos.z,&vmc[FL2].sita1,&vmc[FL2].sita2);
				inv_end_state(vmc[BL1].param.tar_epos.x,vmc[BL1].param.tar_epos.z,&vmc[BL1].sita1,&vmc[BL1].sita2);
				if(vmc[FL1].ground&&vmc[FL2].ground&&vmc[BL1].ground&&vmc[BL2].ground)
				 {
					if(vmc_all.delay_time[2]>0) 
					state++;
					else
					{time_delay=state=vmc_all.gait_on=0;switch_flag=!switch_flag;}	
					vmc[FL2].param.trig_state=vmc[BL1].param.trig_state=0;
					//vmc[FL2].param.ground_state=vmc[BL1].param.ground_state=1;
					vmc[FL2].force_deng[1]=vmc[BL1].force_deng[1]=0;
					vmc[FL2].force_deng[2]=vmc[BL1].force_deng[2]=0; 
				 }
			}	
		break;
		case 2:
			//四足着地延时
		  time_delay+=dt;
      if(time_delay>vmc_all.delay_time[2])
				{time_delay=state=vmc_all.gait_on=0;switch_flag=!switch_flag;}	
		break;
	}
	//--------------------------控制器------------------------------
 
	 //跳跃
	 jump_4leg(dt);
	 
	 float pit_err=(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr])-vmc_all.att_ctrl[3];
	 #if GLOBAL_CONTROL
		 if(gait_test[0]!=3&&vmc_all.power_state>=2){
			 height_control_all(dt);
	  
		 if(vmc_all.use_att==1)
			 attitude_control_all(dt);
	   }
	 #endif
	 //---------------蹬腿和足尖速度控制-------------------
   for(i=0;i<4;i++){
	     base_reset(&vmc[i],dt);//复位基准
		 
		 	 if(vmc_all.power_state>=2){//着地时姿态控制		
				 
			 if(vmc_all.tar_spd.x>0)
			 {deng_sel=0;T_GAIN=2;}
			 else
			 {deng_sel=1;T_GAIN=1;}	 
			 
			 #if CHECK_USE_SENSOR
       if(vmc[i].ground){//
			 #endif

				 if(gait_test[0]!=3){
					 
				 #if !GLOBAL_CONTROL	 
				 height_control1(&vmc[i],dt);	
				 if(vmc_all.use_att==1)
					 attitude_control1(&vmc[i],dt);
				 
				 //腿部未触地延伸
			   if(vmc_all.use_ground_sensor&&vmc_all.hand_hold==0&&vmc_all.power_state>=2)
				   trig_extend1(&vmc[i],dt);
				 #endif
				//跨腿下蹬 
				#if SLIP_MODE
			  if(vmc[i].force_deng[2]>0&&
					 vmc[i].ground&&
				   vmc_all.power_state>=2)
				{
					if(((deng_sel==1&&1)||0)&&0){//COS
					if(vmc[i].ground){
							deng_control(&vmc[i],dt);
							vmc_all.kp_deng_ctrl[1][i]=vmc_all.kp_deng_ctrl[0][i];
							float kp_deng_all=vmc_all.kp_deng[i]//Basic 
																+vmc_all.kp_deng_ctrl[1][i]//ROL err
																;//COG err
							float kp_deng_use=1;
							
							if(vmc[i].force_deng[2]>0&&vmc_all.power_state>=2){
							 if(vmc[i].force_deng[2]==1)
									 vmc[i].force_deng[0]=-cosd(vmc[i].force_deng[1])*kp_deng_use*
										 LIMIT(kp_deng_all,-2*vmc_all.kp_deng[i],2*vmc_all.kp_deng[i])
										 *0.4/(vmc_all.gait_time[1]+0.0000001); 
							 else if(vmc[i].force_deng[2]==2)
									 vmc[i].force_deng[0]=-cosd(vmc[i].force_deng[1])*kp_deng_use*
										 LIMIT(kp_deng_all,-2*vmc_all.kp_deng[i],2*vmc_all.kp_deng[i])
										 *0.4/(vmc_all.gait_time[1]+0.0000001); 

							if(flag_att_force[Yr])
								vmc[i].force_deng[0]+=vmc[i].force_deng[0]*cosd(fabs(pit_err));//test
							 
							vmc[i].force_deng[1]+=T_GAIN*MAX_DENG_ANGLE/2/(vmc_all.stance_time+vmc_all.delay_time[2]+vmc_all.gait_delay_time)*dt;
							if(vmc[i].force_deng[1]>MAX_DENG_ANGLE/2*0.98){
							 vmc[i].force_deng[2]=vmc[i].force_deng[1]=0;
							}
							}else 
							vmc[i].force_deng[1]=vmc[i].force_deng[0]=0;
						}
					}else
					 cal_force_deng(&vmc[i],dt);
				}
				#else
		     //正旋下蹬腿模式
				 if(deng_sel==1){//COS
					if(vmc[i].ground){
							deng_control(&vmc[i],dt);
							vmc_all.kp_deng_ctrl[1][i]=vmc_all.kp_deng_ctrl[0][i];
							float kp_deng_all=vmc_all.kp_deng[i]//Basic 
																+vmc_all.kp_deng_ctrl[1][i]//ROL err
																;//COG err
							float kp_deng_use=1;
							
							if(vmc[i].force_deng[2]>0&&vmc_all.power_state>=2){
							 if(vmc[i].force_deng[2]==1)
									 vmc[i].force_deng[0]=-cosd(vmc[i].force_deng[1])*kp_deng_use*
										 LIMIT(kp_deng_all,-2*vmc_all.kp_deng[i],2*vmc_all.kp_deng[i])
										 *0.4/(vmc_all.gait_time[1]+0.0000001); 
							 else if(vmc[i].force_deng[2]==2)
									 vmc[i].force_deng[0]=-cosd(vmc[i].force_deng[1])*kp_deng_use*
										 LIMIT(kp_deng_all,-2*vmc_all.kp_deng[i],2*vmc_all.kp_deng[i])
										 *0.4/(vmc_all.gait_time[1]+0.0000001); 

							if(flag_att_force[Yr])
								vmc[i].force_deng[0]+=vmc[i].force_deng[0]*cosd(fabs(pit_err));//test
							 
							vmc[i].force_deng[1]+=T_GAIN*MAX_DENG_ANGLE/2/(vmc_all.stance_time+vmc_all.delay_time[2]+vmc_all.gait_delay_time)*dt;
							if(vmc[i].force_deng[1]>MAX_DENG_ANGLE/2*0.98){
							 vmc[i].force_deng[2]=vmc[i].force_deng[1]=0;
							}
							}else 
							vmc[i].force_deng[1]=vmc[i].force_deng[0]=0;
						}
					}else{//SIN
						if(vmc[i].ground){
							deng_control(&vmc[i],dt);
							vmc_all.kp_deng_ctrl[1][i]=vmc_all.kp_deng_ctrl[0][i];
							float kp_deng_all=vmc_all.kp_deng[i]//Basic 
																+vmc_all.kp_deng_ctrl[1][i]//ROL err
																;//COG err
							float kp_deng_use=1;

							if(vmc[i].force_deng[2]>0&&vmc_all.power_state>=2){
								if(vmc[i].force_deng[2]==1)//正常
									 vmc[i].force_deng[0]=-sind(vmc[i].force_deng[1])*kp_deng_use*
										 LIMIT(kp_deng_all,-2*vmc_all.kp_deng[i],2*vmc_all.kp_deng[i])
										 *0.4/(vmc_all.gait_time[1]+0.0000001); 	
							 if(vmc[i].force_deng[2]==2)//继续蹬腿 或回蹬腿
									 vmc[i].force_deng[0]=-sind(vmc[i].force_deng[1])*kp_deng_use*
										 LIMIT(kp_deng_all,-2*vmc_all.kp_deng[i],2*vmc_all.kp_deng[i])
										 *0.4/(vmc_all.gait_time[1]+0.0000001); 
							
							 if(flag_att_force[Yr])
								vmc[i].force_deng[0]+=vmc[i].force_deng[0]*cosd(fabs(pit_err));//test
							 
							vmc[i].force_deng[1]+=T_GAIN*MAX_DENG_ANGLE/(vmc_all.stance_time)*dt;
							if(vmc[i].force_deng[1]>MAX_DENG_ANGLE/T_GAIN){
							 vmc[i].force_deng[2]=vmc[i].force_deng[1]=0;
							}
							}else 
							 vmc[i].force_deng[1]=vmc[i].force_deng[0]=0;
						}
					}
					
					#if USE_ATT_POLAR	//旋转Z轴下蹬力
					float pit_err1=-vmc_all.att_trig[3];	
					float vec_x=0;
					float vec_z=vmc[i].force_deng[0];
					//vmc[i].force[Xr] =cosd(pit_err1)*vec_x-sind(pit_err1)*vec_z;
					vmc[i].force_deng[0] =sind(pit_err1)*vec_x+cosd(pit_err1)*vec_z;
					#endif
				#endif  //end deng
							
				leg_tar_publish(&vmc[i],dt);
			 #if CHECK_USE_SENSOR
       }else//
				leg_check_publish(&vmc[i],dt);//
			 #endif
		  } 

			 //虚拟力反馈控制
			if(vmc[i].ground){	
			 cal_jacobi(&vmc[i]);
			 if(vmc_all.power_state==3)
			   cal_force_y(&vmc[i],dt);
			 cal_force_z(&vmc[i],dt);	
			 		 
			 //倾斜补偿
			 vmc[i].force[Xr]=vmc[i].force_cmd[Xr]*cosd(pit_err)//test
												-flag_att_force[Xr]*vmc[i].force_cmd[Zr]*sind((pit_err));
 			 vmc[i].force[Zr]=vmc[i].force_cmd[Zr]*cosd((pit_err))
												+flag_att_force[Zr]*fabs(vmc[i].force_cmd[Xr]*sind((pit_err)));//test
			 
			 if(flag_att_force[Xr]==0&&flag_att_force[Zr]==0)
				{vmc[i].force[Xr]=vmc[i].force_cmd[Xr]; vmc[i].force[Zr]=vmc[i].force_cmd[Zr];}
				
			 #if USE_ATT_POLAR	//旋转X轴力
				 float pit_err1=-vmc_all.att_trig[3];	
				 float vec_x=vmc[i].force[Xr];
				 float vec_z=0;//vmc[i].force[Zr];
					vmc[i].force[Xr] =cosd(pit_err1)*vec_x;//-sind(pit_err1)*vec_z;
					vmc[i].force[Zr]+=sind(pit_err1)*vec_x*en_fix[3];//+cosd(pit_err1)*vec_z;
			 #endif
				
		 //腿部未触地延伸
//			 if(vmc_all.use_ground_sensor&&
//				  vmc_all.hand_hold==0&&vmc_all.power_state>=2)
//				 trig_extend(&vmc[i],dt);
			 
			 out_range_protect(); 
				
			 #if USE_LOW_TORQUE
			  cal_torque(&vmc[i]);
			 #else
			  cal_torque1(&vmc[i],FLT_TORQUE,dt);
			 #endif
			 //虚拟电机模型
			 vmc[i].sita1+=vmc[i].param.spd_dj[0]*dt;
			 vmc[i].sita2+=vmc[i].param.spd_dj[1]*dt;
		  }
		 }
   }	 
	 LIMIT(vmc[i].sita1,-40,90);LIMIT(vmc[i].sita2,90,210);
	 //计算PWM
	 if(vmc_all.sita_test[4]){//强制角度测试
	 for(i=0;i<4;i++){
		 vmc[i].sita1=vmc_all.sita_test[0];
		 vmc[i].sita2=vmc_all.sita_test[1];
		 convert_mine_to_vmc_test(&vmc[i]);
	 }
   }
	 else{
		 for(i=0;i<4;i++){
				 if(vmc[i].tar_epos_force.x!=0||vmc[i].tar_epos_force.z!=0){
		     vmc[i].param.tar_epos.x=vmc[i].tar_epos_force.x;
		     vmc[i].param.tar_epos.z=vmc[i].tar_epos_force.z;    
		     inv_end_state(vmc[i].param.tar_epos.x,vmc[i].param.tar_epos.z,&vmc[i].sita1,&vmc[i].sita2);
				 }
				}
		 convert_mine_to_vmc(&vmc[FL1]);convert_mine_to_vmc(&vmc[BL1]);
		 convert_mine_to_vmc(&vmc[FL2]);convert_mine_to_vmc(&vmc[BL2]);
   }		
  
   for(i=0;i<10;i++)
    timer_task[i]+=dt;	 
 }
}
#endif