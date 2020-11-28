#include "my_math.h"
#include "math.h"
#include "vmc.h"

void get_trajecotry_end( float p0, float v0, float a0, float a,float  b, float g, float t ,float *pos,float *acc,float *spd){
	*pos=p0 + v0*t + (1/2.0)*a0*t*t + (1/6.0)*g*t*t*t + (1/24.0)*b*t*t*t*t + (1/120.0)*a*t*t*t*t*t;
	*acc=a0 + g*t  + (1/2.0)*b*t*t  + (1/6.0)*a*t*t*t;
	*spd=v0 + a0*t + (1/2.0)*g*t*t  + (1/6.0)*b*t*t*t + (1/24.0)*a*t*t*t*t;
}

void plan_end(float p0,float v0,float a0,float pf,float vf,float af,float Tf,char flag[3],float *a,float *b,float *g,float *cost){

	 char posGoalDefined=flag[0];
	 char velGoalDefined=flag[1];
	 char accGoalDefined=flag[2];
//define starting position:
   float delta_a = af - a0;
   float delta_v = vf - v0 - a0*Tf;
   float delta_p = pf - p0 - v0*Tf - 0.5*a0*Tf*Tf;

 // %powers of the end time:
    float T2 = Tf*Tf;
    float T3 = T2*Tf;
    float T4 = T3*Tf;
    float T5 = T4*Tf;
  
 // %solve the trajectories, depending on what's constrained:
  if(posGoalDefined && velGoalDefined && accGoalDefined){
    *a = ( 60*T2*delta_a - 360*Tf*delta_v + 720* 1*delta_p)/T5;
    *b = (-24*T3*delta_a + 168*T2*delta_v - 360*Tf*delta_p)/T5;
    *g = (  3*T4*delta_a -  24*T3*delta_v +  60*T2*delta_p)/T5;
	}
  else if( posGoalDefined && velGoalDefined){
    *a = (-120*Tf*delta_v + 320*   delta_p)/T5;
    *b = (  72*T2*delta_v - 200*Tf*delta_p)/T5;
    *g = ( -12*T3*delta_v +  40*T2*delta_p)/T5;
	}
  else if (posGoalDefined && accGoalDefined){
    *a = (-15*T2*delta_a + 90*   delta_p)/(2*T5);
    *b = ( 15*T3*delta_a - 90*Tf*delta_p)/(2*T5);
    *g = (- 3*T4*delta_a + 30*T2*delta_p)/(2*T5);
	}
  else if(velGoalDefined && accGoalDefined){
    *a = 0;
    *b = ( 6*Tf*delta_a - 12*   delta_v)/T3;
    *g = (-2*T2*delta_a +  6*Tf*delta_v)/T3;
	}
  else if(posGoalDefined){  
    *a =  20*delta_p/T5;
    *b = -20*delta_p/T4;
    *g =  10*delta_p/T3;
	}
  else if(velGoalDefined){
    *a = 0;
    *b =-3*delta_v/T3;
    *g = 3*delta_v/T2;
	}
  else if(accGoalDefined){
    *a = 0;
    *b = 0;
    *g = delta_a/Tf;
	}
  else{
    *a = 0;
    *b = 0;
    *g = 0;
  }
 //Calculate the cost:
  *cost =  *g* *g + *b* *g*Tf + *b* *b*T2/3.0 + *a* *g*T2/3.0 + *a* *b*T3/4.0 + *a* *a*T4/20.0;
}

int gait_trj_planner_init(VMC* in, GAIT_SHEC* gait,char end_sel)
{
	static char init[4];
	char flag[3];
	char i;
	int spd_flag=1;
	float cost;
	if(!init[in->param.id]){init[in->param.id]=1;
		in->param.end_planner.spd_lift=0.03;//
		in->param.end_planner.scale_x=0.25;//
		in->param.end_planner.scale_z=0.5;//
		in->param.end_planner.scale_lift=1.5;//
	}
	
	if(gait->T_sw==0)
		return 0;
	
	in->param.end_planner.Tsw=gait->T_sw;
	in->param.end_planner.leg_h=in->param.delta_h;
	
	in->param.end_planner.leg_dis=   sqrt(pow(in->st_pos.x-in->tar_pos.x,2)
																			 +pow(in->st_pos.y-in->tar_pos.y,2)
																			 +pow(in->st_pos.z-in->tar_pos.z,2));
  if(in->tar_pos.x>in->st_pos.x)
		spd_flag=1;
	else
		spd_flag=-1;
	
	in->param.end_planner.max_spd=in->param.end_planner.leg_dis/in->param.end_planner.Tsw;
	in->param.end_planner.limit_spd= in->param.end_planner.max_spd*3;

	in->param.end_planner.T12=in->param.end_planner.Tsw*0.2;
	in->param.end_planner.T23=(in->param.end_planner.Tsw-in->param.end_planner.T12*2)/2;
	in->param.end_planner.T34=in->param.end_planner.T23;
	in->param.end_planner.T45=in->param.end_planner.T12;
	
	//%1 start
	in->param.end_planner.p1[0]=in->st_pos.x;
	in->param.end_planner.p1[1]=in->st_pos.y;
	in->param.end_planner.p1[2]=in->st_pos.z;
	#if EN_PLAN_USE_JERK==2
	in->param.end_planner.v1[0]=-in->spd.x;
	#else
	in->param.end_planner.v1[0]=-in->param.end_planner.max_spd*spd_flag;
	#endif
	in->param.end_planner.v1[1]=0;
	in->param.end_planner.v1[2]=in->param.end_planner.spd_lift;

	//%2 lift
	in->param.end_planner.p2[0]=in->st_pos.x
														 -in->param.end_planner.leg_dis/2*in->param.end_planner.scale_x*spd_flag;
	in->param.end_planner.p2[1]=0;
	if(spd_flag)
			in->param.end_planner.p2[2]=in->st_pos.z
															 +in->param.end_planner.leg_h*in->param.end_planner.scale_z/1.75;
	else
			in->param.end_planner.p2[2]=in->st_pos.z
															 +in->param.end_planner.leg_h*in->param.end_planner.scale_z/1;
	in->param.end_planner.v2[0]=0;
	in->param.end_planner.v2[1]=0;
	in->param.end_planner.v2[2]=in->param.end_planner.max_spd*in->param.end_planner.scale_lift;

	flag[0]=1;
	flag[1]=1;
	flag[2]=0;
	for(i=0;i<3;i++){
		if(i==0 || i==2)
			plan_end(
			in->param.end_planner.p1[i],
			in->param.end_planner.v1[i],
			in->param.end_planner.a1[i],
			in->param.end_planner.p2[i],
			in->param.end_planner.v2[i],
			in->param.end_planner.a2[i],
			in->param.end_planner.T12,flag,
			&in->param.end_planner.a12[i],
			&in->param.end_planner.b12[i],
			&in->param.end_planner.g12[i],&cost);
	}
	//%3 ------------------------mid--------------------------
	in->param.end_planner.p3[0]=in->st_pos.x/2+in->tar_pos.x/2;
	in->param.end_planner.p3[1]=in->st_pos.y/2+in->tar_pos.y/2;
	in->param.end_planner.p3[2]=in->st_pos.z/2+in->tar_pos.z/2+in->param.end_planner.leg_h;
	#if EN_PLAN_USE_JERK==2
	in->param.end_planner.v3[0]=in->spd.x*3;
	#else
	in->param.end_planner.v3[0]=in->param.end_planner.limit_spd*spd_flag;
	#endif
	in->param.end_planner.v3[1]=0;
	in->param.end_planner.v3[2]=0;

	flag[0]=1;
	flag[1]=1;
	flag[2]=0;
	for(i=0;i<3;i++){
		if(i==0 || i==2)
			plan_end(
			in->param.end_planner.p2[i],
			in->param.end_planner.v2[i],
			in->param.end_planner.a2[i],
			in->param.end_planner.p3[i],
			in->param.end_planner.v3[i],
			in->param.end_planner.a3[i],
			in->param.end_planner.T23,flag,
			&in->param.end_planner.a23[i],
			&in->param.end_planner.b23[i],
			&in->param.end_planner.g23[i],&cost);
	}
	//%4 down
	in->param.end_planner.p4[0]=in->tar_pos.x
														 +in->param.end_planner.leg_dis/2*in->param.end_planner.scale_x*spd_flag;
	in->param.end_planner.p4[1]=0;
	if(spd_flag)
		in->param.end_planner.p4[2]=in->tar_pos.z
																 +in->param.end_planner.leg_h*in->param.end_planner.scale_z;
	else
		in->param.end_planner.p4[2]=in->tar_pos.z
																 +in->param.end_planner.leg_h*in->param.end_planner.scale_z/1.75;
	in->param.end_planner.v4[0]=0;
	in->param.end_planner.v4[1]=0;
	in->param.end_planner.v4[2]=-in->param.end_planner.max_spd*in->param.end_planner.scale_lift;

	flag[0]=1;
	flag[1]=1;
	flag[2]=1;
	for(i=0;i<3;i++){
		if(i==0 || i==2)
			plan_end(
			in->param.end_planner.p3[i],
			in->param.end_planner.v3[i],
			in->param.end_planner.a3[i],
			in->param.end_planner.p4[i],
			in->param.end_planner.v4[i],
			in->param.end_planner.a4[i],
			in->param.end_planner.T34,flag,
			&in->param.end_planner.a34[i],
			&in->param.end_planner.b34[i],
			&in->param.end_planner.g34[i],&cost);
	}
  
	//%5 end
	in->param.end_planner.p5[0]=in->tar_epos.x;
	in->param.end_planner.p5[1]=in->tar_epos.y;
	in->param.end_planner.p5[2]=in->tar_epos.z;
	#if EN_PLAN_USE_JERK==2
	in->param.end_planner.v5[0]=-in->spd.x;
	#else
	in->param.end_planner.v5[0]=-in->param.end_planner.max_spd*spd_flag;
	#endif
	in->param.end_planner.v5[1]=0;
	in->param.end_planner.v5[2]=0;
	
	flag[0]=1;
	flag[1]=1;
	flag[2]=1;
	for(i=0;i<3;i++){
		if(i==0 || i==2)
			plan_end(
			in->param.end_planner.p4[i],
			in->param.end_planner.v4[i],
			in->param.end_planner.a4[i],
			in->param.end_planner.p5[i],
			in->param.end_planner.v5[i],
			in->param.end_planner.a5[i],
			in->param.end_planner.T45,flag,
			&in->param.end_planner.a45[i],
			&in->param.end_planner.b45[i],
			&in->param.end_planner.g45[i],&cost);
	}
}

int gait_trj_get(VMC* in, GAIT_SHEC* gait,float time_now){
float time=time_now;//gait->S_sw[in->param.id]*gait->T_sw;
if(time_now!=0)
	 time=time_now;
time=LIMIT(time,0,in->param.end_planner.Tsw);
float pos,acc,spd;
char i=0;
for(i=0;i<3;i++){
if(i==0 || i==2){
		if (time<=in->param.end_planner.T12)
				get_trajecotry_end( 
				in->param.end_planner.p1[i],
				in->param.end_planner.v1[i],
				in->param.end_planner.a1[i],
				in->param.end_planner.a12[i],
				in->param.end_planner.b12[i],
				in->param.end_planner.g12[i],
				time,
				&pos,&acc,&spd);
		else if (time<=in->param.end_planner.T12+in->param.end_planner.T23)
				get_trajecotry_end( 
				in->param.end_planner.p2[i],
				in->param.end_planner.v2[i],
				in->param.end_planner.a2[i],
				in->param.end_planner.a23[i],
				in->param.end_planner.b23[i],
				in->param.end_planner.g23[i],
				time-in->param.end_planner.T12,
				&pos,&acc,&spd);
		else if (time<=in->param.end_planner.T12+in->param.end_planner.T23+in->param.end_planner.T34)
				get_trajecotry_end( 
				in->param.end_planner.p3[i],
				in->param.end_planner.v3[i],
				in->param.end_planner.a3[i],
				in->param.end_planner.a34[i],
				in->param.end_planner.b34[i],
				in->param.end_planner.g34[i],
				time-in->param.end_planner.T12-in->param.end_planner.T23,
				&pos,&acc,&spd);
		else if (time<=in->param.end_planner.T12+in->param.end_planner.T23+in->param.end_planner.T34+in->param.end_planner.T45)
				get_trajecotry_end( 
				in->param.end_planner.p4[i],
				in->param.end_planner.v4[i],
				in->param.end_planner.a4[i],
				in->param.end_planner.a45[i],
				in->param.end_planner.b45[i],
				in->param.end_planner.g45[i],
				time-in->param.end_planner.T12-in->param.end_planner.T23-in->param.end_planner.T34,
				&pos,&acc,&spd);  
     else
        return 0;			 

		in->param.end_planner.pos_now[i]=pos;
		in->param.end_planner.spd_now[i]=spd;
		in->param.end_planner.acc_now[i]=acc;
	}
}
   return 1;
}

void end_plan_test(float dt)
{
 static char state;
 static float timer=0;
 char temp;
	gait.T_sw=1;//
	vmc[0].param.delta_h=0.5;//
	vmc[0].st_pos.x=-0.5;
	vmc[0].st_pos.y=0;
	vmc[0].st_pos.z=0;
	vmc[0].tar_pos.x=0.5;
	vmc[0].tar_pos.y=0;
	vmc[0].tar_pos.z=0;
	switch(state)
	{
		case 0:
			if(vmc[0].ground==0){
		 	  gait_trj_planner_init(&vmc[0], &gait, 0);
	      state=1;
		    timer=0.001;
			}
		break;
		case 1:
			  timer+=dt;
				temp=gait_trj_get(&vmc[0], &gait, timer);
		    if(timer>gait.T_sw)
					state=0;
		break;
	}
}