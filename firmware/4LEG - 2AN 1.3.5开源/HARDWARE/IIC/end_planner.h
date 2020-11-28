
typedef struct 
{ 
	float Tsw;//=0.25;
	float leg_dis;//=0.5;
	float leg_h;//=0.25;

	float spd_lift;//=0.8;
	float scale_x;//=1.25;
	float scale_y;//=0.5;
	float scale_lift;//=1.5;
	float max_spd;//=leg_dis/Tsw;
	float limit_spd;//= max_spd*4;

	float T12;//=Tsw*0.2;%???????
	float T45;//=T12;
	float T23;//=(Tsw-T12*2)/2;
	float T34;//=T23;

}END_PLANNER;

void get_trajecotry_end( float p0, float v0, float a0, float a,float  b, float g, float t ,float *pos,float *acc,float *spd);
void plan_end(float p0,float v0,float a0,float pf,float vf,float af,float Tf,char flag[3],float *a,float *b,float *g,float *cost);