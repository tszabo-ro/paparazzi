#include "avoid_nav.h"
void sayhello()
{
    printf("hello!\n");
}


void vec2d_abs(vec2d* a) {
    a->mag = sqrt(pow(a->x,2)+pow(a->y,2));
}
void vec2d_cs(vec2d* a) {
    vec2d_abs(a);
    a->c1 = a->x/a->mag;
    a->s1 = a->y/a->mag;
}
void vec2d_q(vec2d* a) {
    vec2d_cs(a);
    a->q.w = sqrt(2+2*a->c1)/2;
    a->q.y = a->s1/2/a->q.w;
}
void vec2d_a_q(vec2d* a) {
    vec2d_q(a);
    a->angle = atan2(2*a->q.y*a->q.w,1-2*pow(a->q.y,2));
}

vec2d vec2d_init(double x, double y) {
    vec2d v;
    v.x = x;
    v.y = y;
    return v;
}

void vec2d_set(vec2d* v, double x, double y) {
    v->x=x;
    v->y=y;
}

vec2d arr2vec2d(double * arrxy) {
    vec2d v;
    v.x = arrxy[0];
    v.y = arrxy[1];
    return v;
}
vec2d vec2d_add(vec2d* a, vec2d* b) {

    vec2d v;
    v.x = a->x+b->x;
    v.y = a->y+b->y;
    return v;
}
vec2d vec2d_sub(vec2d* a, vec2d* b) {

    vec2d v;
    v.x = b->x-a->x;
    v.y = b->y-a->y;
    return v;
}
double vec2d_dot(vec2d* a, vec2d* b){
    
   /*printf("dot: %f\n",a->x*b->x+a->y*b->y);*/
    return (a->x*b->x+a->y*b->y);
    
}
double vec2d_ang(vec2d* a, vec2d* b){
    vec2d_abs(a);
    vec2d_abs(b);
    /*printf("abs a: %f abs b: %f\n",a->mag,b->mag);*/
    double dot = vec2d_dot(a,b);
    /*printf("costhe: %f\n",dot/a->mag/b->mag);*/
    return acos(dot/a->mag/b->mag);
}
double vec2d_ang_q(vec2d* a,vec2d* b){
    vec2d_a_q(a);
    vec2d_a_q(b);
    return b->angle-a->angle;
}
vec2d vec2d_norm(vec2d a){
    vec2d_abs(&a);
    a.x = a.x/a.mag;
    a.y = a.y/a.mag;
    return a;
}
vec2d hme2grd(vec2d* a){
    vec2d v;
    vec2d out;
    v = vec2d_add(a,&xy_grd.offset);
    out.x = vec2d_dot(&v,&xy_grd.x);
    out.y = vec2d_dot(&v,&xy_grd.y);
    return out;
}
vec2d grd2hme(vec2d* a){
    vec2d v;
    vec2d out;
    v = vec2d_add(a,&xy_grd.offset);
    out.x = vec2d_dot(&v,&xy_grd.x);
    out.y = vec2d_dot(&v,&xy_grd.y);
    return out;
}

void vec2d_print(vec2d* a){
    printf("[%f,%f]",a->x,a->y);
}



void printarr_double(double a[],int n){
    int i;
    printf("[");
    for (i=0; i<n;i++) {

        printf("%f, ",a[i]);
    }
    printf("]\n");
}
void printarr_int(int a[],int n){
    int i;
    printf("[");
    for (i=0; i<n;i++) {

        printf("%i, ",a[i]);
    }
    printf("]\n");
}

void print2darr_double(double* a,int n, int m){
    int i;
    int j;
    
    for(i=0;i<n;i++){
        printf("[");
        for(j=0;j<m;j++){
            /*printf("%i,%i %i %f",i,j,i*m+j,a[i*m+j]);*/
            printf("%f",a[i*m+j]);
            if (j<m-1){
                printf(", ");
            }
        }
        printf("]\n");
    }
}

void sw_arr_int(int *a,int s,int d){
    int tmp = a[d];
    a[d]=a[s];
    a[s]=tmp;
}
void sw_arr_double(double *a,int s,int d){
    double tmp = a[d];
    a[d]=a[s];
    a[s]=tmp;
}


#define OBS_SLOTS 10
#define GRID_RES 20
#define TRACKING_LIMIT 2
typedef struct{
    int n_matches;
    int n_drops;
    int n_new;
    int matches_s[OBS_SLOTS];
    int matches_d[OBS_SLOTS];
    int drop[OBS_SLOTS];
    int new[OBS_SLOTS];
} point_tracker;

void tracker_report (point_tracker track){
    printf("Track report\npoints matched: %i\nmatches:\n",track.n_matches);
    int i;
    for(i=0;i<track.n_matches;i++){
        printf("p%i to p%i\n",track.matches_s[i],track.matches_d[i]);
    }
    printf("dropped points:\n");
    for(i=0;i<track.n_drops;i++) printf("%i\n",track.drop[i]);
    printf("new points:\n");
    for(i=0;i<track.n_new;i++) printf("%i\n",track.new[i]);
    
}

void match_1d(point_tracker *track, double *s, double *d,int sn,int dn){

    double D[sn*dn];
    int dn_i[dn];
    int sn_j[sn];

    int i=0;
    int j=0;

    while (j<sn) {
        if (!isnan(s[i])){
            sn_j[j]=i;
            j++;
        }
        i++;
    }

    i=0;
    j=0;

    while (j<dn) {
        if (!isnan(d[i])){
            dn_i[j]=i;
            j++;
        }
        i++;
    }

    for (i=0;i<dn;i++){
        for(j=0;j<sn;j++){
            D[i*sn+j]=fabs(s[sn_j[j]]-d[dn_i[i]]);
        }
    }

    double tmp = INFINITY;
    double tmp2 = INFINITY;
    int pivot[] = {0,0};
    for (i=0;i<dn;i++){
        for(j=0;j<sn;j++){
            if(D[i*sn+j]<tmp){
                pivot[0]=i;
                pivot[1]=j;
                tmp = D[i*sn+j];
            }
        }
    }

    int i_index[dn];
    int j_index[sn];
    for (i = 0; i<dn; i++) i_index[i] = i;
    for (i = 0; i<sn; i++) j_index[i] = i;

    sw_arr_int(i_index,0,pivot[0]);
    sw_arr_int(j_index,0,pivot[1]);

    int n = (dn < sn) ? dn:sn;

    int k;

    for (k=0;k<n;k++) {
        tmp = D[i_index[k]*sn+j_index[k]];

        for (i=k;i<dn;i++){
            tmp2 = D[i_index[i]*sn+j_index[k]];
            if (tmp2<tmp){
                sw_arr_int(i_index,i,k);
                tmp = tmp2;
            }
        }
        for (i=k;i<sn;i++){
            tmp2 = D[i_index[k]*sn+j_index[i]];
            if (tmp2<tmp){
                sw_arr_int(j_index,i,k);
                tmp = tmp2;
            }
        }
    }

    double D_new[sn*dn];
    for (i=0;i<dn;i++){
        for (j=0;j<sn;j++){
            D_new[i*sn+j]=D[i_index[i]*sn+j_index[j]];
        }
    }



    int m = (dn > sn) ? dn:sn;
    int n_matches = 0;


    for (i=0;i<n;i++) {
        if (D_new[i*sn+i]<TRACKING_LIMIT) n_matches++;
    }

    track->n_matches = n_matches;
    track->n_drops = sn-n_matches;
    track->n_new = dn-n_matches;
    
    j = 0;
    k = 0;

    printf("Started filling up:\n");
    for(i=0;i<m;i++){
        if(i<n&&i<n_matches){
            track->matches_d[i] = dn_i[i_index[i]];
            track->matches_s[i] = sn_j[j_index[i]];
            track->matches_d[i] = dn_i[i_index[i]];
            track->matches_s[i] = sn_j[j_index[i]];
        }
        else{
            if(i<dn){
                track->new[j]=dn_i[i_index[i]];
                j++;
            }
            if(i<sn){
                track->drop[k]=sn_j[j_index[i]];
                k++;
            }
        }
    }


}



vec2d vec2d_triangulate(vec2d* a, vec2d* b, double gamma1, double gamma2) {
    double c1 = cos(gamma1);
    double s1 = sin(gamma1);
    double c2 = cos(gamma2);
    double s2 = sin(gamma2);
    double c1c2 = c1*c2;
    double c1s2 = c2*s2;
    double c2s1 = c2*s1;
    double s1s2 = s1*s2;
    double det = 1/(c1s2-c2s1);
    
    vec2d v;
    v.x = det*(-a->x*c2s1+a->x*c1s2+b->x*c1s2-b->y*c1c2);
    v.y = det*(-a->x*s1s2+a->x*s1s2+b->x*c1s2-b->y*c2s1);
    return v;
    
}

typedef struct {
    vec2d orig;
    double gamma_orig;
    vec2d upd;
    double gamma_upd;
    vec2d xy;
    int id;
    int n_tracked;
} obstacle;

typedef struct {
    int n_obs;
    obstacle obs[OBS_SLOTS];
    double angleslots_n[OBS_SLOTS];
    double angleslots_np1[OBS_SLOTS];
    int free_slots[OBS_SLOTS];

    double gridx_coord[GRID_RES];
    double gridy_coord[GRID_RES];

    double grid_weights_exp[GRID_RES][GRID_RES];
    double grid_weights_obs[GRID_RES][GRID_RES];
} Arena;

Arena arena;






/*SIMULATION*/
#define SIM_TIME 3
int sim_timestep;
vec2d sim_vehxy[SIM_TIME];
double sim_veho[SIM_TIME]={PI/4,PI/4,PI/4};



double sim_gamma[SIM_TIME][10] = {
        {0,NAN,NAN,NAN,NAN,NAN,NAN,100,NAN,NAN},
        {NAN,NAN,10,NAN,PI/4,NAN,NAN,NAN,NAN,NAN},
        {NAN,PI/2,NAN,NAN,NAN,NAN,300,NAN,200,NAN},
};

int sim_npoints[SIM_TIME] = {2,3,3};

void sim_init(){
    sim_vehxy[0] = vec2d_init(0,0);
    sim_vehxy[1] = vec2d_init(0,1);
    sim_vehxy[2] = vec2d_init(0,2);
    sim_timestep = 0;
    
    
}
/*SIMULATION END*/


void request_obstacles(double *v,int *n){
    printarr_double(sim_gamma[sim_timestep],10);
    int i;
    for(i=0;i<OBS_SLOTS;i++){
        v[i] = sim_gamma[sim_timestep][i];
    }
    *n = sim_npoints[sim_timestep];
}
void obstacle_update(double v[],int n,vec2d v_xy,double o){
    int i;
    if(n>0){
        if(arena.n_obs == 0){
            for(i=0;i<OBS_SLOTS;i++){
                arena.angleslots_n[i] = v[i];
                arena.free_slots[i] = 0;
            }
            arena.n_obs = n;
        }
    }

}
void request_position(vec2d *loc, double *orient){
    *loc = sim_vehxy[sim_timestep];
    *orient = sim_veho[sim_timestep];
}
void obstacle_add(vec2d loc, double orient, double gamma, int id){
    int i = 0;
    int freeslot = -1;
    while(freeslot==-1&&i<OBS_SLOTS){
        if(arena.free_slots[i]==1) freeslot = i;
        i++;
    }
    obstacle[i];

}
void obstacle_destroy(){}
void move(){}

void navigate(){
    printf("Navigating\n");
    double vision_tmp[OBS_SLOTS];
    int n_tmp = 0;
    request_obstacles(vision_tmp,&n_tmp);
    /*int n_points;*/
    /*request_obstacles(vision_tmp,);*/
    /*printf("%f\n",vision_tmp[0]);*/
    printarr_double(vision_tmp,OBS_SLOTS);
    printf("%i\n",n_tmp);
    vec2d v_xy;
    double o;
    request_position(&v_xy,&o);
    obstacle_update(vision_tmp,n_tmp,v_xy,o);
}



void init_map(double xy[][2],double* home_pos) {

    printf("Running map initizlization...\n");
    xy_grd.x = vec2d_norm(vec2d_init(xy[1][0]-xy[0][0],xy[1][1]-xy[0][1]));
    xy_grd.y = vec2d_norm(vec2d_init(xy[3][0]-xy[0][0],xy[3][1]-xy[0][1]));
    xy_grd.offset = vec2d_init(-xy[0][0],-xy[0][1]);

    xy_hme.x = vec2d_init(1,0);
    xy_hme.y = vec2d_init(0,1);
    xy_hme.offset = vec2d_init(0,0);

    lside = sqrt(pow(xy[1][0]-xy[0][0],2)+pow(xy[1][1]-xy[0][1],2));
    eta = atan2(abs(xy[1][1]-xy[0][1]),abs(xy[1][0]-xy[0][0]));
    ceta = cos(eta);
    seta = sin(eta);
    printf("Rectangle side: %f m ,rotation: %f deg\n",lside,eta*180/PI);

    double dl = lside/GRID_RES;
    int i;
    int j;
    for(i=0;i<GRID_RES;i++){
        arena.gridx_coord[i]=i*dl;
        arena.gridy_coord[i]=i*dl;
    }

    for(i=0;i<GRID_RES;i++){
        for(j=0;j<GRID_RES;j++){
            arena.grid_weights_exp[i][j]=0;
            arena.grid_weights_obs[i][j]=0;
        }
    }

    for(i=0;i<OBS_SLOTS;i++){
        arena.angleslots_n[i]=INFINITY;        
        arena.free_slots[i]=1;
    }
    
    arena.n_obs=0;

    sim_init();
}
