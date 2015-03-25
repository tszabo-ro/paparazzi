#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_sf_bessel.h>
#include <gsl/gsl_matrix.h>

double lside;
double eta;
double ceta;
double seta;
gsl_vector *v_unit_x;
gsl_vector *v_unit_y;
gsl_vector *v_home;
gsl_matrix *m_rotate_2d;

double mav_xy[2];
#define PI 3.14159265358979323846

typedef struct {
    double w;
    /*double x;*/
    double y;
    /*double z;*/
} qua2d;
typedef struct {
    double x;
    double y;
    double c1;
    double s1;
    double mag;
    double angle;
    qua2d q;
} vec2d;

typedef struct {
    vec2d x;
    vec2d y;
    vec2d offset;
} coord_sys;

coord_sys xy_abs;
coord_sys xy_grd;
coord_sys xy_hme;


