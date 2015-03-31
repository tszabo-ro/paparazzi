vec2d a,b,c;
float gamma1,gamma2;


void bang1(void){
    a = vec2d_init(0.9,0);
    b = vec2d_init(0,0);
    gamma1 = BANG;
}

void bang2(void){
    gamma2 = BANG2;
}

void print_output(void){
    c = vec2d_triangulate(a,b,gamma1,gamma2);
    printf("Coords: %f,%f\n",c.x,c.y);
}

