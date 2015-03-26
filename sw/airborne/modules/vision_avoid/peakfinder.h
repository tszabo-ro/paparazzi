#ifndef PEAKFINDER_H
#define PEAKFINDER_H
/*
struct valueIndexPair
{
  float value;
  int   index;
};*/

//extern void valueIndexSort(struct valueIndexPair* data, int N);

//extern void peakfinder (int cols, int rows, float *hflow, float threshold, float vangle, float * np, float * angle);
void peakfinder (int Ncols, int Nflows, int *hPos, int *hFlow, float threshold, float vangle, int *np, float * angle);
#endif
