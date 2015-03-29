#ifndef PEAKFINDER_H
#define PEAKFINDER_H

extern void peakfinder (int Ncols, int Nflows, int *hPos, int *hFlow, float threshold, float vangle, int *np, float * angle);

// The two functions below are the two parts of peakfinder. It is separated, so the summed flow can be overlayed on the downlinked image in debug mode
extern void cv_flowSum(int *hPos, int *hFlow, int NFlows, int NCols, float *flowSum, float *maxFlow);
extern void cv_peakFinder(float *flowSum, float maxFlow, int NCols, float threshold, int *np, float *angle, float visualAngle);
#endif
