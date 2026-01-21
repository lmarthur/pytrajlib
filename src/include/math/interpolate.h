#ifndef INTERPOLATE_H
#define INTERPOLATE_H

double linterp(double x, double xs[], double ys[], int n){
    /*
    Linear interpolation function

    INPUTS:
    ----------
        x: double
            value to interpolate
        xs: double *
            pointer to the x-values
        ys: double *
            pointer to the y-values
        n: int
            number of data points
    OUTPUTS:
    ----------
        y: double
            interpolated value
    */

    // Initialize the output value
    double y = 0;

    // Find the two points to interpolate between
    int i = 0;
    while (x > xs[i]){
        i++;
    }

    if (i == 0){
        y = ys[0];
        return y;
    }

    // Perform the interpolation
    y = ys[i-1] + (ys[i] - ys[i-1]) * (x - xs[i-1]) / (xs[i] - xs[i-1]);

    return y;
}

#endif