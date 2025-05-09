#ifndef RNG_H
#define RNG_H


#include <math.h>
#include "mt19937-64/mt64.h"

double ran_flat(double min, double max) {
    /*
    Return a uniformly distributed random number in the range [min, max]

    INPUTS:
    ----------
        min: double
            minimum value of the range
        max: double
            maximum value of the range
    OUTPUTS:
    ----------
        double: uniformly distributed random number
     */
    return genrand64_real1() * (max - min) + min;
}

double ran_gaussian(double stddev) {
    /* 
    Return a normally distributed random number with mean 0 and standard deviation 
    stddev. Uses the Box-Muller transform to generate two independent standard normal
    random variables from two independent uniform random variables. Only one of 
    the random variables is returned per call. 

    Modified from p. 289-290 of Winkler, J. R. Numerical recipes in C: The Art of 
    Scientific Computing. (1993).

    INPUTS:
    ----------
        stddev: double
            standard deviation of the normal distribution
    OUTPUTS:
    ----------
        double: normally distributed random number
     */
    double ran1(long *idum); 
    static int iset=0; 
    static double gset; 
    double fac,rsq,v1,v2;
    
    if (iset == 0) {
        do {  
            v1=2.0*genrand64_real1()-1.0; 
            v2=2.0*genrand64_real1()-1.0; 
            rsq=v1*v1+v2*v2;
        } while (rsq >= 1.0 || rsq == 0.0);
        fac=sqrt(-2.0*log(rsq)/rsq);
        gset=v1*fac;
        iset=1;
        return v2*fac;
    } else { 
         iset=0;
         return gset;
    }
}

#endif