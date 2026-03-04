#ifndef RNG_H
#define RNG_H

#include "mt19937-64/mt64.h"
#include <math.h>

/**
 * Returns a uniformly distributed random number in the range `[min, max)`.
 *
 * @param min Minimum value of the range.
 * @param max Maximum value of the range.
 * @return Uniformly distributed random number.
 */
double ran_flat(double min, double max) {
  return genrand64_real1() * (max - min) + min;
}

/**
 * Returns a normally distributed random number with mean `0` and standard
 * deviation `stddev`.
 *
 * Uses the Box-Muller transform to generate two independent standard normal
 * random variables from two independent uniform random variables. Only one of
 * the generated random variables is returned per call.
 *
 * Modified from Winkler, J. R., Numerical Recipes in C: The Art of Scientific
 * Computing (1993), p. 289-290.
 *
 * @param stddev Standard deviation of the normal distribution.
 * @return Normally distributed random number.
 */
double ran_gaussian(double stddev) {
  double ran1(long *idum);
  static int iset = 0;
  static double gset;
  double fac, rsq, v1, v2;

  if (iset == 0) {
    do {
      v1 = 2.0 * genrand64_real1() - 1.0;
      v2 = 2.0 * genrand64_real1() - 1.0;
      rsq = v1 * v1 + v2 * v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0 * log(rsq) / rsq);
    gset = v1 * fac;
    iset = 1;
    return v2 * fac * stddev;
  } else {
    iset = 0;
    return gset * stddev;
  }
}

#endif