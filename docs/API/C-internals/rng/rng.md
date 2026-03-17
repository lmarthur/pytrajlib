# Rng

## `ran_flat`

Returns a uniformly distributed random number in the range `[min, max)`.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `min` | `double` | Minimum value of the range. |
| `max` | `double` | Maximum value of the range. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Uniformly distributed random number. |

## `ran_gaussian`

Returns a normally distributed random number with mean `0` and standard
deviation `stddev`.

Uses the Box-Muller transform to generate two independent standard normal
random variables from two independent uniform random variables. Only one of
the generated random variables is returned per call.

Modified from Winkler, J. R., Numerical Recipes in C: The Art of Scientific
Computing (1993), p. 289-290.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `stddev` | `double` | Standard deviation of the normal distribution. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Normally distributed random number. |
