# Linalg

## `zeros`

Create a zero 3-vector.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Vector with all components equal to zero. |

## `gaussian_cartvec`

Generate a cartesian vector with independent N(0, 1) entries.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Gaussian random 3-vector. |

## `gaussian_anglevec`

Generate an angle vector with independent N(0, 1) entries.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `anglevec` | Gaussian random angle vector. |

## `dot`

Compute the dot product of two 3-vectors: result = a · b

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `cartvec` | First vector |
| `b` | `cartvec` | Second vector |

### Returns

| Type | Description |
| --- | --- |
| `double` | The scalar dot product |

## `norm`

Get the L2 norm of a 3-vector

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vec` | `cartvec` | Vector to compute norm of |

### Returns

| Type | Description |
| --- | --- |
| `double` | The L2 norm |

## `smultiply`

Multiply a 3-vector by a scalar.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vec` | `cartvec` | Vector to scale. |
| `s` | `double` | Scalar multiplier. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Scaled vector. |

## `smultiply_angle`

Multiply an angle vector by a scalar.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vec` | `anglevec` | Angle vector to scale. |
| `s` | `double` | Scalar multiplier. |

### Returns

| Type | Description |
| --- | --- |
| `anglevec` | Scaled angle vector. |

## `sdivide`

Divide a 3-vector by a scalar.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vec` | `cartvec` | Vector to divide. |
| `s` | `double` | Scalar divisor. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Scaled vector. |

## `multiply_anglevec`

Elementwise multiplication of two angle vectors.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `anglevec` | First angle vector. |
| `b` | `anglevec` | Second angle vector. |

### Returns

| Type | Description |
| --- | --- |
| `anglevec` | Elementwise product. |

## `add`

Add vector b to vector a.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `cartvec` | First vector. |
| `b` | `cartvec` | Second vector. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Sum vector. |

## `add_anglevec`

Add angle vector b to angle vector a.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `anglevec` | First angle vector. |
| `b` | `anglevec` | Second angle vector. |

### Returns

| Type | Description |
| --- | --- |
| `anglevec` | Sum angle vector. |

## `subtract`

Subtract vector b from vector a.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `cartvec` | First vector. |
| `b` | `cartvec` | Second vector. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Difference vector. |

## `matvec_multiply`

Multiply a 3x3 matrix by a 3-vector: result = matrix * vec

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `double matrix[3][3]` | `-` |  |
| `vec` | `cartvec` | 3-vector |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | The product vector |

## `cross`

Compute the cross product of two 3-vectors: result = a x b

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `cartvec` | First vector |
| `b` | `cartvec` | Second vector |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | The cross product vector |

## `rotate`

Rotate vector v around unit vector k by specified angle.
Uses Rodrigues' rotation formula.

>Wikipedia contributors. (2026). Rodrigues' rotation formula—Wikipedia, The
Free Encyclopedia.
https://en.wikipedia.org/w/index.php?title=Rodrigues%27_rotation_formula&oldid=1340370650

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `v` | `cartvec` | Vector to rotate. |
| `k` | `cartvec` | Unit vector defining rotation axis. |
| `angle` | `double` | Rotation angle in radians. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Rotated vector. |

## `project`

Project vector v onto vector u.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `v` | `cartvec` | Vector to project. |
| `u` | `cartvec` | Direction to project onto. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Projection of v onto u. |

## `gram_schmidt_orthonorm`

Create an orthonormal basis e0, e1 from linearly independent vectors
v0, v1 where e0 = v0 / norm(v0).

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `v0` | `cartvec` | First input vector. |
| `v1` | `cartvec` | Second input vector. |
| `e0` | `cartvec *` | Output first orthonormal basis vector. |
| `e1` | `cartvec *` | Output second orthonormal basis vector. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |
