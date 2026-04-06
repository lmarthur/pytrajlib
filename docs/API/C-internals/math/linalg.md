# Linalg

## `gaussian_cartvec`

Generate a cartesian vector with independent N(0, 1) entries.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `gaussian_anglevec`

Generate an angle vector with independent N(0, 1) entries.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `anglevec` |  |

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

Multiply a 3-vector by a scalar

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vec` | `cartvec` |  |
| `s` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `smultiply_angle`

Multiply an anglevec by a scalar

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vec` | `anglevec` |  |
| `s` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `anglevec` |  |

## `sdivide`

Divide a 3-vector by a scalar

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vec` | `cartvec` |  |
| `s` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `multiply_anglevec`

Elementwise multiplication of two anglevecs

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `anglevec` |  |
| `b` | `anglevec` |  |

### Returns

| Type | Description |
| --- | --- |
| `anglevec` |  |

## `add`

Add vector b to vector a

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `cartvec` |  |
| `b` | `cartvec` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `add_anglevec`

Add anglevec b to anglevec a

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `anglevec` |  |
| `b` | `anglevec` |  |

### Returns

| Type | Description |
| --- | --- |
| `anglevec` |  |

## `subtract`

Subtract vector b from vector a

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `cartvec` |  |
| `b` | `cartvec` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

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
Uses Rodrigues' rotation formula

>Wikipedia contributors. (2026). Rodrigues’ rotation formula—Wikipedia, The
Free Encyclopedia.
https://en.wikipedia.org/w/index.php?title=Rodrigues%27_rotation_formula&oldid=1340370650

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `v` | `cartvec` |  |
| `k` | `cartvec` |  |
| `angle` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `project`

Project vector v onto vector u

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `v` | `cartvec` |  |
| `u` | `cartvec` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `gram_schmidt_orthonorm`

Create an orthonormal basis e0, e1 from linearly independent vectors
v0, v1 where e0 = v0 / norm(v0)

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `v0` | `cartvec` |  |
| `v1` | `cartvec` |  |
| `e0` | `cartvec *` |  |
| `e1` | `cartvec *` |  |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |
