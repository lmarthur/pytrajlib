# Lift

We model lift acceleration during the reentry period below 100km altitude.

There are two components of the state that model the lift acceleration: the
available lift and the true lift. The available lift is an intermediate step to
calculate the true lift, which is used to update the vehicle's velocity.

## `project_and_clip`

Project a vector into the lift plane, clip, then reconstruct in Cartesian.

Given orthonormal lift-plane basis vectors $(\mathbf e_2,\mathbf e_3)$,
the vector components are clipped independently to
$[-\text{max\_val},\text{max\_val}]$ and mapped back.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `yhat` | `cartvec` | Lift-plane basis vector |
| `zhat` | `cartvec` | Lift-plane basis vector |
| `arr` | `cartvec` | Vector to project and clip |
| `max_val` | `double` | Absolute clip limit in projected coordinates |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Projected and clipped vector in Cartesian coordinates |
