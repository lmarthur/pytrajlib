# Gravity

## `get_gravity_acc`

The gravity model assumes a spherical earth of radius $R_e = 6,371,000$ m, and
uncertainties in geoid height are modeled with an uncertainty parameter $\delta
h$ as described by Arthur and Kemp (2025). The acceleration due to gravity is a
function of the geoid height, current position, represented as the distance in
meters from the center of the Earth $r$, and the gravitational acceleration at
the surface $g_0$ (m/$s^2$):
$$
\begin{align}
\mathbf a_\text{grav} = -g_0 \frac{(R_e + \delta h)^2}{r^2} \hat{\mathbf r}.
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `grav` | `grav *` | Pointer to gravity model parameters |
| `state` | `state *` | Pointer to state updated with gravity acceleration |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |