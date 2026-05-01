# Gravity

## `get_gravity_acc`

The gravity model assumes a spherical Earth of radius
$R_e = 6{,}371{,}000\;m$, and uncertainties in geoid height are modeled with
an uncertainty parameter $\delta h$. The acceleration due to gravity is a
function of geoid height, current radius from Earth's center $r$, and the
gravitational acceleration at the surface $g_0$ ($m/s^2$):
$$
\begin{align}
a_\text{grav} = g_0 \frac{(R_e + \delta h)^2}{r^2}.
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
