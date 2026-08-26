import numpy as np
import sympy


def solve_nose_radius_base_radius():
    L, theta_n = sympy.symbols("L theta_n")
    R_b, r_sphere = sympy.symbols("R_b r_sphere", positive=True)

    values = {
        theta_n: float(
            sympy.rad(5.25)
        ),  # cone half angle 5.25 degrees --> radians. From Iliff and Shafer (1995)
        L: 2.75,  # tip-to-tail length is 2.75 m (Murbach et al. 1997)
    }

    sphere_horizontal_extent = r_sphere * (
        1 - sympy.sin(theta_n)
    )  # from basic geometry
    cone_x_neg_extent = (
        r_sphere * sympy.cos(theta_n) / sympy.tan(theta_n) - sphere_horizontal_extent
    )  # more basic trig
    total_cone_length = L + cone_x_neg_extent

    eq1 = r_sphere / R_b - 0.07  # from Iliff and Shafer (1995)
    eq2 = (
        R_b - total_cone_length * sympy.tan(theta_n)
    )  # The cone extends to x < 0, so to determine the base radius, we need to add that extension to the tip-to-tail length

    solns = sympy.solve([eq1, eq2], [R_b, r_sphere])
    for k, v in solns.items():
        values[k] = v.subs(values)
        print(k, v.subs(values))
    return values[R_b], values[r_sphere]


# Pershing II radar section
pershing_radar_r = 0.5 / 2
pershing_radar_h = 1.3
pershing_radar_mass = 105.7
pershing_radar_v = 1 / 3 * np.pi * pershing_radar_r**2 * pershing_radar_h
pershing_radar_density = pershing_radar_mass / pershing_radar_v

# medium-large KEP whose diameter is 1/20 the length
l_cyl = 2.0
r_cyl = l_cyl / 20 / 2
V_cyl = np.pi * r_cyl**2 * l_cyl

# RV cone measurements
r_cone, r_sphere = solve_nose_radius_base_radius()
l_cone = 2.75
V_cone = np.pi * r_cone**2 * l_cone / 3

# Keep the cone as a uniform density and use the excess density due to the KEP
density_tungsten = 17.6 / 1000 * 100**3  # g/cm^3 * 1kg/1000g * (100cm)^3/1m^3
density_cyl = density_tungsten - pershing_radar_density
m_cyl = density_cyl * V_cyl

# Assume the mass of the cone by multiplying the density of the Pershing II radar section by the volume of the cone
m_cone = pershing_radar_density * V_cone
total_mass = m_cone + m_cyl

# center of mass
x_cm_cone = 3 / 4 * l_cone
x_cm_cyl = 1.5  # place the cylinder 0.5m from the start of the cone. It ends at 2.5m
x_cm = (x_cm_cyl * m_cyl + x_cm_cone * m_cone) / total_mass
f_cm = x_cm / l_cone

# moment of inertia (each term uses its own effective mass, so they add directly)
I_cone = m_cone * (3 / 20 * r_cone**2 + 3 / 80 * l_cone**2)
I_cyl = 1 / 12 * m_cyl * (3 * r_cyl**2 + l_cyl**2)
I_cone_cm = I_cone + m_cone * (x_cm - x_cm_cone) ** 2
I_cyl_cm = I_cyl + m_cyl * (x_cm - x_cm_cyl) ** 2

print(f"{pershing_radar_density=}")
print(f"{density_tungsten=}, {density_cyl=}")
print(f"{V_cone=}")
print("cm fraction of length", f_cm, "cm location", x_cm)
print(f"{m_cone=}, {m_cyl=}")
print(f"mass total = {m_cone + m_cyl}")
print(f"{r_cyl=}, {x_cm_cyl=}")
print(f"I_total = {I_cone_cm + I_cyl_cm}")
