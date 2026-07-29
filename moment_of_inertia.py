import numpy as np

# medium-large KEP whose diameter is 1/20 the length
l_cyl = 2.0
r_cyl = l_cyl / 20 / 2
V_cyl = np.pi * r_cyl**2 * l_cyl

# RV cone measurements
r_cone = 0.277
l_cone = 2.75
V_cone = np.pi * r_cone**2 * l_cone / 3

density_cyl = 17.6 / 1000 * 100**3  # g/cm^3 * 1kg/1000g * (100cm)^3/1m^3
m_cyl = density_cyl * V_cyl

# Assume total mass is 450kg
m_cone = 450 - m_cyl

# center of mass
x_cm_cone = 3 / 4 * l_cone
x_cm_cyl = 0.5 * l_cone
f_cm = (3 / 4 * m_cone + 0.5 * m_cyl) / (m_cone + m_cyl)
x_cm = f_cm * l_cone

# moment of inertia
I_cone = m_cone * (3 / 20 * r_cone**2 + 3 / 80 * l_cone**2)
I_cyl = 1 / 12 * m_cyl * (3 * r_cyl**2 + l_cyl**2)
I_cone_cm = I_cone + m_cone * (x_cm - x_cm_cone) ** 2
I_cyl_cm = I_cyl + m_cyl * (x_cm - x_cm_cyl) ** 2

print("cm fraction of length", f_cm)
print(f"{m_cone=}, {m_cyl=}")
print(f"mass total = {m_cone + m_cyl}")
print(f"{r_cyl=}")
print(f"I_total = {I_cone_cm + I_cyl_cm}")
