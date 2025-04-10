# This script contains code to generate system response plots for maneuverability analysis and validation.

import numpy as np
import matplotlib.pyplot as plt


# Define the system parameters
rv_density = 2000 # kg/m^3 (for uniform density assumption)

def atm_density(altitude, rho_0=1.225):
    """
    Estimate the atmospheric density at a given altitude using a simple exponential model.
    """
    # Constants
    scale_height = 8000  # m (scale height for the atmosphere)
    
    # Calculate atmospheric density
    rho = rho_0 * np.exp(-altitude / scale_height)
    return rho

# Define a center of mass function
def center_of_mass(radius, length_cylinder, length_cone):
    """
    Calculate the center of mass of a composite shape consisting of a cylinder and a cone, defined relative to the tip of the cone.
    """
    # Volume of the cylinder
    V_cylinder = np.pi * radius**2 * length_cylinder
    
    # Volume of the cone
    V_cone = (1/3) * np.pi * radius**2 * length_cone
    
    # Center of mass of the cylinder (from the tip of the cone)
    x_cylinder = length_cylinder / 2 + length_cone

    # Center of mass of the cone (from the tip of the cone)
    x_cone = 3 * length_cone / 4

    # Total volume
    V_total = V_cylinder + V_cone

    # Center of mass of the composite shape (from the tip of the cone)
    x_cm = -(V_cylinder * x_cylinder + V_cone * x_cone) / V_total

    return x_cm

# Define a center of pressure function

# Define a moment of inertia function
def moment_of_inertia(radius, length_cylinder, length_cone, density):
    """
    Calculate the moment of inertia of a composite shape consisting of a cylinder and a cone, defined relative to the center of mass, assuming uniform density.
    """
    
    # Mass of the cylinder
    m_cylinder = density * np.pi * radius**2 * length_cylinder

    # Mass of the cone
    m_cone = (1/3) * density * np.pi * radius**2 * length_cone

    # Moment of inertia of the cylinder about its own center of mass
    I_cylinder = (1/12) * m_cylinder * (3 * radius**2 + length_cylinder**2)
   
    # Moment of inertia of the cone about its own center of mass
    I_cone = m_cone * ( (3/20) * radius**2 + (3/80)*length_cone**2 )

    # Center of mass of the cone
    x_cone = - 3 * length_cone / 4

    # Center of mass of the cylinder
    x_cylinder = -length_cylinder / 2 - length_cone

    # Parallel axis theorem to shift to the center of mass of the composite shape
    x_cm = center_of_mass(radius, length_cylinder, length_cone)

    I_cylinder_cm = I_cylinder + m_cylinder * (x_cm - x_cylinder)**2

    I_cone_cm = I_cone + m_cone * (x_cm - x_cone)**2
    # Total moment of inertia
    I_total = I_cylinder_cm + I_cone_cm

    return I_total

# Define a moment function

# Define a time constant function
def time_constant(moment_of_inertia, atm_density, c_m_alpha, radius, vel):
    """
    Calculate the time constant of a composite shape consisting of a cylinder and a cone, defined relative to the center of mass.
    """
    ref_area = np.pi * radius**2  # Reference area (cross-sectional area of the cylinder)

    time_constant = np.sqrt(-2 * moment_of_inertia / (c_m_alpha * atm_density * ref_area * vel**2))

    return time_constant

# Define a propnav function

# Define a transfer function

# Plot the acceleration command and response

# plot the time constant as a function of velocity
# Define the velocity range
velocities = np.linspace(1, 10000, 99)  # Velocity range from 1 to 10000 m/s
time_constants_vel = np.zeros(len(velocities))
# Calculate the atmospheric density at sea level
rho = atm_density(altitude=0)
for i, vel in enumerate(velocities):
    # Calculate the moment of inertia
    Iy = moment_of_inertia(radius=0.25, length_cylinder=1.63, length_cone=1.12, density=rv_density)
    
    # Calculate the time constant
    tc = time_constant(Iy, rho, c_m_alpha=-0.15, radius=0.5, vel=vel)
    
    # Store the time constant
    time_constants_vel[i] = tc

# Plot the time constant vs velocity
plt.figure(figsize=(10, 6))
plt.plot(velocities, time_constants_vel)
plt.xscale('log')
plt.title('Time Constant vs Velocity')
plt.xlabel('Velocity (m/s)')
plt.ylabel('Time Constant (s)')
plt.grid()
plt.show()

# Plot the time constant as a function of altitude
# Define the altitude range
altitudes = np.linspace(0, 100000, 100)  # Altitude range from 0 to 20000 m
time_constants_altitude = np.zeros(len(altitudes))
# Calculate the atmospheric density at each altitude
for i, alt in enumerate(altitudes):
    # Calculate the atmospheric density
    rho = atm_density(altitude=alt)
    
    # Calculate the moment of inertia
    Iy = moment_of_inertia(radius=0.25, length_cylinder=1.63, length_cone=1.12, density=rv_density)
    
    # Calculate the time constant
    tc = time_constant(Iy, rho, c_m_alpha=-0.15, radius=0.5, vel=10000)  # Using a constant velocity of 10000 m/s
    
    # Store the time constant
    time_constants_altitude[i] = tc
# Plot the time constant vs altitude
plt.figure(figsize=(10, 6))
plt.plot(altitudes, time_constants_altitude)
plt.title('Time Constant vs Altitude')
plt.xlabel('Altitude (m)')
plt.ylabel('Time Constant (s)')
plt.grid()
plt.show()
