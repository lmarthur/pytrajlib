# This script contains a function to estimate the torque on a reentry vehicle and the corresponding force on a control surface. 

import numpy as np

def rv_torque(C_M_alpha, alpha, velocity, altitude, char_length=2.75, char_area=0.24):
    """
    Function to estimate the torque on a reentry vehicle and the corresponding force on a control surface.

    Parameters:
    ----------
    C_M_alpha : float
        Moment coefficient with respect to angle of attack (alpha).
    alpha : float
        Angle of attack in radians.
    velocity : float
        Velocity of the reentry vehicle in m/s.
    altitude : float    
        Altitude of the reentry vehicle in meters.
    char_length : float, optional
        Characteristic length of the vehicle (default is 2.75 meters).
    char_area : float, optional
        Characteristic area of the vehicle (default is 0.24 square meters).
    """

    # Calculate the dynamic pressure (q)
    density = 1.225 * np.exp(-altitude / 8000)  # Simplified atmospheric density model (kg/m^3)
    q = 0.5 * density * velocity**2  # Dynamic pressure

    torque = C_M_alpha * q * char_area * char_length * alpha

    return torque

print(rv_torque(-0.15, np.radians(5), 7500, 1000))
print(rv_torque(-0.15, np.radians(5), 7500, 10000))
print(rv_torque(-0.15, np.radians(5), 7500, 50000))