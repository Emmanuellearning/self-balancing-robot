#!/usr/bin/env python3
"""
Simple script to calculate basic inverted pendulum properties and suggest PID gains.
"""

import math

def main():
    print("=== Self-Balancing Robot PID Calculation ===")
    
    # Robot parameters
    m_body = 2.0    # mass of body (kg)
    m_wheel = 0.5   # mass of wheels (kg)
    l = 0.2         # distance to center of mass (m)
    g = 9.81        # gravity (m/s^2)
    
    # Moment of inertia for body (box: height 0.4, depth 0.1)
    I = (1.0 / 12.0) * m_body * (0.1**2 + 0.4**2)
    
    # Natural frequency (omega_n)
    omega_n = math.sqrt((m_body * g * l) / (I + m_body * (l**2)))
    
    print(f"Body Mass: {m_body} kg")
    print(f"Length (COM): {l} m")
    print(f"Natural Frequency: {omega_n:.2f} rad/s")
    print("\nChosen PID Gains:")
    print("  Kp = 22.0  (restoring force)")
    print("  Ki = 0.4   (steady-state error correction)")
    print("  Kd = 2.5   (damping against oscillations)")
    print("  Kp_pos = 0.15 (position holding)")
    print("  Kd_pos = 0.30 (velocity damping)")
    print("============================================")

if __name__ == '__main__':
    main()
