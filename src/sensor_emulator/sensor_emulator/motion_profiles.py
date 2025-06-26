import math, random

def linear_motion(t, speed=1.0):
    # x increases at constant speed, y stays zero
    x = speed * t
    y = 0.0
    theta = 0.0
    return x, y, theta

def zigzag_motion(t, speed=1.0, period=2.0):
    # x increases, y oscillates sine-wave
    x = speed * t
    y = math.sin(2 * math.pi * t / period) * speed
    theta = math.atan2(y, x)
    return x, y, theta

def circle_motion(t, radius=5.0, omega=0.5):
    theta = omega * t
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    heading = theta + math.pi/2
    return x, y, heading

def hover_drift(t, amp=0.5):
    # small random walk around origin
    x = random.uniform(-amp, amp)
    y = random.uniform(-amp, amp)
    theta = random.uniform(-0.1, 0.1)
    return x, y, theta

