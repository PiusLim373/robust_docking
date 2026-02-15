import math

x = -0.03
y = 0.0


rho = math.sqrt(x * x + y * y)
alpha = math.atan2(y, x)

print(f"rho: {rho}, alpha: {math.degrees(alpha)} degrees")

k_rho = 0.8
k_alpha = 2.0
# Nonlinear control law
v = k_rho * rho * math.cos(alpha)
omega = k_alpha * alpha


print(f"v: {v}, omega: {omega}")