# Simulation parameters
t_final = 50.0
dt = 0.1
times = 0:dt:t_final

# Initial conditions
x0, y0 = 0.0, 0.0
vx0, vy0 = 1.0, 2.0
ax, ay = 0.2, -0.1

# Compute the path
positions = [(x0 + vx0*t + 0.5*ax*t^2,
              y0 + vy0*t + 0.5*ay*t^2) 
             for t in times]

using Random
Random.seed!(123)
noise_level = 0.1
positions_noisy = [(x + randn()*noise_level, y + randn()*noise_level)
                for (x, y) in positions]

# for (i, (x, y)) in enumerate(positions_noisy)
#     println("t=$(times[i]) -> x=$x, y=$y")
# end
using DataFrames, CSV
df = DataFrame(t=times, measurement_x=[x for (x, y) in positions_noisy], measurement_y=[y for (x, y) in positions_noisy], acutal_x=[x for (x, y) in positions], acutal_y=[y for (x, y) in positions])
CSV.write("data.csv", df)