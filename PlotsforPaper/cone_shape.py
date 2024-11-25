import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
plt.rcParams['text.usetex'] = True

# Parameters
x0, y0, z0 = 0, 0, 0  # Starting point
t_values = np.linspace(0, 10, 100)  # Range of t
r_max = 0.21  # Maximum radial distance controlling initial divergence
outward_factor = 0.04  # Factor to simulate outward flow as trajectories progress
num_trajectories = 60  # Number of diverging trajectories
curve_factor = 0.1  # Controls the curvature of the trajectories
num_samples = 10  # Number of points to sample along each trajectory

# Generate multiple curved diverging trajectories within a filled cone
def generate_trajectory(t, radial_offset, angle_offset):
    x = x0 + t
    y = y0 + (radial_offset * t + outward_factor * t**2) * np.cos(angle_offset) + curve_factor * np.sin(t)
    z = z0 + (radial_offset * t + outward_factor * t**2) * np.sin(angle_offset) + curve_factor * np.cos(t)
    return x, y, z

# Generate random radial and angular offsets for each trajectory
radial_offsets = np.random.uniform(0, r_max, num_trajectories)  # Random radial distances
angles = np.random.uniform(0, 2 * np.pi, num_trajectories)  # Random angles

# Generate trajectories
trajectories = [generate_trajectory(t_values, radial_offsets[i], angles[i]) for i in range(num_trajectories)]

# Plotting the trajectories
fig = plt.figure(figsize=(5, 3))
ax = fig.add_subplot(111, projection='3d')

# Create a colormap for the sampled points (using 'viridis' for the heatmap colors)
colormap = cm.get_cmap('jet')

# Plot each trajectory
for i, (x, y, z) in enumerate(trajectories):
    # Plot the trajectory line
    ax.plot(x, y, z, color='black', lw=1, alpha=0.7)
    
    # Sample points along the trajectory (10 equally spaced samples)
    sampled_idx = np.linspace(0, len(t_values) - 1, num_samples, dtype=int)
    sampled_x = x[sampled_idx]
    sampled_y = y[sampled_idx]
    sampled_z = z[sampled_idx]

    # Calculate distance of each sample point from the origin
    distances = np.sqrt((sampled_x - x0)**2 + (sampled_y - y0)**2 + (sampled_z - z0)**2)
    normalized_distances = distances / np.max(distances)  # Normalize distances to [0, 1]

    # Plot sampled points with minimized edge color alpha and lower edge transparency
    for j in range(len(sampled_x)):
        ax.scatter(sampled_x[j], sampled_y[j], sampled_z[j], 
                color='black',#colormap(normalized_distances[j]), 
                s=10, edgecolor='none',#(0, 0, 0, 1.0),  # Black edges with 30% opacity
                alpha=0.9,  # 90% opacity for the face color
                zorder=5)


# Plot the starting point
ax.scatter(x0, y0, z0, color='black', s=50, label='Starting Point', zorder=6)

# Labels and title
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
# ax.set_zlabel('Z [m]')
# ax.set_title('Cone-Shaped 3D Trajectories')
ax.grid(True)

# Set a colorbar for the distance-based heatmap effect
mappable = plt.cm.ScalarMappable(cmap=colormap)
mappable.set_array(np.linspace(0, np.max(distances), num_samples))
cbar = plt.colorbar(mappable, ax=ax, shrink=0.6)
cbar.set_label('Energy Consumption ($E^i_t$)')

# Set closer axis limits for better visualization
ax.set_xlim(-1, 15)
ax.set_ylim(-5, 5)
ax.set_zlim(-10, 10)
# Remove all axis elements: spines, ticks, labels
ax.axis('off')

# Set the background to be transparent (optional)
fig.patch.set_alpha(0)
# Save the plot with a transparent background
plt.savefig("cone_trajectories_heatmap.png", transparent=True, bbox_inches='tight')

# Show the plot
plt.tight_layout()
plt.show()
