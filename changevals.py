import numpy as np
from scipy.ndimage import gaussian_filter

def simulate_martian_surface(inputFile, outputFile):
    # 1. Load your original structured data
    # This preserves the exact X and Y coordinates your code expects
    data = np.loadtxt(inputFile)
    
    # 2. Identify dimensions
    x_coords = np.unique(data[:, 0])
    y_coords = np.unique(data[:, 1])
    dimY, dimX = len(y_coords), len(x_coords)
    
    # 3. Generate "Martian" Terrain
    # Start with pure white noise
    np.random.seed(37) # For reproducibility
    noise = np.random.normal(0, 1, (dimY, dimX))
    
    # Apply a Gaussian filter to create smooth "hills" and "craters"
    # A sigma of 2rd to 5th of the grid size usually looks like planetary terrain
    smooth_terrain = gaussian_filter(noise, sigma=2.0)
    
    # Scale it to Martian-like elevation ranges (e.g., -4000m to 2000m)
    min_elev, max_elev = -4000, 50
    smooth_terrain = (smooth_terrain - smooth_terrain.min()) / (smooth_terrain.max() - smooth_terrain.min())
    simulated_z = smooth_terrain * (max_elev - min_elev) + min_elev
    
    # 4. Reconstruct the XYZ format
    # We flatten the 2D simulated grid back into the 1D column
    data[:, 2] = simulated_z.flatten()
    
    # 5. Save as elevationdata_v2.txt
    # fmt='%.4f' ensures it looks like your original Scripps format
    np.savetxt(outputFile, data, fmt=['%.4f', '%.4f', '%.2f'])
    
    print(f"Successfully simulated Martian surface in {outputFile}")
    print(f"Dimensions: {dimX}x{dimY} ({len(data)} points)")

# Run the transformation
simulate_martian_surface("elevationdata.txt", "elevationdata_v2.txt")