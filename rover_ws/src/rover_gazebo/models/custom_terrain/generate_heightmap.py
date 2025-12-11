import numpy as np
from PIL import Image
import os

# Create output directory if it doesn't exist
os.makedirs('materials/textures', exist_ok=True)

# Create a 129x129 heightmap (must be 2^n + 1)
size = 513
hills_percent = 30
hills_heightmax = 50
heightmap = np.zeros((size, size))

# Generate some random hills
for _ in range(int(size/hills_heightmax * hills_percent)):
    x = np.random.randint(0, size)
    y = np.random.randint(0, size)
    radius = np.random.randint(10, 30)
    height = np.random.uniform(0.1*hills_heightmax, hills_heightmax)
    
    X, Y = np.ogrid[:size, :size]
    dist = np.sqrt((X - x)**2 + (Y - y)**2)
    heightmap += height * np.exp(-dist**2/(2*radius**2))

# Normalize and convert to grayscale image
heightmap = (heightmap * 255 / heightmap.max()).astype(np.uint8)
img = Image.fromarray(heightmap)

# Save the image
output_path = 'materials/textures/heightmap.png'
img.save(output_path)
print(f"Heightmap saved to: {os.path.abspath(output_path)}")