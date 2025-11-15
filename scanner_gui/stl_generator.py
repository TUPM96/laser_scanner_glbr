"""
STL file generator from 3D point cloud
"""

import numpy as np
from scipy.spatial import ConvexHull
from scipy.interpolate import griddata

def generate_stl_from_points(points, filename):
    """
    Generate STL file from point cloud
    
    Args:
        points: numpy array of shape (N, 3) with x, y, z coordinates
        filename: output STL filename
    """
    if len(points) < 3:
        raise ValueError("Not enough points to generate mesh")
    
    # Simple approach: create a mesh using Delaunay triangulation
    # For better results, you might want to use more sophisticated methods
    
    # Project points to 2D for triangulation (using theta and z)
    # Convert back to cartesian for STL
    
    # For now, use a simple convex hull approach
    try:
        hull = ConvexHull(points)
        write_stl_binary(filename, points, hull.simplices)
    except:
        # Fallback: create a simple mesh grid
        create_grid_mesh(points, filename)

def create_grid_mesh(points, filename):
    """Create a mesh from points using grid interpolation"""
    # Get bounds
    x_min, x_max = points[:, 0].min(), points[:, 0].max()
    y_min, y_max = points[:, 1].min(), points[:, 1].max()
    z_min, z_max = points[:, 2].min(), points[:, 2].max()
    
    # Create grid
    x_res = 50
    y_res = 50
    
    xi = np.linspace(x_min, x_max, x_res)
    yi = np.linspace(y_min, y_max, y_res)
    xi_grid, yi_grid = np.meshgrid(xi, yi)
    
    # Interpolate z values
    zi_grid = griddata(
        (points[:, 0], points[:, 1]),
        points[:, 2],
        (xi_grid, yi_grid),
        method='linear',
        fill_value=z_min
    )
    
    # Create mesh from grid
    vertices = []
    faces = []
    
    for i in range(y_res - 1):
        for j in range(x_res - 1):
            # Create two triangles for each quad
            v1 = [xi_grid[i, j], yi_grid[i, j], zi_grid[i, j]]
            v2 = [xi_grid[i, j+1], yi_grid[i, j+1], zi_grid[i, j+1]]
            v3 = [xi_grid[i+1, j], yi_grid[i+1, j], zi_grid[i+1, j]]
            v4 = [xi_grid[i+1, j+1], yi_grid[i+1, j+1], zi_grid[i+1, j+1]]
            
            # Triangle 1
            idx1 = len(vertices)
            vertices.append(v1)
            idx2 = len(vertices)
            vertices.append(v2)
            idx3 = len(vertices)
            vertices.append(v3)
            faces.append([idx1, idx2, idx3])
            
            # Triangle 2
            idx4 = len(vertices)
            vertices.append(v2)
            idx5 = len(vertices)
            vertices.append(v4)
            idx6 = len(vertices)
            vertices.append(v3)
            faces.append([idx4, idx5, idx6])
    
    vertices = np.array(vertices)
    write_stl_binary(filename, vertices, np.array(faces))

def write_stl_binary(filename, vertices, faces):
    """Write STL file in binary format"""
    with open(filename, 'wb') as f:
        # Write header (80 bytes)
        header = b'3D Scanner STL File' + b'\x00' * 61
        f.write(header)
        
        # Write number of facets
        num_facets = len(faces)
        f.write(np.uint32(num_facets).tobytes())
        
        # Write facets
        for face in faces:
            # Get triangle vertices
            v1 = vertices[face[0]]
            v2 = vertices[face[1]]
            v3 = vertices[face[2]]
            
            # Calculate normal
            normal = calculate_normal(v1, v2, v3)
            
            # Write normal (3 floats)
            f.write(np.float32(normal).tobytes())
            
            # Write vertices (3 floats each)
            f.write(np.float32(v1).tobytes())
            f.write(np.float32(v2).tobytes())
            f.write(np.float32(v3).tobytes())
            
            # Write attribute byte count (usually 0)
            f.write(np.uint16(0).tobytes())

def calculate_normal(v1, v2, v3):
    """Calculate normal vector for triangle"""
    edge1 = np.array(v2) - np.array(v1)
    edge2 = np.array(v3) - np.array(v1)
    normal = np.cross(edge1, edge2)
    norm = np.linalg.norm(normal)
    if norm > 0:
        normal = normal / norm
    return normal

