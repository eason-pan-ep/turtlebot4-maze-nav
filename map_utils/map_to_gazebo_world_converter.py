#!/usr/bin/env python3

import yaml
import numpy as np
from PIL import Image
import argparse
import os

def generate_gazebo_world(pgm_file, yaml_file, output_file):
    # Load YAML metadata
    with open(yaml_file, 'r') as f:
        map_data = yaml.safe_load(f)
    
    resolution = map_data['resolution']
    origin = map_data['origin']
    
    # Load PGM image
    img = Image.open(pgm_file)
    img_array = np.array(img)
    
    # Get dimensions
    height, width = img_array.shape
    
    # Calculate real-world dimensions
    real_width = width * resolution
    real_height = height * resolution
    
    # Start building the world file
    world_file = f"""<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Map walls -->
    <model name="map_walls">
      <static>true</static>
      <link name="walls_link">
"""
    
    # Process the occupancy grid to identify walls
    occupied_thresh = map_data.get('occupied_thresh', 0.65)
    free_thresh = map_data.get('free_thresh', 0.196)
    negate = map_data.get('negate', 0)
    
    # Detect walls (occupied cells)
    wall_positions = []
    
    # Threshold value depends on the PGM format (0-255 or 0-1)
    max_val = np.max(img_array)
    threshold = int(occupied_thresh * max_val) if max_val > 1 else occupied_thresh
    
    if negate:
        # If negated, white (255/high values) is occupied
        walls = img_array >= threshold
    else:
        # If not negated, black (0/low values) is occupied
        walls = img_array <= threshold
    
    # Find all wall positions
    wall_indices = np.where(walls)
    for i in range(len(wall_indices[0])):
        y, x = wall_indices[0][i], wall_indices[1][i]
        # Convert to real-world coordinates
        real_x = x * resolution + origin[0]
        real_y = (height - y) * resolution + origin[1]  # Flip y-coordinate
        wall_positions.append((real_x, real_y))
    
    # Generate wall elements
    wall_height = 2.0  # Default wall height in meters
    wall_width = resolution  # Width of each wall segment
    
    for i, (x, y) in enumerate(wall_positions):
        world_file += f"""
        <collision name="wall_collision_{i}">
          <pose>{x} {y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_width} {wall_width} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_visual_{i}">
          <pose>{x} {y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_width} {wall_width} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>"""
    
    # Close the model
    world_file += """
      </link>
    </model>
  </world>
</sdf>
"""
    
    # Write the world file
    with open(output_file, 'w') as f:
        f.write(world_file)
    
    print(f"Created Gazebo world file: {output_file}")
    print(f"Map dimensions: {width}x{height} pixels ({real_width:.2f}x{real_height:.2f} meters)")
    print(f"Map origin: {origin}")
    print(f"Total walls generated: {len(wall_positions)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert ROS map to Gazebo world')
    parser.add_argument('pgm_file', help='Path to PGM map file')
    parser.add_argument('yaml_file', help='Path to YAML map metadata file')
    parser.add_argument('--output', default='map_world.world', help='Output Gazebo world file')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.pgm_file):
        print(f"Error: PGM file {args.pgm_file} not found")
        exit(1)
    
    if not os.path.exists(args.yaml_file):
        print(f"Error: YAML file {args.yaml_file} not found")
        exit(1)
    
    generate_gazebo_world(args.pgm_file, args.yaml_file, args.output)