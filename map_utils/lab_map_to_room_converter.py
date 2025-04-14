#!/usr/bin/env python3

import yaml
import numpy as np
from PIL import Image
import argparse
import os
import cv2

def create_simple_room_gazebo_world(pgm_file, yaml_file, output_file):
    """
    Create a minimalist Gazebo world from a map that is primarily an empty room.
    This approach uses boundary detection to create just the outer walls.
    """
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
    
    # Process the occupancy grid
    occupied_thresh = map_data.get('occupied_thresh', 0.65)
    negate = map_data.get('negate', 0)
    
    # Threshold value depends on the PGM format (0-255 or 0-1)
    max_val = np.max(img_array)
    threshold = int(occupied_thresh * max_val) if max_val > 1 else occupied_thresh
    
    if negate:
        # If negated, white (255/high values) is occupied
        walls_binary = (img_array >= threshold).astype(np.uint8) * 255
    else:
        # If not negated, black (0/low values) is occupied
        walls_binary = (img_array <= threshold).astype(np.uint8) * 255
    
    # Apply some morphological operations to clean up the map
    kernel = np.ones((5, 5), np.uint8)
    walls_cleaned = cv2.morphologyEx(walls_binary, cv2.MORPH_CLOSE, kernel)
    
    # Find contours of walls
    contours, _ = cv2.findContours(walls_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # For an empty room, we primarily care about the outer boundary
    # Sort contours by area to find the main room boundary
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
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
    
    <!-- Room from map -->
    <model name="room_model">
      <static>true</static>
      <link name="walls_link">
"""
    
    # Add four simple walls based on the room dimensions
    # This assumes a rectangular room for maximum simplicity
    wall_height = 2.5  # Default wall height in meters
    wall_thickness = 0.2  # Wall thickness in meters
    
    # If we found contours, use the largest one (main room boundary)
    if contours:
        main_contour = contours[0]
        
        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(main_contour)
        
        # Convert to real-world coordinates
        real_x = x * resolution + origin[0]
        real_y = (height - y - h) * resolution + origin[1]  # Flip y-coordinate
        real_width = w * resolution
        real_height = h * resolution
        
        # Calculate wall positions
        # North wall (top)
        north_x = real_x + real_width/2
        north_y = real_y + real_height
        
        # South wall (bottom)
        south_x = real_x + real_width/2
        south_y = real_y
        
        # East wall (right)
        east_x = real_x + real_width
        east_y = real_y + real_height/2
        
        # West wall (left)
        west_x = real_x
        west_y = real_y + real_height/2
        
        # Add north wall
        world_file += f"""
        <collision name="north_wall_collision">
          <pose>{north_x} {north_y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{real_width + wall_thickness} {wall_thickness} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="north_wall_visual">
          <pose>{north_x} {north_y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{real_width + wall_thickness} {wall_thickness} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/White</n>
            </script>
          </material>
        </visual>
        
        <!-- South wall -->
        <collision name="south_wall_collision">
          <pose>{south_x} {south_y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{real_width + wall_thickness} {wall_thickness} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="south_wall_visual">
          <pose>{south_x} {south_y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{real_width + wall_thickness} {wall_thickness} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/White</n>
            </script>
          </material>
        </visual>
        
        <!-- East wall -->
        <collision name="east_wall_collision">
          <pose>{east_x} {east_y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {real_height} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="east_wall_visual">
          <pose>{east_x} {east_y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {real_height} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/White</n>
            </script>
          </material>
        </visual>
        
        <!-- West wall -->
        <collision name="west_wall_collision">
          <pose>{west_x} {west_y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {real_height} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="west_wall_visual">
          <pose>{west_x} {west_y} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {real_height} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/White</n>
            </script>
          </material>
        </visual>"""
        
        # Look for major internal obstacles (if any)
        if len(contours) > 1:
            # Analyze a few of the largest internal obstacles
            for i, contour in enumerate(contours[1:4]):  # Consider up to 3 internal obstacles
                area = cv2.contourArea(contour)
                if area > 200:  # Only consider substantial obstacles
                    # Get the center point of the obstacle
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Get approximate size using minimum area rectangle
                        rect = cv2.minAreaRect(contour)
                        # This line was causing the error, let's simplify it
                        # No need to convert box points to int0
                        # Just use the rect dimensions directly
                        rect_width, rect_height = rect[1]
                        
                        # Convert to real world
                        real_cx = cx * resolution + origin[0]
                        real_cy = (height - cy) * resolution + origin[1]
                        real_rect_width = rect_width * resolution
                        real_rect_height = rect_height * resolution
                        angle = rect[2]
                        
                        world_file += f"""
        <!-- Obstacle {i+1} -->
        <collision name="obstacle_{i+1}_collision">
          <pose>{real_cx} {real_cy} {wall_height/2} 0 0 {angle * 3.14159/180}</pose>
          <geometry>
            <box>
              <size>{real_rect_width} {real_rect_height} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="obstacle_{i+1}_visual">
          <pose>{real_cx} {real_cy} {wall_height/2} 0 0 {angle * 3.14159/180}</pose>
          <geometry>
            <box>
              <size>{real_rect_width} {real_rect_height} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/Grey</n>
            </script>
          </material>
        </visual>"""
    else:
        # No contours found, create a default square room
        default_room_size = 10.0  # 10 meters square
        
        world_file += f"""
        <!-- Default room since no walls were detected -->
        <!-- North wall -->
        <collision name="north_wall_collision">
          <pose>0 {default_room_size/2} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{default_room_size} {wall_thickness} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="north_wall_visual">
          <pose>0 {default_room_size/2} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{default_room_size} {wall_thickness} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/White</n>
            </script>
          </material>
        </visual>
        
        <!-- South wall -->
        <collision name="south_wall_collision">
          <pose>0 {-default_room_size/2} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{default_room_size} {wall_thickness} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="south_wall_visual">
          <pose>0 {-default_room_size/2} {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{default_room_size} {wall_thickness} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/White</n>
            </script>
          </material>
        </visual>
        
        <!-- East wall -->
        <collision name="east_wall_collision">
          <pose>{default_room_size/2} 0 {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {default_room_size} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="east_wall_visual">
          <pose>{default_room_size/2} 0 {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {default_room_size} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/White</n>
            </script>
          </material>
        </visual>
        
        <!-- West wall -->
        <collision name="west_wall_collision">
          <pose>{-default_room_size/2} 0 {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {default_room_size} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="west_wall_visual">
          <pose>{-default_room_size/2} 0 {wall_height/2} 0 0 0</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {default_room_size} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <n>Gazebo/White</n>
            </script>
          </material>
        </visual>"""
    
    # Close the model and world
    world_file += """
      </link>
    </model>
  </world>
</sdf>
"""
    
    # Write the world file
    with open(output_file, 'w') as f:
        f.write(world_file)
    
    if contours:
        main_contour = contours[0]
        x, y, w, h = cv2.boundingRect(main_contour)
        real_width = w * resolution
        real_height = h * resolution
        print(f"Created simplified room Gazebo world: {output_file}")
        print(f"Room dimensions: {real_width:.2f} x {real_height:.2f} meters")
        print(f"Total number of meshes: 8 (4 walls + optional internal obstacles)")
    else:
        print(f"Created default room Gazebo world: {output_file}")
        print(f"Room dimensions: 10 x 10 meters (default)")
        print(f"Total number of meshes: 8 (4 walls)")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert a simple room map to minimal Gazebo world')
    parser.add_argument('pgm_file', help='Path to PGM map file')
    parser.add_argument('yaml_file', help='Path to YAML map metadata file')
    parser.add_argument('--output', default='simple_room.world', help='Output Gazebo world file')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.pgm_file):
        print(f"Error: PGM file {args.pgm_file} not found")
        exit(1)
    
    if not os.path.exists(args.yaml_file):
        print(f"Error: YAML file {args.yaml_file} not found")
        exit(1)
    
    create_simple_room_gazebo_world(args.pgm_file, args.yaml_file, args.output)