import subprocess
import os

# If you have the AprilTag C library built with generation tools
def generate_with_apriltag_tool(tag_family, start_id, end_id):
    """Generate AprilTags using the AprilTag command-line tool"""
    for i in range(start_id, end_id + 1):
        output_file = f'apriltag_{tag_family}_{i:02d}.png'
        cmd = [
            '/workspaces/isaac_ros-dev/apriltag/build/opencv_demo',  # Actual path to the tool
            '-f', tag_family,
            '-i', str(i),
            '-s', '200',
            '-o', output_file
        ]
        
        try:
            subprocess.run(cmd, check=True)
            print(f"Generated {output_file}")
        except FileNotFoundError:
            print("AprilTag generation tool not found. Please install the AprilTag C library.")
            break
        except subprocess.CalledProcessError:
            print(f"Failed to generate tag ID {i}")

# Try to generate tags using the tool
generate_with_apriltag_tool('tag36h11', 0, 4)