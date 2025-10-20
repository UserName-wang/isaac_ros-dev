#!/usr/bin/env python3

import cv2
import os

def generate_larger_apriltags():
    """Generate larger AprilTags from the tag36h11 family with IDs 0-4"""
    
    # Create output directory
    output_dir = "larger_apriltag_images"
    os.makedirs(output_dir, exist_ok=True)
    
    # Get the tag36h11 dictionary
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    print("Generating larger AprilTags...")
    
    # Generate 5 tags (IDs 0-4) with larger size (500x500 pixels)
    tag_size = 500
    
    for i in range(5):
        # Generate tag with ID i, larger size
        tag = cv2.aruco.generateImageMarker(dictionary, i, tag_size)
        
        # Add white border for better detection (20 pixels border)
        border_size = 20
        bordered_tag = cv2.copyMakeBorder(
            tag, border_size, border_size, border_size, border_size, 
            cv2.BORDER_CONSTANT, value=[255, 255, 255])
        
        # Save the tag
        filename = f'apriltag_36h11_id_{i:02d}_large.png'
        filepath = os.path.join(output_dir, filename)
        cv2.imwrite(filepath, bordered_tag)
        print(f"Saved {filepath}")
        
        # Calculate physical size at 300 DPI
        total_size = tag_size + 2 * border_size
        physical_size_mm = total_size * 25.4 / 300  # Convert pixels to mm at 300 DPI
        print(f"  Tag size: {total_size}x{total_size} pixels ({physical_size_mm:.1f}mm x {physical_size_mm:.1f}mm at 300 DPI)")
    
    print(f"\nSuccessfully generated 5 larger AprilTags in the '{output_dir}' directory!")
    print("These tags are more than 2x larger than the previous ones.")

if __name__ == "__main__":
    generate_larger_apriltags()