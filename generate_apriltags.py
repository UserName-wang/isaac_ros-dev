#!/usr/bin/env python3

import cv2
import os

def generate_apriltags():
    """Generate 5 AprilTags from the tag36h11 family with IDs 0-4"""
    
    # Create output directory
    output_dir = "apriltag_images"
    os.makedirs(output_dir, exist_ok=True)
    
    # Get the tag36h11 dictionary
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    print("Generating AprilTags...")
    
    # Generate 5 tags (IDs 0-4)
    for i in range(5):
        # Generate tag with ID i, size 200x200 pixels
        tag = cv2.aruco.generateImageMarker(dictionary, i, 200)
        
        # Add white border for better detection
        bordered_tag = cv2.copyMakeBorder(tag, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=[255, 255, 255])
        
        # Save the tag
        filename = f'apriltag_36h11_id_{i:02d}.png'
        filepath = os.path.join(output_dir, filename)
        cv2.imwrite(filepath, bordered_tag)
        print(f"Saved {filepath}")
    
    print(f"\nSuccessfully generated 5 AprilTags in the '{output_dir}' directory!")

if __name__ == "__main__":
    generate_apriltags()