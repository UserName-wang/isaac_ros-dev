#!/usr/bin/env python3

import cv2
import numpy as np
import os

def create_larger_apriltag_sheet():
    """Create a single A4 sheet with all 5 larger AprilTags"""
    
    # A4 dimensions at 300 DPI (standard print quality)
    # A4 = 210mm x 297mm = 2480 x 3508 pixels at 300 DPI
    a4_width, a4_height = 2480, 3508
    
    # Create a white A4 canvas
    a4_sheet = np.ones((a4_height, a4_width, 3), dtype=np.uint8) * 255
    
    # Load all 5 larger AprilTag images
    tag_files = [
        'apriltag_36h11_id_00_large.png',
        'apriltag_36h11_id_01_large.png',
        'apriltag_36h11_id_02_large.png',
        'apriltag_36h11_id_03_large.png',
        'apriltag_36h11_id_04_large.png'
    ]
    
    # Load and place tags on the sheet
    tags = []
    for filename in tag_files:
        filepath = os.path.join('larger_apriltag_images', filename)
        if os.path.exists(filepath):
            tag = cv2.imread(filepath)
            tags.append(tag)
            print(f"Loaded {filename}: {tag.shape[1]}x{tag.shape[0]} pixels")
        else:
            print(f"Warning: {filepath} not found")
            return
    
    # Arrange tags in a grid (2 columns, 3 rows to accommodate 5 tags)
    # Calculate spacing for the larger tags
    margin = 100
    tag_spacing_x = (a4_width - 2 * margin) // 2  # Space for 2 columns
    tag_spacing_y = (a4_height - 2 * margin) // 3  # Space for 3 rows
    
    # Position tags
    positions = [
        (margin, margin),                                    # Top-left
        (margin + tag_spacing_x, margin),                    # Top-right
        (margin, margin + tag_spacing_y),                    # Middle-left
        (margin + tag_spacing_x, margin + tag_spacing_y),    # Middle-right
        (margin + tag_spacing_x // 2, margin + 2 * tag_spacing_y)  # Bottom-center
    ]
    
    # Place tags on the A4 sheet
    for i, (tag, pos) in enumerate(zip(tags, positions)):
        tag_h, tag_w = tag.shape[:2]
        x, y = pos
        
        # Center the tag in its allocated space
        x_offset = x + (tag_spacing_x - tag_w) // 2
        y_offset = y + (tag_spacing_y - tag_h) // 2
        
        # Place the tag on the sheet
        a4_sheet[y_offset:y_offset+tag_h, x_offset:x_offset+tag_w] = tag
        
        # Add label below each tag
        label = f"ID: {i}"
        label_pos = (x_offset + tag_w//2 - 30, y_offset + tag_h + 30)
        cv2.putText(a4_sheet, label, label_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 0, 0), 2, cv2.LINE_AA)
    
    # Add title
    title = "Larger AprilTags 36h11 (ID 0-4)"
    title_size = 2
    title_thickness = 3
    title_pos = (a4_width//2 - 300, 50)
    cv2.putText(a4_sheet, title, title_pos, cv2.FONT_HERSHEY_SIMPLEX, 
               title_size, (0, 0, 0), title_thickness, cv2.LINE_AA)
    
    # Add size information
    size_info = "Each tag: 500x500px + 20px border = 45.7mm x 45.7mm at 300 DPI"
    size_info_pos = (a4_width//2 - 400, 90)
    cv2.putText(a4_sheet, size_info, size_info_pos, cv2.FONT_HERSHEY_SIMPLEX,
               0.8, (0, 0, 0), 1, cv2.LINE_AA)
    
    # Save the combined sheet
    output_path = "larger_apriltag_a4_sheet.png"
    cv2.imwrite(output_path, a4_sheet)
    print(f"A4 sheet with all larger AprilTags saved as {output_path}")
    
    # Show sheet info
    print(f"A4 sheet dimensions: {a4_width}x{a4_height} pixels (300 DPI)")
    print("These tags are more than 2x larger than the previous ones.")
    print("You can now print this file on A4 paper for a 1:1 scale print.")

if __name__ == "__main__":
    create_larger_apriltag_sheet()