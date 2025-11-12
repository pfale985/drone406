#!/usr/bin/env python3
"""
AprilTag Pose Estimation Test Script
Minimal version for performance testing
"""

import time
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector
import cv2

# ============= CONFIGURATION =============

# Camera parameters (MUST BE CALIBRATED for accurate pose estimation)
CAMERA_PARAMS = {
    'fx': 1000.0,  # Focal length x (pixels)
    'fy': 1000.0,  # Focal length y (pixels)
    'cx': 320.0,   # Principal point x (pixels)
    'cy': 240.0,   # Principal point y (pixels)
}

# AprilTag parameters
TAG_SIZE = 0.16  # Tag size in meters (measure your actual tag!)
TAG_FAMILY = 'tag36h11'

# Camera resolution
RESOLUTION = (640, 480)

# Detection parameters
DETECTOR_PARAMS = {
    'families': TAG_FAMILY,
    'nthreads': 4,           # 4 cores on Pi 3B/Zero 2W
    'quad_decimate': 1.5,    # Reduce resolution for faster detection
    'quad_sigma': 0.0,       # Blur (0 = no blur)
    'decode_sharpening': 0.25,
    'refine_edges': 1,
}

# ============= MAIN PROGRAM =============

def main():
    print("Initializing AprilTag detector...")
    print(f"Resolution: {RESOLUTION[0]}x{RESOLUTION[1]}")
    print(f"Tag Size: {TAG_SIZE}m, Family: {TAG_FAMILY}\n")
    
    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": RESOLUTION, "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # Let camera warm up
    
    # Initialize detector
    detector = Detector(**DETECTOR_PARAMS)
    
    # Prepare camera parameters for pose estimation
    camera_params = [
        CAMERA_PARAMS['fx'],
        CAMERA_PARAMS['fy'],
        CAMERA_PARAMS['cx'],
        CAMERA_PARAMS['cy']
    ]
    
    print("Starting detection... (Ctrl+C to stop)\n")
    
    frame_count = 0
    total_time = 0
    detection_count = 0
    
    try:
        while True:
            loop_start = time.time()
            
            # Capture and convert to grayscale
            frame = picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            
            # Detect tags with pose estimation
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=TAG_SIZE
            )
            
            loop_time = time.time() - loop_start
            frame_count += 1
            total_time += loop_time
            
            num_detected = len(detections)
            fps = 1.0 / loop_time if loop_time > 0 else 0
            
            if num_detected > 0:
                detection_count += 1
                det = detections[0]
                tvec = det.pose_t.flatten()
                distance = np.linalg.norm(tvec)
                
                print(f"Frame {frame_count} | Tags: {num_detected} | ID: {det.tag_id} | "
                      f"Pos: [{tvec[0]:+.3f}, {tvec[1]:+.3f}, {tvec[2]:+.3f}]m | "
                      f"Dist: {distance:.3f}m | {loop_time*1000:.1f}ms | {fps:.1f}Hz")
            else:
                print(f"Frame {frame_count} | Tags: 0 | {loop_time*1000:.1f}ms | {fps:.1f}Hz")
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        # Print statistics
        avg_time = total_time / frame_count if frame_count > 0 else 0
        avg_fps = 1.0 / avg_time if avg_time > 0 else 0
        detection_rate = (detection_count / frame_count * 100) if frame_count > 0 else 0
        
        print(f"\n{'='*50}")
        print("STATISTICS")
        print(f"{'='*50}")
        print(f"Total Frames: {frame_count}")
        print(f"Detections: {detection_count} ({detection_rate:.1f}%)")
        print(f"Avg Time: {avg_time*1000:.1f}ms | Avg Freq: {avg_fps:.1f}Hz")
        print(f"{'='*50}\n")
        
        picam2.stop()

if __name__ == "__main__":
    main()