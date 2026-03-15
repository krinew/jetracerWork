#!/usr/bin/env python3
import sys
import os
import cv2
import time 

# Ensure the submodule is in the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../jetcam'))
from jetcam.usb_camera import USBCamera

def update_image(change):
    image = change['new']
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
    cv2.imshow('CSI Camera Test', image)
    cv2.waitKey(1)
def test_csi(sensor_id,flip_method,wwidth,height,fps):
    # Fallback to V4L2 (USBCamera) to bypass broken Jetson Docker NV-Argus bindings
    cam = USBCamera(capture_device=sensor_id, width=wwidth, height=height)
    print(f"V4L2 Camera initialized with device /dev/video{sensor_id}, width={wwidth}, height={height}")
    print("Press Ctrl+C to exit")
    try:
        
        start_time = time.time()
        end_time = start_time+5
        cam.running = True
        cam.observe(update_image, names='value')
        while(time.time()<end_time):
            print("I think the camera is running fine")
            time.sleep(0.1)
        print("Stopping the Cmeera")
        
    except KeyboardInterrupt as e:
        print(f"Error: {e}") 
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cam.running = False
        cam.unobserve(update_image, names='value')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # Add a call to the function so the script actually runs
    test_csi(0, 0, 640, 480, 20)
