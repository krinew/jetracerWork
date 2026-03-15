#!/usr/bin/env python3
import sys
import os
import cv2
import time 

# Ensure the submodule is in the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../jetcam'))
from jetcam.csi_camera import CSICamera

def update_image(change):
    image = change['new']
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
    cv2.imshow('CSI Camera Test', image)
    cv2.waitKey(1)
def test_csi(sensor_id,flip_method,wwidth,height,fps):
    cam = CSICamera(sensor_id=sensor_id, flip_method=flip_method, width=wwidth, height=height, fps=fps)
    print(f"Camera initialized with sensor_id={sensor_id}, flip_method={flip_method}, width={wwidth}, height={height}, fps={fps}")
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
