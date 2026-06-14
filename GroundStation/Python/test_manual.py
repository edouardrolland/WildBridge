import os
import sys
import time
from datetime import datetime

from djiInterface import DJIInterface, LENS_KEYS

IP_RC = "10.177.40.4"

def main():

    print(f"Connecting to WildBridge bridge at {IP_RC}:8080 ...")

    drone = DJIInterface(IP_RC)

    # Save all images into a timestamped folder next to this script.
    out_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    
    os.makedirs(out_dir, exist_ok=True)

    print("Taking the picture(s) — one shutter (no download yet)...")    
    picture_info = drone.requestCapture()
    print("Capture completed.")
    
    
    print("Picture info:")
    for key, value in picture_info.items():
        print(f"  {key}: {value}")
        
    print("Downloading the captured images...")
    for lens in LENS_KEYS:
        if picture_info.get(lens):
            drone.downloadByName(picture_info[lens], out_dir=out_dir)
    
    
        
     

if __name__ == "__main__":
    main()