import numpy as np
import pandas as pd
import os
import time
import subprocess as sp


# prompt user for angles
yaw = float(input("enter yaw angle in degrees:"))
pitch = float(input("enter pitch angle in degrees:"))
exposures = np.linspace(100, 2000, 20)
photo_ISOs = np.linspace(1,8,8)
results = []

# path to photo directory
archive_dir = "/home/pi/Star_Tracker/RPi/utu_img"

# Ensure the directory exists
if not os.path.exists(archive_dir):
    os.makedirs(archive_dir)
    print(f"Created missing directory: {archive_dir}")

# sweep through exposures
i = 1
for photo_ISO in photo_ISOs:
    for exposure in exposures: 
        # time data
        startTime = time.perf_counter()
        utc_timestamp = time.time()

        # take picture
        photo_dir = f"{archive_dir}/img_{int(yaw)}_{int(pitch)}_{int(exposure)}_{int(photo_ISO)}.jpg"
        task = f"rpicam-still -o {photo_dir} -t 1 --width 1024 --height 1024 --shutter {exposure*1000} --gain {photo_ISO} --awbgains 1.0,1.0 2>/dev/null"
        #2>/dev/null
        try:
            sp.run(task, shell=True, check=True)
        except sp.CalledProcessError:
            print(f"---> ERROR: Failed to capture image {i}")
            i += 1
            continue # Skip to next exposure if one fails

        # calculate elapsed time
        endTime = time.perf_counter()
        captureTime = endTime - startTime

        # save results of current photo to table
        results.append({
            'photo': i,
            'exposure_time_ms': exposure,
            'ISO': photo_ISO,
            'yaw': yaw,
            'pitch': pitch,
            'capture_time': captureTime,
            'utc_time': utc_timestamp,
            })

        # iterate i
        i += 1
    
# save results to csv or append if the file already exists
csv_dir = f"{archive_dir}/archive.csv"
if os.path.exists(csv_dir):
    df1 = pd.read_csv(csv_dir)
    df2 = pd.DataFrame(results)
    df = pd.concat([df1,df2])
else:
    df = pd.DataFrame(results)
    
df.to_csv(csv_dir, index=False)    
print(f"{len(results)} photos have been captured and archived to:\n {csv_dir}")
