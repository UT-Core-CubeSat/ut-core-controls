import numpy as np
from utu_flight import StarTracker as st
from stt_flight_param import Param
import time
import math
import pandas as pd

# instantiate star tracker class
starTracker = st(Param)
success = 0
yaw = [0,60,120,180]
results = []
flag = "fail"

# solve lost in space problem for a subset of UTU images
for psi in yaw:
    for ISO in range(1,9):
       for exposure in range(1,21):
           starTracker.alpha =  starTracker.delta = starTracker.phi = np.nan
           flag = "fail"
           photoid = f"{psi}_45_{100*exposure}_{ISO}"
           starTracker.pull_photo(photoid)
           try: 
               starTracker.solve_LIS()
           except Exception as err:
               print(f"----> {err}")
           if not math.isnan(starTracker.alpha):
               success += 1    
               flag = "pass"
               
           results.append({
                'yaw': psi,
                'pitch': 45,
                'ISO': ISO,
                'exposure': exposure,
                'RA': starTracker.alpha,
                'DEC': starTracker.delta,
                'ROLL': starTracker.phi,
                'flag': flag
            })
                    
starTracker.close_tracker()
print(f"The star tracker succeeded {success} times")

# save results to a csv for later analysis...
df = pd.DataFrame(results)
csv_dir = f"{starTracker.DIR_SCRIPT}/utu_img/sweep_test_results.csv"
df.to_csv(csv_dir, index=False)    
print(f"The results of this grid search have been saved to:\n {csv_dir}")