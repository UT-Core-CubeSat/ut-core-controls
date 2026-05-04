# define parameters for flight code
import numpy as np
import re
import os

Param = {}

# camera parameters
Param['exp_time'] = 800
Param['cat_div']  = 5
Param['NUM_PIX']  = 1024
Param['ISO']      = 4

# initialization of output values
Param['alpha'] = np.nan
Param['delta'] = np.nan
Param['roll'] = np.nan
Param['captureTime'] = np.nan
Param['utc_timestamp'] = np.nan
Param['flag'] = "fail"

# NAMES
Param['SEXTRACTOR_STR'] = "source-extractor"

# DIRECTORIES
Param['DIR_STARS'] = "sext"
Param['DIR_SCRIPT'] = os.getcwd()
Param['DIR_PROJ_CAT_RPI']      = f"{Param['DIR_SCRIPT']}/Catalog/RPi/Projected"
Param['DIR_PROJ_CAT_STEREO']   = f"{Param['DIR_SCRIPT']}/Catalog/STEREO/Projected"
Param['DIR_NORMAL_CAT_RPI']    = f"{Param['DIR_SCRIPT']}/Catalog/RPi/Normal"
Param['DIR_NORMAL_CAT_STEREO'] = f"{Param['DIR_SCRIPT']}/Catalog/STEREO/Normal"
Param['NEW_PROJ_CAT']          = "new_cat"
Param['DIR_IMG']               = f"{Param['DIR_SCRIPT']}/stt_data/stt_img.jpg"
Param['DIR_FITS']              = f"{Param['DIR_SCRIPT']}/stt_data/img.fits"
Param['DIR_STT_DATA']          = f"{Param['DIR_SCRIPT']}/stt_data"

# ALGORITHM PARAMETERS
Param['SEXTRACTOR_MAX_STARS'] = 40
Param['X_PIX']                = 512
Param['Y_PIX']                = 512
Param['PIX2MM_RPI']           = 0.002695 # v2.1

#PIX2MM_RPI = 0.003545 # v3 wide
Param['PIX2MM_STEREO']    = 0.027021
Param['LIS_MAX_ITER']     = 3
Param['FOCAL_LEN_MM_RPI'] = 3.04 # v2.1

#FOCAL_LEN_MM_RPI = 2.75 # v3 wide
Param['FOCAL_LEN_MM_STEREO'] = 78.46

# MATCH PARAMETERS
Param['PARAM1']  = "trirad=0.002 nobj=15 max_iter=1 matchrad=0.1 scale=1"
Param['PARAM2']  = "trirad=0.002 nobj=20 max_iter=5 matchrad=0.01 scale=1"

# REGULAR EXPRESSIONS
Param['MATCH_STD']  = re.compile(r"sig=(-*\d\.\d+e...) Nr=(-*\d+) Nm=(-*\d+) sx=(-*\d\.\d+e...) sy=(-*\d\.\d+e...)")
Param['MATCH_NUMBERS']  = re.compile(r"a=(-*\d\.\d+e...) b=(-*\d\.\d+e...) c=(-*\d\.\d+e...) "
                           r"d=(-*\d\.\d+e...) e=(-*\d\.\d+e...) f=(-*\d\.\d+e...)")