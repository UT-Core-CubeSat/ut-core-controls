import multiprocessing
import numpy as np
import os
import platform
import re
import subprocess as sp
import time
import math
from astropy.io import fits, ascii
from astropy.table import Table
from functools import partial
from PIL import Image

# TODO write a failsafe for each method. if it crashes catch the error and notify system. 

class StarTracker:
    """ A class intended to execute star tracker commands and logic"""
    
    def __init__(self, Param):
        # NAMES
        self.SEXTRACTOR_STR = Param['SEXTRACTOR_STR']
        
        # DIRECTORIES
        self.DIR_STARS             = Param['DIR_STARS']
        self.DIR_SCRIPT            = Param['DIR_SCRIPT']
        self.DIR_PROJ_CAT_RPI      = Param['DIR_PROJ_CAT_RPI']
        self.DIR_PROJ_CAT_STEREO   = Param['DIR_PROJ_CAT_STEREO']
        self.DIR_NORMAL_CAT_RPI    = Param['DIR_NORMAL_CAT_RPI']
        self.DIR_NORMAL_CAT_STEREO = Param['DIR_NORMAL_CAT_STEREO']
        self.NEW_PROJ_CAT          = Param['NEW_PROJ_CAT']
        self.DIR_IMG               = Param['DIR_IMG']
        self.DIR_FITS              = Param['DIR_FITS']
        self.DIR_STT_DATA          = Param['DIR_STT_DATA']
        
        # ALGORITHM PARAMETERS
        self.SEXTRACTOR_MAX_STARS = Param['SEXTRACTOR_MAX_STARS']
        self.X_PIX                = Param['X_PIX']
        self.Y_PIX                = Param['Y_PIX']
        self.PIX2MM_RPI           = Param['PIX2MM_RPI']
        self.PIX2MM_STEREO        = Param['PIX2MM_STEREO']
        self.LIS_MAX_ITER         = Param['LIS_MAX_ITER']
        self.FOCAL_LEN_MM_RPI     = Param['FOCAL_LEN_MM_RPI']
        self.FOCAL_LEN_MM_STEREO  = Param['FOCAL_LEN_MM_STEREO']
        
        # MATCH PARAMETERS
        self.PARAM1 = Param['PARAM1']
        self.PARAM2 = Param['PARAM2']
        self.first_match_results = []
        
        # REGULAR EXPRESSIONS
        self.MATCH_STD     = Param['MATCH_STD']
        self.MATCH_NUMBERS = Param['MATCH_NUMBERS']
        
        # Camera Parameters
        self.EXP     = Param['exp_time']*1000 # milli --> micro seconds 
        self.NUM_PIX = Param['NUM_PIX']
        self.failed_photo = 0
        self.ISO = Param['ISO']
        
        # Catalog parameters
        self.CAT_DIV = Param['cat_div']
        self.catalog_center_list = self.get_catalog_center_points()
        
        # initialize dynamic variables
        self.alpha = np.nan
        self.delta = np.nan
        self.phi   = np.nan
        
        self.quaternion = np.nan
        
        # multiprocessing params
        self.n_cores = multiprocessing.cpu_count()
        self.pool = multiprocessing.Pool(self.n_cores)
 #----------------------------------------------------------------------------------------- 
    def solve_LIS(self):
        """Method to solve the lost in space problem"""
        
        # step 1: Take Photo with RPi camera (or pull from photo archive)
        #self.take_photo()
        #self.pull_photo("60_45_800_4", "UTU")
        
        # step 2: convert to FITS file
        self.jpg2fits()
        
        # step 3: fire up the sextractor baby!!!
        sext_x_pix, sext_y_pix, sext_x_mm, sext_y_mm = self.apply_sextractor()
        
        # step 4: run match on entire catalog and get list of potential candidates
        self.map_match_and_radec_list_multiprocess()
        cand_table = self.get_table_with_matchs()
        match_candidates = self.get_match_candidates(cand_table)
        
        # step 5: iterate match to find orientation
        self.match_itr(match_candidates)
        
        # step 6: convert to quaternion
        self.quaternion = self.ECI_2_QUAT()
        
        # step 7: send to cdh (print to command line for rn)
        self.output_to_cdh()
        # self.close_tracker()
 #-----------------------------------------------------------------------------------------
    def take_photo(self):
        """Take a photo using RPi camera"""
        
        task = ["rpicam-still", "-o", self.DIR_IMG, "-t", "1", 
                "--width", str(self.NUM_PIX), "--height", str(self.NUM_PIX),
                "--shutter", str(self.EXP), "--gain", str(self.ISO), 
                "--awbgains", "1.0,1.0"]
        try:
            sp.run(task, check=True, stderr=sp.DEVNULL)
        except sp.CalledProcessError:
            self.failed_photo += 1
        
        #TODO change awbgains to be a property instantiated from param
 #-----------------------------------------------------------------------------------------
    def pull_photo(self, photo_id, archive="UTU"):
        """pull a photo from a specified catalog"""
        
        if archive == "RPi":
            self.DIR_IMG = f"{self.DIR_SCRIPT}/Sample_images/RPi/img_{photo_id}.jpg"
        elif archive == "UTU":
            self.DIR_IMG = f"{self.DIR_SCRIPT}/utu_img/img_{photo_id}.jpg"
        else:
            raise NameError("Image archive does not exist")
 #-----------------------------------------------------------------------------------------   
    def jpg2fits(self):
        """ 
        Generate .fits from .jpg image 
        """
        image = Image.open(self.DIR_IMG)
        image_bw = image.convert('L') # convert to greyscale
        x_size, y_size = image_bw.size
        fits1 = image_bw.getdata()
        if platform.machine().endswith('64'):
            fits2 = np.array(fits1, dtype=np.int32)
        else:
            fits2 = np.array(fits1)
        fits3 = fits2.reshape(y_size, x_size)
        fits4 = np.flipud(fits3)
        fits5 = fits.PrimaryHDU(data=fits4)
        fits5.writeto(self.DIR_FITS, overwrite=True)
        return 0
 #--------------------------------------------------------------------------------------------
    def apply_sextractor(self):
        """ Apply Source Extractor over an image and generates a catalog. """
        
        # run Sextractor with .fits image
        task = [self.SEXTRACTOR_STR, self.DIR_FITS]
        sp.run(task, cwd=self.DIR_STT_DATA, check=True, stderr=sp.DEVNULL)
        
        # Ensure you are looking in the right directory for the output file
        cat_path = os.path.join(self.DIR_STT_DATA, 'test.cat')
        sext1 = ascii.read(cat_path, format='sextractor')
    
        sext1.sort(['MAG_ISO'])
        sext2 = sext1[0:self.SEXTRACTOR_MAX_STARS]
        sext_x_pix = sext2['X_IMAGE']
        sext_y_pix = sext2['Y_IMAGE']
        sext_mag = sext2['MAG_ISO']
        sext_x_mm = (sext_x_pix - self.X_PIX) * self.PIX2MM_RPI
        sext_y_mm = (sext_y_pix - self.Y_PIX) * self.PIX2MM_RPI
        sext_filename = 'sext'
        ascii.write([sext_x_mm, sext_y_mm, sext_mag], sext_filename, delimiter=' ', format='no_header', overwrite=True,
                    formats={'X': '% 15.10f', 'Y': '% 15.10f', 'Z': '% 15.10f'}, names=['X', 'Y', 'Z'])
        return sext_x_pix, sext_y_pix, sext_x_mm, sext_y_mm
 #--------------------------------------------------------------------------------------------
    def get_catalog_center_points(self, x_center=0, y_center=0):
        """
        Get the center points for different catalogs segments, for a given distance and starting point.
        It considers declination of center site.
        """
        catalog_center_list = []
        
        for jj1 in range(y_center, 90, self.CAT_DIV):
            aux1 = (1 / np.cos(np.deg2rad(jj1)))
            distance_ra1 = int(round(self.CAT_DIV * aux1))
            for ii1 in range(x_center, 360, distance_ra1):
                 catalog_center_list.append([ii1, jj1])
        for jj2 in range(y_center - self.CAT_DIV, -90, -self.CAT_DIV):
            aux2 = (1 / np.cos(np.deg2rad(jj2)))
            distance_ra2 = int(round(self.CAT_DIV * aux2))
            for ii2 in range(x_center, 360, distance_ra2):
                catalog_center_list.append([ii2, jj2])
        catalog_center_list = catalog_center_list + [[0, 90], [0, -90]]
        return catalog_center_list
 #--------------------------------------------------------------------------------------------       
    def map_match_and_radec_list_multiprocess(self):
        """multi process the match function"""
        func = partial(StarTracker.call_match, 
                       DIR_STARS=self.DIR_STARS, 
                       DIR_PROJ_CAT_RPI=self.DIR_PROJ_CAT_RPI, 
                       PARAM=self.PARAM1, 
                       base='catalog')

        if self.n_cores == 1:
            self.first_match_results = list(map(func, self.catalog_center_list))
        else:        
            # Use the persistent pool initialized in __init__
            self.first_match_results = self.pool.map(func, self.catalog_center_list)
 #--------------------------------------------------------------------------------------------  
    @staticmethod
    def call_match(coords, DIR_STARS, DIR_PROJ_CAT_RPI, PARAM, base):
        """
        Call 'Match' with a single set of RA/DEC coordinates.
        """
        if isinstance(coords, str):
            ra_dec_str = coords
        else:
            ra, dec = coords
            ra_dec_str = "cat_RA_{}_DEC_{}".format(ra, dec)

        # Build a single string for the shell
        if base == 'catalog':
            match_cmd = "match {} 0 1 2 {}/{} 0 1 2 {}".format(DIR_STARS, DIR_PROJ_CAT_RPI, ra_dec_str, PARAM)
        else:
            match_cmd = "match {}/{} 0 1 2 {} 0 1 2 {}".format(DIR_PROJ_CAT_RPI, ra_dec_str, DIR_STARS, PARAM)

        # Use the legacy shell execution for better C-tool compatibility
        import subprocess as sp
        status, result = sp.getstatusoutput(match_cmd)
        return status, result
    
 #-----------------------------------------------------------------------------------------------------------------------
    def get_table_with_matchs(self):
        """
        Select RA/DEC values in which a successful match was obtained, and generate a 'match' table.
        """
        match_table = Table(names=('RA_center', 'DEC_center', 'sig', 'Nr'))

        # Unpack the (status, result) tuple returned by getstatusoutput
        for i, (status, result) in enumerate(self.first_match_results):
            if status == 0:  # 0 indicates success in shell execution
                ra, dec = self.catalog_center_list[i]
                # Use 'result' directly (it's the string output)
                match_data = self.MATCH_STD.findall(result)
                if match_data:
                    regexp_result = match_data[0]
                    sig = float(regexp_result[0])
                    nr = int(regexp_result[1])
                    match_table.add_row([str(ra), str(dec), sig, nr])

        if len(match_table) == 0:
            raise ValueError("---> ERROR: There is no match ...")
        else:
            # Combined stable sort: prioritize highest star count (Nr), 
            # then lowest error (sig) for ties.
            match_table.sort(['Nr', 'sig'], reverse=[True, False])
        return match_table
 #-----------------------------------------------------------------------------------------------------------------------    
    def get_match_candidates(self, match_table):
        """
        Select three 'match' candidates from 'first match table'.
        """
        len_table = len(match_table)
        if len_table == 1:
            match_candidates = match_table
        elif len_table == 2:
            match_candidates = match_table
        else:
            match_candidates = match_table[0:self.LIS_MAX_ITER]
        return match_candidates
#-----------------------------------------------------------------------------------------------------------------------    
    def apply_match_trans(self, data):
        """
        Apply the 'match' transformation between picture and projected catalog.
        This is: center of picture (pix) ==> point in the projected catalog (mm).
        """
        match_a = float(data[0])
        match_b = float(data[1])
        match_c = float(data[2])
        match_d = float(data[3])
        match_ra_mm, match_dec_mm = match_a, match_d
        match_roll_rad = np.arctan2(match_c, match_b)
        match_roll_deg = np.rad2deg(match_roll_rad)
        return match_ra_mm, match_dec_mm, match_roll_deg
#-----------------------------------------------------------------------------------------------------------------------
    def plane2sky(self, ra_match_mm, dec_match_mm, coords):
        """
        Deproject any arbitrary point in the camera. This is: mm ==> sky coordinates.
        """

        focal_len_mm = self.FOCAL_LEN_MM_RPI
        ra_catalog = coords[0]
        dec_catalog = coords[1]

        xi = ra_match_mm / float(self.FOCAL_LEN_MM_RPI)
        eta = dec_match_mm / float(self.FOCAL_LEN_MM_RPI)
        dec_catalog_rad = np.deg2rad(dec_catalog)
        arg1 = np.cos(dec_catalog_rad) - eta * np.sin(dec_catalog_rad)
        arg2 = np.arctan(xi / arg1)
        arg3 = np.sin(arg2)
        arg4 = eta * np.cos(dec_catalog_rad) + np.sin(dec_catalog_rad)
        alpha = ra_catalog + np.rad2deg(arg2)
        delta = np.rad2deg(np.arctan((arg3 * arg4) / xi))
        
        return alpha, delta
#-----------------------------------------------------------------------------------------------------------------------   
    def sky2plane(self, star_list, ra_project_point, dec_project_point):
        """
        With a list of 'matched' stars, project all in the tangent plane.
        """
        cat_projected = Table([[], [], []])
        stars_len = len(star_list)
            
        for index in range(stars_len):
            alpha_deg = star_list[index][0]
            delta_deg = star_list[index][1]
            mag = star_list[index][2]
            alpha_rad = np.deg2rad(alpha_deg)
            delta_rad = np.deg2rad(delta_deg)
            alpha_0_rad = np.deg2rad(ra_project_point)
            delta_0_rad = np.deg2rad(dec_project_point)
            xi_up = np.cos(delta_rad) * np.sin(alpha_rad - alpha_0_rad)
            xi_down = np.sin(delta_0_rad) * np.sin(delta_rad)\
                + np.cos(delta_0_rad) * np.cos(delta_rad) * np.cos(alpha_rad - alpha_0_rad)
            xi = xi_up/xi_down
            eta_up = np.cos(delta_0_rad) * np.sin(delta_rad)\
                - np.sin(delta_0_rad) * np.cos(delta_rad) * np.cos(alpha_rad - alpha_0_rad)
            eta_down = xi_down
            eta = eta_up / eta_down
            xi_mm = xi * self.FOCAL_LEN_MM_RPI
            eta_mm = eta * self.FOCAL_LEN_MM_RPI
            cat_projected.add_row([xi_mm, eta_mm, mag])
        cat_name = f"{self.DIR_PROJ_CAT_RPI}/{self.NEW_PROJ_CAT}"
        ascii.write(cat_projected, cat_name, delimiter=' ', format='no_header', overwrite=True,
                    formats={'X': '% 15.5f', 'Y': '% 15.5f', 'Z': '% 15.2f'}, names=['X', 'Y', 'Z'])
        return 0
    #-----------------------------------------------------------------------------------------------------------------------
    @staticmethod
    def search_catalog_objects(DIR_NORMAL_CAT_RPI, ra_first_match, dec_first_match):
        """
        Search in the normal catalog for all sky-objects (to the nearest catalog), and create a table.
        """
        ra_catalog   = int(round(ra_first_match))
        dec_catalog  = int(round(dec_first_match))
        new_cat_name = f"{DIR_NORMAL_CAT_RPI}/cat_RA_{ra_catalog}_DEC_{dec_catalog}"
        
        try:
            new_cat = ascii.read(new_cat_name)
            noproj_table = new_cat.columns[:3]
            return Table(noproj_table)
        except FileNotFoundError:
            print(f"Warning: Catalog file not found at {new_cat_name}")
            return Table()
#-----------------------------------------------------------------------------------------------------------------------       
    def match_itr(self, match_candidates):
        attempts = 0
        while attempts < self.LIS_MAX_ITER:
            try:
                # --- ITERATION 1 ---
                # Start with the catalog center from the match table
                first_coords = [int(match_candidates[attempts][0]), int(match_candidates[attempts][1])]
                status, results = self.call_match(first_coords, self.DIR_STARS, self.DIR_PROJ_CAT_RPI, self.PARAM1, 'catalog')

                data1 = self.MATCH_NUMBERS.findall(results)[0]
                ra_mm1, dec_mm1, roll1 = self.apply_match_trans(data1)
                # Plane2sky uses the initial catalog center
                alpha1, delta1 = self.plane2sky(ra_mm1, dec_mm1, first_coords)

                # --- ITERATION 2 ---
                # Use alpha1/delta1 to find nearby objects and project them
                tab2 = self.search_catalog_objects(self.DIR_NORMAL_CAT_RPI, alpha1, delta1)
                self.sky2plane(tab2, alpha1, delta1)

                # call_match_once equivalent: match against the NEW_PROJ_CAT
                status2, results2 = self.call_match(self.NEW_PROJ_CAT, self.DIR_STARS, self.DIR_PROJ_CAT_RPI, self.PARAM2, 'catalog')
                data2 = self.MATCH_NUMBERS.findall(results2)[0]
                ra_mm2, dec_mm2, roll2 = self.apply_match_trans(data2)
                # Plane2sky now uses alpha1/delta1 as the center
                alpha2, delta2 = self.plane2sky(ra_mm2, dec_mm2, [alpha1, delta1])

                # --- ITERATION 3 ---
                # Project the SAME table onto the refined alpha2/delta2 center
                self.sky2plane(tab2, alpha2, delta2)
                status3, results3 = self.call_match(self.NEW_PROJ_CAT, self.DIR_STARS, self.DIR_PROJ_CAT_RPI, self.PARAM2, 'catalog')
                data3 = self.MATCH_NUMBERS.findall(results3)[0]
                ra_mm3, dec_mm3, roll3 = self.apply_match_trans(data3)
                # Final alpha/delta uses alpha2/delta2 as the center
                alpha3, delta3 = self.plane2sky(ra_mm3, dec_mm3, [alpha2, delta2])

                # Assign final values to class properties
                self.alpha, self.delta, self.phi = alpha3, delta3, roll3
                break 

            except Exception as err:
                #print(f"---> ERROR: {err}")
                # skip to next iteration
                attempts += 1
#-----------------------------------------------------------------------------------------------------------------------
    def ECI_2_QUAT(self):
        """convert alpha, delta, phi to a quaternion"""

        # convert to ECI unit vector
        u_ECI_x = math.cos(math.radians(self.delta)) * math.cos(math.radians(self.alpha))
        u_ECI_y = math.cos(math.radians(self.delta)) * math.sin(math.radians(self.alpha))
        u_ECI_z = math.sin(math.radians(self.delta))

        # convert to quaternion
        w = math.cos(math.radians(self.phi) / 2)
        x = math.sin(math.radians(self.phi) / 2) * u_ECI_x
        y = math.sin(math.radians(self.phi) / 2) * u_ECI_y
        z = math.sin(math.radians(self.phi) / 2) * u_ECI_z

        return [w,x,y,z]
#-----------------------------------------------------------------------------------------------------------------------
    def output_to_cdh(self):
            # Output result to CDH
            # This is a placeholder function. The actual implementation will depend on the communication protocol with CDH.
            print(f"RA: {self.alpha}, DEC: {self.delta}, ROLL: {self.phi}")           
#-----------------------------------------------------------------------------------------------------------------------           
    def close_tracker(self):
        self.pool.close()
        self.pool.join()