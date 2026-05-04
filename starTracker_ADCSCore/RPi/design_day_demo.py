import customtkinter
from PIL import Image
import os
import numpy as np
import time
from utu_flight import StarTracker as st
from stt_flight_param import Param
import math

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        self.title("UTU-Core Star Tracker Demo")
        self.geometry("1900x1000")
        
        # Configure grid: Column 0 is for labels, Column 1 is for sliders
        self.grid_columnconfigure(2, weight=1)

        # --- EXPOSURE SECTION ---
        self.label_exp = customtkinter.CTkLabel(self, text="Exposure (ms)", font=("Courier", 15, "bold"))
        self.label_exp.grid(row=0, column=0, padx=20, pady=(20, 0), sticky="w")

        self.slider_exp = customtkinter.CTkSlider(self, from_=100, to=2000, number_of_steps=19, command=self.exp_event)
        self.slider_exp.grid(row=1, column=0, padx=20, pady=(0, 20), sticky="ew")

        # --- ISO SECTION ---
        self.label_iso = customtkinter.CTkLabel(self, text="ISO Settings",font=("Courier", 15, "bold"))
        self.label_iso.grid(row=2, column=0, padx=20, pady=(20, 0), sticky="w")
        
        self.slider_iso = customtkinter.CTkSlider(self, from_=1, to=8, number_of_steps=7, command=self.iso_event)
        self.slider_iso.grid(row=3, column=0, padx=20, pady=(0, 20), sticky="ew")
        
        # --- HEADING SECTION ---
        self.label_heading = customtkinter.CTkLabel(self, text="Heading (deg)",font=("Courier", 15, "bold"))
        self.label_heading.grid(row=4, column=0, padx=20, pady=(20, 0), sticky="w")

        self.slider_exp = customtkinter.CTkSlider(self, from_=0, to=180, number_of_steps=3, command=self.heading_event)
        self.slider_exp.grid(row=5, column=0, padx=20, pady=(0, 20), sticky="ew")

        # --- BUTTON ---
        self.button = customtkinter.CTkButton(self, text="Run Star Tracker",font=("Courier", 15, "bold") ,command=self.button_callback)
        self.button.grid(row=6, column=0, padx=20, pady=40, sticky="ew")
        
        
        # --- ST PHOTO ---
        self.current_dir = os.getcwd()
        self.title_label = customtkinter.CTkLabel(self, text="Star Tracker Photo",font=("Courier", 20, "bold"))
        self.title_label.grid(row=0, column=1, padx=20, pady=5, sticky="ew")
        self.image_path = f"{self.current_dir}/utu_img/img_0_45_100_1.jpg"
        self.st_image = customtkinter.CTkImage(light_image=Image.open(self.image_path),
                                               dark_image=Image.open(self.image_path),
                                               size=(500,500))
        self.image_label = customtkinter.CTkLabel(self, image=self.st_image, text="")
        self.image_label.grid(row=1, column=1, rowspan=6, padx=200, pady=20, sticky="ew")
        
        # --- skyview photo ---
        self.sv_label = customtkinter.CTkLabel(self, text="NASA SkyView Photo",font=("Courier", 20, "bold"))
        self.sv_label.grid(row=0, column=2, padx=100, pady=5, sticky="ew")
        self.sv_path = f"{self.current_dir}/skyview_img/sky_0.jpg"
        self.sv_image = customtkinter.CTkImage(light_image=Image.open(self.sv_path),
                                               dark_image=Image.open(self.sv_path),
                                               size=(500,500))
        self.sv_image_label = customtkinter.CTkLabel(self, image=self.sv_image, text="")
        self.sv_image_label.grid(row=1, column=2, rowspan=6, padx=100, pady=20, sticky="ew")
        
        # --- RESULTS ---
        self.results_title_label = customtkinter.CTkLabel(self, text="Star Tracker Results:",font=("Courier", 30, "bold"))
        self.results_title_label.grid(row=7, column=1, columnspan=2, padx=20, pady=5, sticky="ew")
        
        self.results_label = customtkinter.CTkLabel(self, text="RA: ---, DEC: ---, ROLL: ---, Time: ---",font=("Courier", 30, "bold"))
        self.results_label.grid(row=8, column=1, columnspan=2, padx=20, pady=20, sticky="ew")
        
        # Data storage
        self.exposure = 100
        self.ISO = 1
        self.heading = 0
        self.starTracker = st(Param)
#---------------------------------------------------------------------------------------------------------------------------
    def exp_event(self, value):
        self.exposure = int(value)
        # Update the title label dynamically so user sees the current value!
        self.label_exp.configure(text=f"Exposure: {self.exposure} ms")
#---------------------------------------------------------------------------------------------------------------------------
    def iso_event(self, value):
        self.ISO = int(value)
        self.label_iso.configure(text=f"ISO: {self.ISO}")
#---------------------------------------------------------------------------------------------------------------------------        
    def heading_event(self, value):
        self.heading = int(value)
        self.label_heading.configure(text=f"Heading: {self.heading} deg")
#---------------------------------------------------------------------------------------------------------------------------
    def button_callback(self):
        # print for debugging
        #print(f"Tracking with Exp: {self.exposure}, ISO: {self.ISO}, Heading: {self.heading}")
        
        # set image path, clear star tracker variables, and pull photo into star tracker class
        self.image_path = f"{self.current_dir}/utu_img/img_{self.heading}_45_{self.exposure}_{self.ISO}.jpg"
        self.starTracker.alpha = self.starTracker.delta = self.starTracker.phi = np.nan
        flag = "failure"
        self.starTracker.pull_photo(f"{self.heading}_45_{self.exposure}_{self.ISO}")
        
        # run star tracker lost-in-space algorithm
        start_time = time.monotonic()
        try: 
            self.starTracker.solve_LIS()
        except Exception as err:
            #print(f"----> {err}")
            f = 1
            
        if not math.isnan(self.starTracker.alpha):    
               flag = "success"
        end_time = time.monotonic()
        delta_time = end_time - start_time
        
        #  Create the new star tracker image object
        new_img = Image.open(self.image_path)
        self.st_image = customtkinter.CTkImage(light_image=new_img, dark_image=new_img, size=(500, 500))
        
        # create the new skyview image object
        self.sv_path = f"{self.current_dir}/skyview_img/sky_{self.heading}.jpg"
        new_sv_image = Image.open(self.sv_path)
        self.sv_image = customtkinter.CTkImage(light_image=new_sv_image, dark_image=new_sv_image, size=(500, 500))
        
        # CONFIGURE the labels
        self.image_label.configure(image=self.st_image)
        self.sv_image_label.configure(image=self.sv_image)
        self.results_title_label.configure(text=f"Star Tracker Results: {flag}")
        if flag == "success":
            self.results_label.configure(fg_color="green", text=f"RA: {round(self.starTracker.alpha,4)} deg | DEC: {round(self.starTracker.delta,4)} deg | ROLL: {round(self.starTracker.phi,4)} deg | Time: {round(delta_time,4)} seconds")
        else:
            self.results_label.configure(fg_color="red",text=f"RA: {round(self.starTracker.alpha,4)} deg | DEC: {round(self.starTracker.delta,4)} deg | ROLL: {round(self.starTracker.phi,4)} deg")   
#-----------------------------------------end of class ---------------------------------------------------------------------
app = App()
app.mainloop()