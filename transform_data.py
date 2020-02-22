import cv2
import datetime as dt
import h5py
import matplotlib.pyplot as plt
import matplotlib.pylab as plb
import numpy as np
import os
import pandas as pd
from glob import glob


HEIGHT = 640
WIDTH = 480
CHANNELS = 3
SHAPE = (HEIGHT, WIDTH, CHANNELS)

images = glob(os.path.join('img_data', "*.png"))


with h5py.File('data.h5', 'w') as hf: 
    for img in images:            
        print('Processing file: ' + img)
        # Images
        image = cv2.imread(img)
        Xset = hf.create_dataset('IMG.' + img, data=image)
        #Yset = hf.create_dataset('LBL.' + img.replace('.png', ''))


## Read image from dataset
 
#with h5py.File('data.h5', 'r') as hf:
#    dataset_names = list(hf.keys())
#    print(dataset_names[0])
#    img = hf['IMG.' + images[0].replace("\\\\", "\\")].value
#
#    print(img)
#
#    cv2.imwrite('data.png', img)