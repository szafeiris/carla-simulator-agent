import matplotlib.pyplot as plt
import numpy as np
import h5py, glob, os

ACT_THROTTLE = 0
ACT_BRAKE    = 1
ACT_LEFT     = 2
ACT_RIGHT    = 3
ACT_NOTHNG   = 4

with h5py.File('data.h5', 'r') as hf:
    dataset_names = list(hf.keys())

    datalist = []

    for img in dataset_names:
        (_, t, b, s , r, _) = img.replace('IMG.img_data', '').replace('.png', '').split('_')
        
        t = 1 if float(t)>0.5 else 0
        b = 1 if float(b)>0.5 else 0
        s = float(s)

        #    (s - min) / (max - min) 
        s = (s+0.417855829000473)/(0.7664270401000977+0.417855829000473)

        # 0.35 is the new mean

        ss = -1 if s < 0.35 else 1
        ss = 0 if abs(s -0.35) < 0.007 else ss


        if ((t, b, ss) == (1, 0, 0)):
            action = ACT_THROTTLE
        elif ((t, b, ss) == (1, 0, -1)):
            action = ACT_LEFT
        elif ((t, b, ss) == (1, 0, 1)):
            action = ACT_RIGHT
        elif ((t, b, ss) == (0, 1, 0)):
            action = ACT_BRAKE
        else:
            action = ACT_NOTHNG

        datalist.append([img, (t, b, ss), action])
        print(datalist[len(datalist)-1])


    # ss = np.array(ss)
    # print(np.min(ss))           # -0.417855829000473
    # print(ss.max())             # 0.7664270401000977
    # print(ss.mean())            # 0.0010923585123650572
    # print(np.median(ss))        # -3.084455238422379e-05
    # print(ss.std())             # 0.0702201795232028




