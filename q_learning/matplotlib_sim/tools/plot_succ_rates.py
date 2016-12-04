# -----------------------------------
# utility functions to plot the succ_rate
# Author: Tao Chen
# Date: 2016.10.28
# -----------------------------------
import matplotlib.pyplot as plt
import numpy as np
import cPickle
import os

def plot_rates(path):
    file_path = os.path.join(path, 'succ_rate.cpickle')
    with open(file_path, 'rb') as f:
        rates = cPickle.load(f)
        data = np.zeros((len(rates), 2))
        count = 0
        for key,value in rates.items():
            data[count, 0]  = key
            data[count , 1] = value
            count += 1

        data = data[data[:,0].argsort()]

        plot_interval = 1
        plt.plot(data[::plot_interval, 0],data[::plot_interval, 1])
        plt.show()

if __name__ == '__main__':
    path = raw_input('please enter the path of the file [succ_rate.cpickle] that contains succ_rate data:')
    plot_rates(path)