# -----------------------------------
# utility functions to plot the succ_rate, hit_cars_rate, hit_wall_rate, hit_hard_time_limit_rate, out_of_time_rate
# Author: Tao Chen
# Date: 2016.10.28
# -----------------------------------
import matplotlib.pyplot as plt
import numpy as np
import cPickle
import os

def plot_rates(file_path):
    file_path = os.path.join(file_path, 'rates.cpickle')
    with open(file_path, 'rb') as f:
        rates = cPickle.load(f)
        trials = rates.keys()
        succ_rates = [rates[trial]['succ_rate'] for trial in trials]
        hit_cars_rates = [rates[trial]['hit_cars_rate'] for trial in trials]
        hit_wall_rates = [rates[trial]['hit_wall_rate'] for trial in trials]
        # hit_hard_time_limit_rates = [rates[trial]['hit_hard_time_limit_rate'] for trial in trials]
        out_of_time_rates = [rates[trial]['out_of_time_rate'] for trial in trials]

        data = np.zeros((len(trials), 6))
        data[:,0] = trials
        data[:,1] = succ_rates
        data[:,2] = hit_cars_rates
        data[:,3] = hit_wall_rates
        data[:,4] = out_of_time_rates
        # data[:,5] = hit_hard_time_limit_rates

        data = data[data[:, 0].argsort()]

        plot_interval = 1
        plt.plot(data[::plot_interval, 0], data[::plot_interval, 1])
        plt.plot(data[::plot_interval, 0], data[::plot_interval, 2])
        plt.plot(data[::plot_interval, 0], data[::plot_interval, 3])
        plt.plot(data[::plot_interval, 0], data[::plot_interval, 4])
        plt.plot(data[::plot_interval, 0], data[::plot_interval, 5])

        plt.plot(trials, succ_rates)
        plt.plot(trials, hit_cars_rates)
        plt.plot(trials, hit_wall_rates)
        plt.plot(trials, out_of_time_rates)
        # plt.plot(trials, hit_hard_time_limit_rates)
        plt.legend(['succ_rates', 'hit_cars_rates', 'hit_wall_rates', 'out_of_time_rates'],fontsize = 'x-small')
        # plt.legend(['succ_rates', 'hit_cars_rates', 'hit_wall_rates', 'out_of_time_rates', 'hit_hard_time_limit_rates'],fontsize = 'x-small')
        plt.show()



if __name__ == '__main__':
    path = raw_input('please enter the path of the file [rates.cpickle] that contains rates data:')
    plot_rates(path)