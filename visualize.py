
import matplotlib.pyplot as plt
import numpy as np
import scienceplots
import math
import time


class Visualizer():
    def __init__(self, N):
        self.fig = plt.figure(dpi=300)
        self.style = plt.style.use(['science',"grid"])
        self.labels = []
        for i in range(N):
            self.labels.append("Agent " + str(i+1))
        # self.cs = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.5, 0.5, 0.5)] # colors for four agents
        # self.cs = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.5, 0.5, 0.2), (0.5, 0.5, 0.5), (0.5, 0.5, 0.8), (0.5, 0.5, 0.1), (0.5, 0.5, 0.3), (0.5, 0.5, 0.4)]
        self.cs = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.5, 0.0, 0.2), (0.5, 0.5, 0.5)]

        # self.cs = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0)]

        # self.fig2d = plt.figure(figsize=(7,7), dpi=100)
        # self.ax2d = self.fig2d.add_subplot(111)
        
        
    #  P_1, P_2, P_3, P_4

    def simulation_swarm_3d(self, P, N, title = "Trajectory of Crazyflies"):
        p_ = {}
        ax = self.fig.add_subplot(111, projection='3d')
        ax.set_xlabel("X", fontsize = 8)
        ax.set_ylabel("Y", fontsize = 8)
        ax.set_zlabel("Z", fontsize = 8)
        # ax.autoscale(tight=True)
        for i in range (N):
            p_[i] = P[:, 3*i:3*i+3]
            ax.plot(p_[i][:,0], p_[i][:,1], p_[i][:,2], '-', label=1, c=self.cs[i])
            ax.set_title(title)
            # add the agent number to the list in for loop
        #ax.set_xticks([10, 20, 30, ...]) ---> to modify the individual thicks 
        # self.plot_formation(N, P[ 10000,:], ax)
        self.plot_formation(N, P[ 30000,:], ax)
        # self.plot_formation(N, P[ 70000,:], ax)
        ax.legend(self.labels, loc='upper right', fontsize = 8)
        plt.show()

    def pos_simulation_all(self, P, N, at):  # iteration x 3*N matris
        # In this function, I plot all pasitions on all three axis
        v = np.array([np.shape(P)])
        v = int(v[0,0])
        
        X_positions = np.zeros((v,N))
        Y_positions = np.zeros((v,N)) 
        Z_positions = np.zeros((v,N))
    
        for i in range (N):
            X_positions[:,i] = P[:, 3*i]
            Y_positions[:,i] = P[:, 3*i+1]
            Z_positions[:,i] = P[:, 3*i+2]
        t = np.linspace(0, at, v)
        
        self.plot_2d(N, t, X_positions, "Positions of Agents on X axis")
        self.plot_2d(N, t, Y_positions, "Positions of Agents on Y axis")
        self.plot_2d(N, t, Z_positions, "Positions of Agents on Z axis")
        # Plot position of drones in time

    def plot_2d (self, N, t, val, title_name = " "):
        plt.style.use(['science',"grid"])
        plt.figure(dpi=300)
        for i in range (N):
            plt.plot(t, val[:,i], '-', label=1, c=self.cs[i])
        plt.title(title_name)
        plt.legend(self.labels, loc='upper right', fontsize = 8)
        plt.show()

    def plot_formation_error(self, for_er, N, for_time):
        for_time = np.linspace(0, for_time, np.shape(for_er)[0])
        plt.style.use(['science',"grid"])
        plt.figure(dpi=300)
        labels = []
        for i in range (N):
            for j in range (N):
                if i != j and i < j:
                    label = "Error between " + str(i+1) + " and " + str(j+1)
                    labels.append(label)
                    plt.plot(for_time, for_er[:,int(j*(j-1)/2+i)], '-')
        plt.legend(labels, loc='upper right', fontsize = 8) 
        plt.title("Formation Errors")
        plt.show()

    def plot_formation(self,N, x, ax):
        x_for = np.zeros((3, N))
        for i in range(N):
            xx = np.array([x[i*3], x[i*3+1], x[i*3+2]])
            # print(xx)
            x_for[:, i] = xx
        # print(x_for)
        for i in range(N):
            for j in range(N):
                px = np.array([x_for[0, i], x_for[0, j]])
                py = np.array([x_for[1, i], x_for[1, j]])
                pz = np.array([x_for[2, i], x_for[2, j]])
                ax.plot(px, py, pz, 'k--', linewidth=1.5)
                ax.plot(x_for[0, i], x_for[1, i], x_for[2, i], '+', markersize=20, color= self.cs[i])


    







