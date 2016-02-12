import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pylab as p
import numpy as np
from scipy import stats
import glob
import os
import logging

def plot_2d_three_sets(sets, axis='xy', xrange=[0.0, 1.0], yrange=[0.0, 1.0]):
    if axis == 'xy':
        plt.plot(sets[0][:,0], sets[0][:,1], 'ro', 
                 sets[1][:,0], sets[1][:,1], 'go', 
                 sets[2][:,0], sets[2][:,1], 'bo')
        plt.xlabel('x')
        plt.ylabel('y')
    plt.xlim([xrange[0], xrange[1]])
    plt.ylim([yrange[0], yrange[1]])
    plt.show()
    
    
def plot_3d_points(points, x_scale=[-4.0, 4.0], y_scale=[-4.0, 4.0], z_scale=[-4.0, 4.0]):
    fig = p.figure()
    ax = Axes3D(fig)
    #ax.set_xticks([-1.0, 1.0])
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]
    ax.set_xlabel("v1")
    ax.set_ylabel("v2")
    ax.set_zlabel("alpha")
    ax.view_init(elev=17.0, azim=-58.0)
    #ax.auto_scale_xyz([-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0])
    ax.scatter(x, y, z, c='r')
    ax.auto_scale_xyz(x_scale, y_scale, z_scale)
    plt.show()
    plt.clf()
    
def plot_3d_n_sets(sets=[],
                   balls=[], 
                   labels=[], 
                   colormap=[], 
                   show_legend=True, 
                   save=False,
                   path="",
                   filename="emd.png"):
    if len(labels) == 0:
        labels=['x', 'y', 'z']
    if len(colormap) == 0:        
        colormap = ['#000000' for i in xrange(len(sets) + len(balls))] 
    fig = plt.figure()
    ax = fig.gca(projection='3d') 
    n = 0   
    for i in xrange(len(sets)):
        x = [sets[i][0][0], sets[i][1][0]]
        y = [sets[i][0][1], sets[i][1][1]]
        z = [sets[i][0][2], sets[i][1][2]]
        if show_legend:
            ax.plot(x, y, z, label='environment', color=colormap[i])
        else:            
            ax.plot(x, y, z, color=colormap[i])
        ax.set_xlabel(labels[0])
        ax.set_ylabel(labels[1])
        ax.set_zlabel(labels[2])
        n = i
    n += 1
    for ball in balls:
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 50)
    
        x = ball[0][0] + ball[1] * np.outer(np.cos(u), np.sin(v))
        y = ball[0][1] + ball[1] * np.outer(np.sin(u), np.sin(v))
        z = ball[0][2] + ball[1] * np.outer(np.ones(np.size(u)), np.cos(v))        
        ax.plot_surface(x, y, z,  rstride=4, cstride=4, color=colormap[n])
        n += 1   
    ax.legend()
    if save:
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        plt.savefig(os.path.join(path, filename))
        plt.clf()
        return
    plt.show()
    plt.clf()
    
'''def plot_2d_n_sets(sets,
                   circles=[], 
                   labels=[], 
                   xlabel='x', 
                   ylabel='y', 
                   axis='xy', 
                   x_range=[0.0, 1.0], 
                   y_range=[0.0, 1.0], 
                   plot_type='lines', 
                   show_legend=True,
                   idx=None, 
                   lw=5,
                   color_map=[],
                   save=False,
                   path="",
                   filename="emd.png"):
    ps = []    
    if len(labels) != len(sets):         
        labels=['default' for i in xrange(len(sets))]
    if len(color_map) == 0:
        color_map = ['#000000' for i in xrange(len(sets))]
    fig = plt.figure()   
    if plot_type == 'lines':
        for i in xrange(len(sets)):
            if i == idx:                
                p, = plt.plot(sets[i][:,0], sets[i][:,1], color_map[i], label=labels[i], linewidth=lw)
            else:                
                p, = plt.plot(sets[i][:,0], sets[i][:,1], color_map[i], label=labels[i])
            ps.append(p)
        if show_legend:
            plt.legend(ps)
    else:        
        ax = fig.add_subplot(111)
        for i in xrange(len(sets)):
            ax.scatter(sets[i][:,0], sets[i][:,1], c=np.random.rand(3,1), label=labels[i], s=13)
        if show_legend:
            plt.legend(loc='upper left')            
    for circle in circles:               
        ax = fig.add_subplot(111)
        circ = plt.Circle((circle[0], circle[1]), radius=circle[2], color='g', fill=True)
        ax.add_patch(circ)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.xlim([x_range[0], x_range[1]])
    plt.ylim([y_range[0], y_range[1]]) 
    
    if save:
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        plt.savefig(os.path.join(path, filename))
        plt.clf()
        return
    plt.show()
    plt.clf()'''
def plot_2d_n_sets(sets,
                   circles=[], 
                   labels=[], 
                   xlabel='x', 
                   ylabel='y', 
                   axis='xy', 
                   x_range=[0.0, 1.0], 
                   y_range=[0.0, 1.0], 
                   plot_type='lines', 
                   show_legend=True,
                   idx=None, 
                   lw=5,
                   color_map=[],
                   save=False,
                   path="",
                   filename="emd.png"):
    ps = []    
    if len(labels) != len(sets):         
        labels=['default' for i in xrange(len(sets))]
    if len(color_map) == 0:
        color_map = ['#000000' for i in xrange(len(sets))]
    fig = plt.figure()   
    if plot_type == 'lines':
        for i in xrange(len(sets)):
            if i == idx:                
                p, = plt.plot(sets[i][:,0], sets[i][:,1], color_map[i], label=labels[i], linewidth=lw)
            else:                
                p, = plt.plot(sets[i][:,0], sets[i][:,1], color_map[i], label=labels[i])
            ps.append(p)
        if show_legend:
            plt.legend(ps)
    else:        
        ax = fig.add_subplot(111)
        for i in xrange(len(sets)):
            ax.scatter(sets[i][:,0], sets[i][:,1], c=np.random.rand(3,1), label=labels[i], s=13)
        if show_legend:
            plt.legend(loc='upper left')            
    for circle in circles:               
        ax = fig.add_subplot(111)
        circ = plt.Circle((circle[0], circle[1]), radius=circle[2], color='g', fill=True)
        ax.add_patch(circ)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.xlim([x_range[0], x_range[1]])
    plt.ylim([y_range[0], y_range[1]]) 
    
    if save:
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        plt.savefig(os.path.join(path, filename))
        plt.clf()
        return
    plt.show()
    plt.clf()
    
def plot_histogram(H, xedges, yedges, save=False, barlabel="Probability", path="", filename="hist1.png"):
    #Hmasked = np.ma.masked_where(H==0,H)
    fig2 = plt.figure()
    plt.pcolormesh(xedges,yedges,H)
    plt.xlabel('x')
    plt.ylabel('y')
    cbar = plt.colorbar()
    cbar.ax.set_ylabel(barlabel)
    if save:
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        fig2.savefig(os.path.join(path, filename))
    else:
        plt.show()
    plt.clf()
    
    #fig = plt.figure(figsize=(15, 15))
    #ax = fig.add_subplot(132)
    #ax.set_title('pcolormesh: exact bin edges')
    #X, Y = np.meshgrid(xedges, yedges)
    #ax.pcolormesh(X, Y, H)
    #ax.set_aspect('equal')
    #plt.show()

def plot_2d_two_sets(sets, labels, axis='xy', xrange=[0.0, 1.0], yrange=[0.0, 1.0]):
    if axis == 'xy':
        p1, = plt.plot(sets[0][:,0], sets[0][:,1], label=labels[0])
        p2, = plt.plot(sets[1][:,0], sets[1][:,1], label=labels[1])
        #plt.plot(sets[0][:,0], sets[0][:,1], 'ro', sets[1][:,0], sets[1][:,1], 'bo')
        plt.legend([p1, p2])
        plt.xlabel('x')
        plt.ylabel('y')
    plt.xlim([xrange[0], xrange[1]])
    plt.ylim([yrange[0], yrange[1]])
    plt.show()

def plot_2d_points(points, axis='xy', xrange=[0.0, 1.0], yrange=[0.0, 1.0]):
    if axis == 'xy':
        plt.plot(points[:,0], points[:,1], 'ro')
        plt.xlabel('x')
        plt.ylabel('y')
    elif axis == 'xz':
        plt.plot(points[:,0], points[:,2], 'ro')
        plt.xlabel('x')
        plt.ylabel('z')            
    elif axis == 'yz':
        plt.plot(points[:,1], points[:,2], 'ro')
        plt.xlabel('y')
        plt.ylabel('z')        
        
    plt.xlim([xrange[0], xrange[1]])
    plt.ylim([yrange[0], yrange[1]])
    plt.show()
    
    
def plot_heat_2d(m1, m2, xmin, xmax, ymin, ymax):
    X, Y = np.mgrid[xmin:xmax:1000j, ymin:ymax:1000j]    
    positions = np.vstack([X.ravel(), Y.ravel()])
    values = np.vstack([m1, m2])
    kernel = stats.gaussian_kde(values)    
    Z = np.reshape(kernel(positions).T, X.shape)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(np.rot90(Z), cmap=plt.cm.gist_earth_r,
              extent=[xmin, xmax, ymin, ymax])
    ax.plot(m1, m2, 'k.', markersize=2)
    ax.set_xlim([xmin, xmax])
    ax.set_ylim([ymin, ymax])
    plt.show()
    
if __name__ == "__main__":
    sets = []
    sets.append(np.array([[0.0, 0.0, 0.0], [0.5, 0.3, 0.5]]))
    sets.append(np.array([[0.8, 0.3, 0.5], [1.0, 1.5, 0.8]]))
    plot_3d_n_sets(sets=sets, show_legend=False)
