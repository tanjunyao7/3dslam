import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy.interpolate import griddata
import sys

def get_color(v):
    if v == 0.0:
        return 'green'
    elif v  < 1.95:
        return (1, 100/255, 32/255)  # RGB values for QColor(1, 100, 32)
    elif 1.95 <= v< 2.2:
        return 'yellow'
    elif 2.2 <= v <= 2.35:
        return (255/255, 192/255, 203/255)  # RGB values for QColor(255, 192, 203)
    else:
        return 'red'


if __name__ == '__main__':
    filename = sys.argv[1]
    directory = filename.replace(filename.split('/')[-1],'')

    with open(filename,'r') as file:
        data = file.readlines()[15:]
        data = np.array([x.replace('\n','').split(';') for x in data])
        x = data[:,0].astype(float)
        y = data[:,1].astype(float)
        z = data[:,3].astype(float)


    # Define a grid for interpolation
    xi, yi = np.meshgrid(np.linspace(min(x), max(x), 100), np.linspace(min(y), max(y), 100))

    # Interpolate values
    zi = griddata((x, y), z, (xi,yi), method='linear')

    # colors = [get_color(v) for v in zi]
    # cmap = LinearSegmentedColormap.from_list('custom_cmap',colors,N=256)


    # Create a contour plot
    plt.contour(xi, yi, zi, levels=10, linewidths=0.5, colors='k')
    plt.contourf(xi, yi, zi, levels=10, cmap='viridis')
    # Add color bar and labels
    plt.colorbar(label='Radiation')

    plt.savefig(directory + '/contourmap.png')
    plt.clf()

    plt.imshow(zi, cmap='viridis', interpolation='nearest', origin='lower')
    plt.colorbar(label='Radiation')

    plt.savefig(directory + '/heatmap.png')
