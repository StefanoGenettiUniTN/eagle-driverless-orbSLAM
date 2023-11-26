import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import maximum_filter

sessions = ['lab-otto', 'outdoor', 'povo-garage', 'povo1-interno']

ORBSLAM2_SCALE = (1, 1, 1)
ORBSLAM3_SCALE = (5, 5, 5)

def plot_orbslam2(session: str, scale = (1,1,1), offset = (0,0,0)):
    print(f'Plotting {session} OrbSLAM 2')
    with open(f'logs/{session}_camera_os2.csv') as f:
        content = f.readlines()[1:]
        x = []
        y = []
        z = []
        for line in content:
            coords = line.split(',')
            x.append((float(coords[1]) + offset[0]) * scale[0])
            y.append((float(coords[3]) + offset[1]) * scale[1])
            z.append((float(coords[2]) + offset[2]) * scale[2])

        plt.plot(x, y, 'r.', markersize=1, label='OrbSLAM 2')

def plot_orbslam3(session: str, scale = (1,1,1), offset = (0,0,0)):
    print(f'Plotting {session} OrbSLAM 3')
    with open(f'logs/{session}_camera_os3.csv') as f:
        content = f.readlines()[1:]
        x = []
        y = []
        z = []
        for line in content:
            coords = line.split(',')
            x.append((float(coords[1]) + offset[0]) * scale[0])
            y.append((float(coords[3]) + offset[1]) * scale[1])
            z.append((float(coords[2]) + offset[2]) * scale[2])

        plt.plot(x, y, 'b.', markersize=1, label='OrbSLAM 3')

def plot_cones(session: str, scale = (1,1,1), offset = (0,0,0)):
    print(f'Plotting {session} Cones')
    with open(f'logs/{session}_cones.csv') as f:
        content = f.readlines()[1:]
        x = []
        y = []
        z = []
        for line in content:
            coords = line.split(',')
            x.append((float(coords[1]) + offset[0]) * scale[0])
            y.append((float(coords[3]) + offset[1]) * scale[1])
            z.append((float(coords[2]) + offset[2]) * scale[2])

        plt.plot(x, y, '.', color='orange', markersize=1, label='Cones')

for session in sessions:
    plt.figure(session)
    plt.title(session)

    plt.xlabel('x [m]')
    plt.ylabel('y [m]')

    plot_orbslam2(session)

    plt.legend(loc="upper left")
    plt.savefig(f'plots/{session}_os2.png')
    # plt.show()

    # ==============================================================================
    plt.clf()
    plt.figure(session)
    plt.title(session)

    plt.xlabel('x [m]')
    plt.ylabel('y [m]')

    plot_orbslam3(session, scale = ORBSLAM3_SCALE)

    plot_cones(session, scale = ORBSLAM3_SCALE)

    plt.legend(loc="upper left")
    plt.savefig(f'plots/{session}_os3-cones.png')
    # plt.show()

    # ==============================================================================
    plt.clf()
    plt.figure(session)
    plt.title(session)

    plt.xlabel('x [m]')
    plt.ylabel('y [m]')

    plot_orbslam2(session)

    plot_orbslam3(session, scale = ORBSLAM3_SCALE)

    plt.legend(loc="upper left")
    plt.savefig(f'plots/{session}_os2-os3.png')
    # plt.show()
