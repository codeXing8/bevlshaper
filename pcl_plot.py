# Import dependencies
from exploreKITTI.explore_kitti_lib import *
import matplotlib.pyplot as plt
import numpy as np

# Constants
Z_GROUND_PLANE = -1.1

# Render 2D bird's-eye-view scene from PCL data
def render_2Dbev(frame, data, clusters, lpoints, points, colors, origin_x, origin_y):
    # Init figure, define points
    f = plt.figure(figsize=(12, 8))
    axis = f.add_subplot(111, xticks=[], yticks=[])
    point_size = 0.01 * (1. / points)

    # Extract coordinate ranges
    # print(data)
    xy_values = data[:, [0, 1]]
    # z_values = data[:, 3]
    # print(xy_values)

    # Draw scatter plot
    axis.scatter(*np.transpose(xy_values), s=point_size, c='black', cmap='gray')
    axis.set_xlim(*axes_limits[0])
    axis.set_ylim(*axes_limits[1])

    # Draw cluster points
    cnt = 0
    for cluster in clusters:
        cntc = np.mod(cnt, len(colors))
        axis.scatter(*np.transpose(cluster), s=point_size, c=colors[cntc], cmap='gray')
        cnt += 1

    # Draw L-shapes
    for lpoints_cluster in lpoints:
        # lp1, lp2, lp3, lp4 = lpoints_cluster[0], lpoints_cluster[1], lpoints_cluster[2], lpoints_cluster[3]
        lp1, lp2, lp3 = lpoints_cluster[0], lpoints_cluster[1], lpoints_cluster[2]
        axis.plot([lp2[0], lp1[0]], [lp2[1], lp1[1]], 'red')
        axis.plot([lp2[0], lp3[0]], [lp2[1], lp3[1]], 'red')
        axis.scatter(lp1[0], lp1[1], s=20, facecolors='none', edgecolors='r')
        axis.scatter(lp2[0], lp2[1], s=20, facecolors='none', edgecolors='r')
        axis.scatter(lp3[0], lp3[1], s=20, facecolors='none', edgecolors='r')
        # axis.plot([lp3[0], lp4[0]], [lp3[1], lp4[1]], 'red')
        # axis.plot([lp4[0], lp1[0]], [lp4[1], lp1[1]], 'red')

    # Draw vehicle center
    axis.scatter(origin_x, origin_y, s=40, facecolors='r', edgecolors='r')

    # Save frame and close plot
    filename = 'frames/frame_{0:0>4}.png'.format(frame)

    # Save figure and end function
    plt.savefig(filename)
    plt.close(f)
    return filename


# Draw 3D plot function
def draw_3Dplot_with_lshapes(frame, dataset, clusters, lpoints, tracklet_rects, tracklet_types, points_ratio):
    # Init figure
    f = plt.figure(figsize=(12, 8))
    # Add 3D subplot
    axis = f.add_subplot(111, projection='3d', xticks=[], yticks=[], zticks=[])
    # Plot
    points_step = int(1. / points_ratio)
    point_size = 0.01 * (1. / points_ratio)
    # Calc range of frames
    velo_range = range(0, dataset[frame].shape[0], points_step)
    # Reduce dataset to relevant frames
    velo_frame = dataset[frame][velo_range, :]

    # Create 3D scatter plot for point cloud
    # print(*np.transpose(velo_frame[:, [0, 1, 2]]).shape)
    axis.scatter(*np.transpose(velo_frame[:, [0, 1, 2]]), s=point_size, c=velo_frame[:, 3], cmap='gray')
    axis.set_xlim3d(*axes_limits[0])
    axis.set_ylim3d(*axes_limits[1])
    axis.set_zlim3d(*axes_limits[2])

    # Draw tracklets
    for t_rects, t_type in zip(tracklet_rects[frame], tracklet_types[frame]):
        draw_box(axis, t_rects, axes=[0, 1, 2], color=colors[t_type])

    # Draw clusters

    # Draw L-shapes
    for lshape in lpoints:
        axis.plot([lshape[1][0], lshape[0][0]], [lshape[1][1], lshape[1][1]], [Z_GROUND_PLANE, Z_GROUND_PLANE], 'blue')
        axis.plot([lshape[1][0], lshape[2][0]], [lshape[1][1], lshape[2][1]], [Z_GROUND_PLANE, Z_GROUND_PLANE], 'blue')
        axis.scatter(*np.array([lshape[0][0], lshape[0][1], Z_GROUND_PLANE]).reshape(3, 1), s=10, c='blue')
        axis.scatter(*np.array([lshape[1][0], lshape[1][1], Z_GROUND_PLANE]).reshape(3, 1), s=10, c='blue')
        axis.scatter(*np.array([lshape[2][0], lshape[2][1], Z_GROUND_PLANE]).reshape(3, 1), s=10, c='blue')

    # Define filename
    filename = 'frames/frame_{0:0>4}.png'.format(frame)
    # plt.show()
    plt.savefig(filename)
    plt.close(f)
    return filename


# Render scene as .gif file function
def render_scene_gif(filenames, fps):
    # Render .gif file
    clip = ImageSequenceClip(filenames, fps=fps)
    clip.write_gif('pcl_lshapes.gif', fps=fps)
    return 0