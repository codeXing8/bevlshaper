# Import dependencies
from pcl_data import *
from pcl_filter import *
from pcl_plot import *
from adaptive_segmentation import *
import sys

# Constants
MAX_RADIUS = 1
POINTS_RATIO = 0.1

# Origin mask
ORIGIN_X = 0
ORIGIN_Y = 0
ORIGIN_CIRCLE_RADIUS = 6.1

# Full lane mask
PATCH_X = -2
PATCH_Y = -1.5
PATCH_WIDTH = 40
PATCH_HEIGHT = 12

# Debug values
COLORS = ['green', 'blue', 'yellow']

# Set parameters
alpha = 0.8
beta = 0.6
min_cluster_len = 20

# Choose input data
# date = '2011_09_26'
# drive = '0001'
date, drive = sys.argv[1], sys.argv[2]

# Load dataset
dataset = load_dataset(date, drive)

# Load tracklets (rects and types)
tracklet_rects, tracklet_types = load_tracklets_for_frames(len(list(dataset.velo)), 'data/{}/{}_drive_{}_sync/tracklet_labels.xml'.format(date, date, drive))

# Redefine dataset
dataset = list(dataset.velo)

# Init empty filenames array
filenames = []
# Select frame
for frame in range(len(dataset)):
    # Log
    print('\n', "Frame:", frame)

    # Get point cloud data
    velo_frame = get_pcl_from_frame(dataset, frame, POINTS_RATIO)

    # Filter point cloud
    velo_frame = filter_ground_plane(velo_frame)
    velo_frame = apply_circular_mask(velo_frame, [ORIGIN_X, ORIGIN_Y], ORIGIN_CIRCLE_RADIUS)
    velo_frame = apply_rectangular_mask(velo_frame, [PATCH_X, PATCH_Y], PATCH_WIDTH, PATCH_HEIGHT)

    # Check if points are left after filter
    if not velo_frame.any():
        continue

    # Get points from filtered point cloud
    pcl = list(np.asarray(velo_frame[:, [0, 1]]))

    # Adaptively segment PCL data
    clusters = cluster_tree(pcl, alpha, beta, min_cluster_len, MAX_RADIUS)

    # Log
    print("Number of clusters:", len(clusters))

    # Determine L-shapes from clusters
    # Init empty array for L-shape points
    lpoints, lpoints_polar = [], []
    # Set angular step parameter
    delta = 0.02
    # Init array for L-shape heading angle over time
    for cluster in clusters:
        # Get rectangle parameters from rectangle fit search
        rectangle_points = search_rectangle_fit(cluster, delta)
        # Get closest L-shape to vehicle from rectangle
        lshape_points = get_closest_lshape(rectangle_points, np.asarray([ORIGIN_X, ORIGIN_Y]))
        lpoints.append(lshape_points)
        # lpoints_polar.append(convert_lshape_to_polar(lshape_points))

    # Print raw L-shape points
    print("Raw L-shape points:", lpoints)
    # print("Raw L-shape points (polar):", lpoints_polar)

    # Render 2D BEV scene with colorized clusters and fitted L-shapes/rectangles
    # filenames += [render_2Dbev(frame, velo_frame, clusters, lpoints, POINTS_RATIO, COLORS, ORIGIN_X, ORIGIN_Y)]
    filenames += [draw_3Dplot_with_lshapes(frame, dataset, clusters, lpoints, tracklet_rects, tracklet_types, POINTS_RATIO)]

# Render .gif file from scene
render_scene_gif(filenames, fps=5)