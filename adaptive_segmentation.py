# Load dependencies
from exploreKITTI.utilities import *
import numpy as np


# Find all points in range function
def find_all_points_in_range(p, r, pcl):
    p_in_range = []
    for n in range(len(pcl)):
        p_tmp = pcl[n]
        if not np.array_equal(p_tmp, p) and np.linalg.norm(p_tmp - p) <= r:
            p_in_range.append(p_tmp)
    return np.asarray(p_in_range)


# Ignore points from point cloud
def ignore_points_from_array(pcl, points):
    pcl_subset = pcl
    for point in points:
        ind = 0
        size = len(pcl_subset)
        while ind != size and not np.array_equal(pcl_subset[ind], point):
            ind += 1
        if ind != size:
            pcl_subset_tmp = []
            for i in range(len(pcl_subset)):
                if i != ind:
                    pcl_subset_tmp.append(pcl_subset[i])
            pcl_subset = pcl_subset_tmp
    return pcl_subset


# Better isin function
def better_isin(points, point):
    if len(points) == 0:
        return False
    else:
        for point_ref in points:
            if np.array_equal(point_ref, point):
                return True


# Cluster K-D tree function
def cluster_tree(pcl, a, b, min_cluster_len, max_radius):
    # Init empty set of clusters
    clusters = []
    # Init empty list of checked points
    checked_p = []
    # Iterate over all points p
    # for n in range(len(pcl)):
    n = 0
    while n < len(pcl):
        # Print progress
        print("Iteration", n, "out of", len(pcl))
        # Select point p
        p = pcl[n]
        # If point p was not considered yet, continue
        # if not better_isin(checked_p, p):
        cluster = [p]
        # Iterate along cluster
        k = 0
        # Iterate over cluster while it is created to extend it
        while k < len(cluster):
            # Print progress
            print_progress(k + 1, len(cluster))
            # Attach iterate over cluster, select last added point
            p_tmp = cluster[k]
            # Set adaptive radius
            r = np.amin([a * np.linalg.norm(p_tmp) + b, max_radius])
            # Select all points in range r from current point p
            pcl = ignore_points_from_array(pcl, cluster)
            ps_in_range = find_all_points_in_range(p_tmp, r, pcl)
            # Check if any point is in range
            if ps_in_range.size > 0:
                # Extend current cluster by new point
                cluster = np.concatenate((cluster, ps_in_range), axis=0)
            k += 1
        # Append complete cluster to set of clusters
        if len(cluster) >= min_cluster_len:
            clusters.append(cluster)
        n += 1
    # Return set of clusters
    return np.asarray(clusters)


# Project edge points on angle function
def project_edge_points_on_angle(cluster, theta):
    # Define edge vectors
    l1 = np.asarray([np.cos(theta), np.sin(theta)])
    l2 = np.asarray([-np.sin(theta), np.cos(theta)])
    # Iterate over points in single cluster and calc projections c along l
    # c1, c2 = [], []
    # for point in cluster:
        # Add calculated projections to arrays
        # c1.append(np.asarray([point[0] * l1[0], point[1] * l1[1]]))
        # c2.append(np.asarray([point[0] * l2[0], point[1] * l2[1]]))
    # Calc projection via matrix multiplication/scalar product
    c1 = np.matmul(np.asarray(cluster), l1)
    c2 = np.matmul(np.asarray(cluster), l2)
    # Return projections
    return [c1, c2]


# Calculate closeness (criterion) function
def calculate_closeness(c1, c2, minimum_distance):
    # Get max projection from both sets
    c1_max, c2_max = np.amax(c1), np.amax(c2)
    c1_min, c2_min = np.amin(c1), np.amin(c2)
    # Init empty index arrays
    d1_max, d1_min, d2_max, d2_min, d1, d2 = [], [], [], [], [], []
    for c1_val, c2_val in zip(c1, c2):
        # Calculate distance vectors containing all distances between the boundaries and each point
        d1_max.append(c1_max - c1_val)
        d1_min.append(c1_val - c1_min)
        d2_max.append(c2_max - c2_val)
        d2_min.append(c2_val - c2_min)
    # Choose distance vector with smaller overall distances
    if np.linalg.norm(d1_max) > np.linalg.norm(d1_min):
        d1 = d1_min
    else:
        d1 = d1_max
    if np.linalg.norm(d2_max) > np.linalg.norm(d2_min):
        d2 = d2_min
    else:
        d2 = d2_max
    # Init with zero quality
    quality = 0
    for n in range(len(d1)):
        # Choose smallest distance from d1, d2
        d = np.amax([np.amin([d1[n], d2[n]]), minimum_distance])
        # Increase quality, quality increases faster if distances d are small
        quality = quality + 1 / d
    # Return quality
    return quality


# Search-based rectangle fitting function
def search_rectangle_fit(cluster, delta):
    # Init array for calculated qualities
    qualities = []
    # Workaround: Range function does not work with floats
    # Calc number of steps between 0 and 90 deg. with chosen step width
    steps_theta = round(np.pi / (2 * delta))
    # Iterate over all steps, choose based on step number
    for num_theta in range(steps_theta + 1):
        # Choose
        theta = np.amin([num_theta * delta, np.pi / 2])
        c1, c2 = project_edge_points_on_angle(cluster, theta)
        # Calc quality of fit considering closeness as criterion
        quality = calculate_closeness(c1, c2, 0.01)
        qualities.append(np.asarray([quality, theta]))
    # Find angle with highest quality of fit
    theta_max = np.transpose(np.asarray(qualities))[1][np.argmax(np.transpose(np.asarray(qualities))[0])]
    # Get related projection
    c1_max, c2_max = project_edge_points_on_angle(cluster, theta_max)
    # Get rectangle parameters based on highest-quality angle
    c1, c2 = np.amin(c1_max), np.amin(c2_max)
    c3, c4 = np.amax(c1_max), np.amax(c2_max)
    # Rectangle parameter a1, b2, a3, b4
    v1 = np.cos(theta_max)
    # Rectangle parameter b1, b3
    v2 = np.sin(theta_max)
    # Rectangle parameter a2, a4
    v3 = -np.sin(theta_max)
    # Rectangle parameter form: a1, a2, a3, a4, b1, b2, b3, b4, c1, c2
    # print("Rectangle parameters:", c1, c2, c3, c4, v1, v2, v3)
    # Calc intersection points of lines represented by rectangle parameters
    p1 = calc_intersection_point(v1, v2, c1, v3, v1, c2)
    p2 = calc_intersection_point(v3, v1, c2, v1, v2, c3)
    p3 = calc_intersection_point(v1, v2, c3, v3, v1, c4)
    p4 = calc_intersection_point(v3, v1, c4, v1, v2, c1)
    # print("Rectangle fit:", p1, p2, p3, p4)
    return [p1, p2, p3, p4]


# Calculate intersection point function
def calc_intersection_point(a1, b1, c1, a2, b2, c2):
    if np.abs(b1) < 0.01:
        b1 = -0.01
    if np.abs(b2) < 0.01:
        b2 = -0.01
    x = (c1 / b1 - c2 / b2) * np.power(a1 / b1 - a2 / b2, -1)
    y = (c1 - a1 * x) / b1
    return np.asarray([x, y])


# Get closest L-shape (towards observer) function
def get_closest_lshape(points, p_origin):
    # Get points
    p1, p2, p3, p4 = points[0], points[1], points[2], points[3]
    # Get maximum over all norms between observer and point
    norms_towards_observer = np.asarray([
        np.linalg.norm(p1 - p_origin),
        np.linalg.norm(p2 - p_origin),
        np.linalg.norm(p3 - p_origin),
        np.linalg.norm(p4 - p_origin)
    ])
    # Calculate maximum norm along point norms
    max_norm = np.amax(norms_towards_observer)
    # Select closest points towards observer, reorder x1, y1, x2, y2, x3, y3 (and discard x4, y4)
    if np.linalg.norm(p1 - p_origin) >= max_norm:
        p1 = p4
    elif np.linalg.norm(p2 - p_origin) >= max_norm:
        p2 = p4
    elif np.linalg.norm(p3 - p_origin) >= max_norm:
        p3 = p4
    # Reshape points
    closest_points = [p1, p2, p3]
    # Return 3 points in order: closest to inner, inner, furthest to inner
    inner_point = closest_points[np.argmin(np.asarray([
        np.linalg.norm(p1 - p2) + np.linalg.norm(p1 - p3),
        np.linalg.norm(p2 - p1) + np.linalg.norm(p2 - p3),
        np.linalg.norm(p3 - p1) + np.linalg.norm(p3 - p2)
    ]))]
    if np.array_equal(inner_point, p1):
        if np.linalg.norm(p2 - inner_point) >= np.linalg.norm(p3 - inner_point):
            furthest_point = p2
            closest_point = p3
        else:
            furthest_point = p3
            closest_point = p2
    elif np.array_equal(inner_point, p2):
        if np.linalg.norm(p3 - inner_point) >= np.linalg.norm(p1 - inner_point):
            furthest_point = p3
            closest_point = p1
        else:
            furthest_point = p1
            closest_point = p3
    else:
        if np.linalg.norm(p2 - inner_point) >= np.linalg.norm(p1 - inner_point):
            furthest_point = p2
            closest_point = p1
        else:
            furthest_point = p1
            closest_point = p2
    # Return points
    return [furthest_point, inner_point, closest_point]


# Angle between vectors function
def angle_between_vectors(u, v):
    cos_theta = np.dot(u, v) / np.linalg.norm(u) / np.linalg.norm(v)
    theta = np.arccos(cos_theta)
    return theta


# Convert L-shape to polar coordinates function
def convert_lshape_to_polar(lshape_points, orientation_vector=np.asarray([0, -1])):
    # Define inner point of corner point
    p0 = lshape_points[1]
    l1 = np.linalg.norm(lshape_points[1] - lshape_points[0])
    l2 = np.linalg.norm(lshape_points[1] - lshape_points[2])
    theta = angle_between_vectors(lshape_points[1] - lshape_points[0], orientation_vector)
    return [p0, [l1, l2, theta]]