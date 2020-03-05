# Get PCL from frame function
def get_pcl_from_frame(data, frame, points):
    points_step = int(1. / points)
    data_range = range(0, data[frame].shape[0], points_step)
    xyz_values = data[frame][data_range, :]
    return xyz_values