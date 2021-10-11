# Q: Why do we need to validate velodyne points?
# A: Without time of each point, it is hard to undistort a scan. In previous velodyne driver,
#    only 'x y z intensity ring' are provided in rosbag file, so we need to compute time by xyz.
#    However, horizontal angle of a scan is usually over 360 degrees, making it impossible to
#    compute time when points are unorganized.
#    Finally, if the field 'time' absents in a bag file, we need to check if point cloud
#    is organized, or points in a ring satisfy CW or CCW order.
#
#    Attention: The field 'ring' is different from scan id in velodyne manual. For convenience,Velodyne driver
#               developer make ring increases with vertical angle.
#
# Q: What is the validation procedure?
# A: 1. check if field 'time' exists, return success while true;
#    2. check if ring is ordered by vertical angle;
#    3. check if all points in each ring are in CW order

import logging
import math
import sys
import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
                    datefmt='%m-%d:%H:%M:%S',
                    level=logging.DEBUG)
logger = logging.getLogger('logger')
logger.setLevel(logging.INFO)


def get_vertical_angle_in_degrees(p):
    return math.atan2(p[2], math.sqrt(p[0] * p[0] + p[1] * p[1])) * 180 / math.pi


def validate_msg(pc2_msg):
    dict_scan_id2points = {}

    # 1. check if field 'time' exists
    for p in pc2.read_points(pc2_msg, skip_nans=True, field_names=("x", "y", "z", "ring", "time")):
        if len(p) == 5:
            logger.info("Validate success: time field detected, frame is valid")
            return

        if len(p) < 4:
            logger.fatal("check failed: fields x, y, z, ring are required")

        ring = p[3]
        if ring in dict_scan_id2points:
            dict_scan_id2points[ring].append(p)
        else:
            dict_scan_id2points[ring] = [p]

    # 2. check if ring is ordered by vertical angle
    for ring in range(len(dict_scan_id2points) - 2):
        # get vertical angles of 3 ring's head point
        angles = [get_vertical_angle_in_degrees(dict_scan_id2points[i][0]) for i in range(ring, ring + 3)]
        if (angles[0] - angles[1]) * (angles[1] - angles[2]) <= 0:
            logger.fatal("ring is not ordered by vertical angles %s" % angles)
    logger.info("Validate success: ring is ordered by vertical angle")

    # 3. check if all points in each ring are in CW order
    for ring in dict_scan_id2points:
        ring_points = dict_scan_id2points[ring]
        for idx in range(len(ring_points) - 1):
            # less than 0 if points are in CCW order
            if np.cross(ring_points[idx][:2], ring_points[idx + 1][:2]) > 0:
                logger.fatal("not all points in ring %d are in CW/CCW order")
    logger.info("Validate success: all points in each ring are in CW order")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        logging.warning("Usage: validate_velodyne_cloud_in_bag.py /path/to/file.bag")
        sys.exit(1)

    bag_filename = sys.argv[1]

    logger.info("Reading bag file...")
    bag = rosbag.Bag(bag_filename, 'r')
    msg_gen = bag.read_messages(topics=[])

    check_first_n_frames = 10
    logger.info("Get first %s frames of type sensor_msgs/PointCloud2..." % check_first_n_frames)
    msgs = []
    for _, msg, _ in msg_gen:
        if check_first_n_frames == 0: break
        if not msg._type == 'sensor_msgs/PointCloud2': continue
        msgs.append(msg)
        check_first_n_frames = check_first_n_frames - 1

    logger.info("Validate frames...")
    for i in range(len(msgs) - 1):
        dt = msgs[i + 1].header.stamp.to_sec() - msgs[i].header.stamp.to_sec()
        validate_msg(msgs[i])
        pass

    bag.close()
