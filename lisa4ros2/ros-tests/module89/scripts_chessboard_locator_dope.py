#!/usr/bin/env python3

import cv2
import time
import numpy as np
import imutils, simplejson, os
import torch, math
from torch import nn
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from isaac_ros_nvengine_interfaces.msg import TensorList

kMapPeakThreshhold = 0.01
kInputMapsRow = 60
kInputMapsColumn = 80
kGaussianSigma = 3.0
kMinimumWeightSum = 1e-6
kOffsetDueToUpsampling = 0.4395

colors = [(255, 255, 255), (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

config = simplejson.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'camera_config.json')))
cameraMatrix = np.array(config['camera_matrix'], np.float32)
dist = np.array(config['dist'])

# NumPy     return (row, col) = (y, x)
# OpenCV    return (x, y) = (col, row)
def make_grid(tensor, nrow=8, padding=2, normalize=False, range_=None, scale_each=False, pad_value=0):
    """Make a grid of images.
    Args:
        tensor (Tensor or list): 4D mini-batch Tensor of shape (B x C x H x W)
            or a list of images all of the same size.
        nrow (int, optional): Number of images displayed in each row of the grid.
            The Final grid size is (B / nrow, nrow). Default is 8.
        padding (int, optional): amount of padding. Default is 2.
        normalize (bool, optional): If True, shift the image to the range (0, 1),
            by subtracting the minimum and dividing by the maximum pixel value.
        range (tuple, optional): tuple (min, max) where min and max are numbers,
            then these numbers are used to normalize the image. By default, min and max
            are computed from the tensor.
        scale_each (bool, optional): If True, scale each image in the batch of
            images separately rather than the (min, max) over all images.
        pad_value (float, optional): Value for the padded pixels.
    Example:
        See this notebook `here <https://gist.github.com/anonymous/bf16430f7750c023141c562f3e9f2a91>`_
    """
    if not (torch.is_tensor(tensor) or
            (isinstance(tensor, list) and all(torch.is_tensor(t) for t in tensor))):
        raise TypeError('tensor or list of tensors expected, got {}'.format(type(tensor)))

    # if list of tensors, convert to a 4D mini-batch Tensor
    if isinstance(tensor, list):
        tensor = torch.stack(tensor, dim=0)

    if tensor.dim() == 2:  # single image H x W
        tensor = tensor.view(1, tensor.size(0), tensor.size(1))
    if tensor.dim() == 3:  # single image
        if tensor.size(0) == 1:  # if single-channel, convert to 3-channel
            tensor = torch.cat((tensor, tensor, tensor), 0)
        tensor = tensor.view(1, tensor.size(0), tensor.size(1), tensor.size(2))

    if tensor.dim() == 4 and tensor.size(1) == 1:  # single-channel images
        tensor = torch.cat((tensor, tensor, tensor), 1)

    if normalize is True:
        tensor = tensor.clone()  # avoid modifying tensor in-place
        if range_ is not None:
            assert isinstance(range_, tuple), \
                "range has to be a tuple (min, max) if specified. min and max are numbers"

        def norm_ip(img, min, max):
            img.clamp_(min=min, max=max)
            img.add_(-min).div_(max - min + 1e-5)

        def norm_range(t, range_):
            if range_ is not None:
                norm_ip(t, range_[0], range_[1])
            else:
                norm_ip(t, float(t.min()), float(t.max()))

        if scale_each is True:
            for t in tensor:  # loop over mini-batch dimension
                norm_range(t, range)
        else:
            norm_range(tensor, range)

    if tensor.size(0) == 1:
        return tensor.squeeze()

    # make the mini-batch of images into a grid
    nmaps = tensor.size(0)
    xmaps = min(nrow, nmaps)
    ymaps = int(math.ceil(float(nmaps) / xmaps))
    height, width = int(tensor.size(2) + padding), int(tensor.size(3) + padding)
    grid = tensor.new(3, height * ymaps + padding, width * xmaps + padding).fill_(pad_value)
    k = 0
    for y in range(ymaps):
        for x in range(xmaps):
            if k >= nmaps:
                break
            grid.narrow(1, y * height + padding, height - padding) \
                .narrow(2, x * width + padding, width - padding) \
                .copy_(tensor[k])
            k = k + 1
    return grid

def get_image_grid(tensor, nrow=2, padding=2, mean=None, std=None):
    """
    Saves a given Tensor into an image file.
    If given a mini-batch tensor, will save the tensor as a grid of images.
    """
    from PIL import Image

    # tensor = tensor.cpu()
    grid = make_grid(tensor, nrow=nrow, padding=padding, pad_value=1)
    if not mean is None:
        # ndarr = grid.mul(std).add(mean).mul(255).byte().transpose(0,2).transpose(0,1).numpy()
        ndarr = grid.mul(std).add(mean).mul(255).byte().transpose(0, 2).transpose(0, 1).numpy()
    else:
        ndarr = grid.mul(0.5).add(0.5).mul(255).byte().transpose(0, 2).transpose(0, 1).numpy()
    im = Image.fromarray(ndarr)
    return im

def IsolateMaxima(img):
    mask = cv2.dilate(img, kernel=np.ones((3, 3)))
    mask = cv2.compare(src1=img, src2=mask, cmpop=cv2.CMP_GE) # CMP_GE: src1 is greater than or equal to src2
    # non_plateau_mask = np.zeros((60, 40, 1), dtype=np.float32)
    non_plateau_mask = cv2.erode(src=img,kernel=np.ones((3, 3)))
    non_plateau_mask = cv2.compare(src1=img, src2=non_plateau_mask, cmpop=cv2.CMP_GE) # CMP_GE: src1 is greater than or equal to src2
    cv2.bitwise_and(mask, non_plateau_mask, mask=mask)
    return mask

def FindPeaks(img, threshold=kMapPeakThreshhold):
    mask = IsolateMaxima(img)
    maxima = cv2.findNonZero(mask)  # OpenCV return (col, row)
    peaks = []
    for i in range(len(maxima)):
        [x, y] = maxima[i][0]
        if img[y, x] > threshold:   # NumPy use [row, col]
            peaks.append(maxima[i])
    return peaks
def FindObjects(maps):  # maps: Array of belief map (80, 60) np.float32
    all_peaks = []      # Vector2(x, y): location of peak   z: belief map value
    channel_peaks = []  # int[kNumVertexChannel]
    for chan in range(len(maps)):
        channel_peaks_buffer = []
        channel_peaks_score = []
        image = maps[chan]  # size = (kInputMapsRow, kInputMapsColumn) dtype=np.float32
        blurred = cv2.GaussianBlur(src=image,
                                   ksize=(0, 0),
                                   sigmaX=kGaussianSigma,
                                   sigmaY=kGaussianSigma,
                                   borderType=cv2.BORDER_DEFAULT)
        peaks = FindPeaks(blurred)
        for pp in range(len(peaks)):
            peak = peaks[pp][0]    # Peak pixel
# Compute the weighted average for localizing the peak, using a 5x5 window
#           ███████████████
#           ███████████████
#           ██████░░░██████
#           ███████████████
#           ███████████████
            peak_sum = [0, 0]
            weight_sum = 0
            for xx in range(-2, 3):
                for yy in range(-2, 3):
                    row = peak[0] + xx
                    col = peak[1] + yy
                    if col < 0 or col >= image.shape[1] or row < 0 or row >= image.shape[0]: continue
                    weight = image[row, col]
                    weight_sum += weight
                    peak_sum[0] += row * weight
                    peak_sum[1] += col * weight
            if image[peak[1], peak[0]] >= kMapPeakThreshhold:
                channel_peaks_score.append(image[peak[1], peak[0]])
                if weight_sum < kMinimumWeightSum:
                    channel_peaks_buffer.append((peak[0] + kOffsetDueToUpsampling,
                                                 peak[1] + kOffsetDueToUpsampling))
                else:
                    channel_peaks_buffer.append((peak_sum[0]/weight_sum + kOffsetDueToUpsampling,
                                                 peak_sum[1]/weight_sum + kOffsetDueToUpsampling))\
        # Single detection (Only maximum)
        if len(channel_peaks_score) > 0:
            # channel_peaks.append([channel_peaks_buffer[channel_peaks_score.index(max(channel_peaks_score))]])
            channel_peaks.append(channel_peaks_buffer)
        else: channel_peaks.append([])
        # Multiple detection
        # channel_peaks.append(channel_peaks_buffer)
    return channel_peaks
def normalize8(I):
    mn = I.min()
    mx = I.max()

    mx -= mn

    I = ((I - mn)/mx) * 255
    return I.astype(np.uint8)
def FindMax(maps):
    max_points = []
    for m in maps:
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv2.GaussianBlur(m, (3, 3), 0))
        max_points.append(max_loc)
    return max_points
def rvec2quat(rvec):
    cy = math.cos(rvec[2][0] * 0.5)
    sy = math.sin(rvec[2][0] * 0.5)
    cp = math.cos(rvec[1][0] * 0.5)
    sp = math.sin(rvec[1][0] * 0.5)
    cr = math.cos(rvec[0][0] * 0.5)
    sr = math.sin(rvec[0][0] * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q
class ChessboardDecoder(Node):
    def __init__(self):
        super().__init__('chessboard_dope_decoder')
        self.tensor_sub = self.create_subscription(TensorList, '/tensor_sub', self.tensor_listener_callback, 10)
        # self.frame_sub = self.create_subscription(Image, '/dope/input', self.image_listener_callback, 10)

        self.pose_pub = self.create_publisher(Pose, '/pose', 10)

        # self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.get_logger().info('Node started ...')
    # def image_listener_callback(self, data):
    #     self.frame = self.bridge.imgmsg_to_cv2(data)
    def tensor_listener_callback(self, tensor_list):
        tensor = tensor_list.tensors[0]
        # tensor_name = tensor.name           # output
        # tensor_shape = tensor.shape         # "isaac_ros_nvengine_interfaces.msg.TensorShape(rank=4, dims=[1, 25, 60, 80])"
        # tensor_data_type = tensor.data_type # 9: float32
        # tensor_strides = tensor.strides     # uint64[]  "array('Q', [480000, 19200, 320, 4])"
        tensor_data = tensor.data           # uint8[480000]
        # self.get_logger().info('Tensor Name: "%s"' % tensor_name)
        # self.get_logger().info('Tensor Shape: "%s"' % tensor_shape)
        # self.get_logger().info('Data Type: "%s"' % tensor_data_type)
        # self.get_logger().info('Tensor Stride: "%s"' % tensor_strides)
        # self.get_logger().info('Tensor Data: "%s"' % tensor_data)
        # self.get_logger().info('Tensor Data: "%d"' % len(tensor_data))
        # cv2.imshow("Camera", self.frame)

        # debug = False
        # if debug:
        #     tensor_data = np.array(tensor_data).view(dtype=np.float32).reshape((5, kInputMapsRow, kInputMapsColumn))
        #     t = torch.from_numpy(tensor_data)
        #     belief_imgs = []
        #     upsampling = nn.UpsamplingNearest2d(size=self.frame.shape[:2])
        #     in_img = (torch.tensor(self.frame).float() / 255.0)
        #     in_img *= 0.5

        #     for i in range(5):
        #         belief = t[i].clone()
        #         # Normalize image to (0, 1)
        #         belief -= float(torch.min(belief).item())
        #         belief /= float(torch.max(belief).item())
        #         belief = torch.clamp(belief, 0, 1).cpu()
        #         belief = upsampling(belief.unsqueeze(0).unsqueeze(0)).squeeze().squeeze().data
        #         belief = torch.cat([
        #             belief.unsqueeze(0) + in_img[:, :, 0],
        #             belief.unsqueeze(0) + in_img[:, :, 1],
        #             belief.unsqueeze(0) + in_img[:, :, 2]
        #         ]).unsqueeze(0)
        #         belief = torch.clamp(belief, 0, 1)
        #         belief_imgs.append(belief.data.squeeze().numpy())
        #     # Create the image grid
        #     belief_imgs = torch.tensor(np.array(belief_imgs))
        #     im_belief = np.asarray(get_image_grid(belief_imgs, mean=0, std=1)) # PIL -> OpenCV
        #     cv2.imshow("GRID", imutils.resize(im_belief, height=480))
        #     cv2.waitKey(1)

        # Convert uint8[4] -> float32 [ALL]
        # tensor_np = np.array(tensor_data).view(dtype=np.float32).reshape((25, 60, 80, 1))

        # Convert uint8[4] -> float32 [4 CORNERS]
#       ░░░ = Black, ███ = White
#     0 ░░░░░░░░░░░░░░░░░░░░░░░░ 2
#       ░░░███░░░███░░░███░░░███
#       ███░░░███░░░███░░░███░░░
#       ░░░███░░░███░░░███░░░███
#       ███░░░███░░4███░░░███░░░
#       ░░░███░░░███░░░███░░░███
#       ███░░░███░░░███░░░███░░░
#       ░░░███░░░███░░░███░░░███
#       ███░░░███░░░███░░░███░░░
#     1 ████████████████████████ 3
        maps = []
        for i in range(5):
            stride = kInputMapsRow * kInputMapsColumn * 4   # 4 = sizeof(float)
            offset = stride * i
            # Slice tensor & convert uint8[4] -> float32
            maps.append(np.array(tensor_data[offset:offset+stride]).view('<f4').reshape((kInputMapsRow, kInputMapsColumn)))
        # objs = FindObjects(maps)
        # self.get_logger().info('Object: "%s"' % str([len(obj) for obj in objs]))

        # canvas = self.frame.copy()
        # for i in range(5):
        #     peak_list = objs[i]
        #     for point in peak_list:
        #         cv2.circle(canvas, (int(point[0]*8), int(point[1]*8)), 3, colors[i%len(colors)], -1)
        # cv2.imshow("Keypoints", canvas)
        # canvas1 = np.hstack([maps[0], maps[1]])
        # canvas2 = np.hstack([maps[2], maps[3]])
        # canvas = np.vstack([canvas1, canvas2])
        # cv2.imshow("Belief Map", imutils.resize(normalize8(canvas), height=480))
        # cv2.waitKey(1)


        points = FindMax(maps)
        # self.get_logger().info(str(points))
        # obj_points = np.array([[-0.2, 0.23, 0], [-0.2, -0.23, 0], [0.2, 0.23, 0], [0.2, -0.23, 0], [0, 0, 0]])
        # img_points = np.array([points[0], points[1], points[2], points[3], points[4]], dtype=np.double)*8
        obj_points = np.array([[-0.2, 0.23, 0], [-0.2, -0.23, 0], [0.2, 0.23, 0], [0.2, -0.23, 0]])
        img_points = np.array([points[0], points[1], points[2], points[3]], dtype=np.double) * 8

        ret, rvec, tvec = cv2.solvePnP(objectPoints=obj_points, imagePoints=img_points, cameraMatrix=cameraMatrix, distCoeffs=dist, flags=0)

        # canvas = self.frame.copy()
        # cv2.aruco.drawAxis(image=canvas, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1)
        # cv2.imshow('A', canvas)
        # cv2.waitKey(1)

        pose_msg = Pose()
        pose_msg.position = Point(x=tvec[0][0],
                                  y=tvec[1][0],
                                  z=tvec[2][0])
        r = R.from_matrix(cv2.Rodrigues(rvec)[0])
        rvec = Quaternion()
        [rvec.x, rvec.y, rvec.z, rvec.w] = r.as_quat()
        pose_msg.orientation = rvec
        self.pose_pub.publish(pose_msg)




def main():
    rclpy.init()
    chessboard_decoder = ChessboardDecoder()
    rclpy.spin(chessboard_decoder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()