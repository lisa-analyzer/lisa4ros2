#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python
import imutils
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, String, UInt8MultiArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from sklearn.cluster import KMeans, AgglomerativeClustering, DBSCAN
from module89.msg import ChessboardImgPose
from module89.srv import ClusterLock

import cv2
import math, random
import json
import pyquaternion as q
import tensorflow as tf

from transform import four_point_transform, order_points, poly2view_angle

import os
import numpy as np

## Avoid to use all GPU(s)VRAM
gpus = tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus: tf.config.experimental.set_memory_growth(gpu, True)

colors = [(255, 255, 255), (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

camera_config = json.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'camera_config.json')))
model_config = json.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'model_config.json')))
cameraMatrix = np.array(camera_config['camera_matrix'], np.float32)
dist = np.array(camera_config['dist'])
export_size = 224
chess_piece_height = {"king": (0.081, 0.097), "queen": (0.07, 0.0762), "bishop": (0.058, 0.065), "knight": (0.054, 0.05715), "rook": (0.02845, 0.048), "pawn": (0.043, 0.045)}
chess_piece_diameter = {"king": (0.028, 0.0381), "queen": (0.028, 0.0362), "bishop": (0.026, 0.032), "knight": (0.026, 0.03255), "rook": (0.026, 0.03255), "pawn": (0.0191, 0.02825)}
mask_contour_index_list = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
scan_box_height = min(chess_piece_height['king'])

cv_bridge = CvBridge()
def pose2view_angle(rvec, tvec):
    rotM = np.zeros(shape=(3, 3))
    rotM, _ = cv2.Rodrigues(rvec, rotM, jacobian=0)
    tvec_tile_final = np.dot(tvec, rotM.T).reshape(3)
    tile_x, tile_y, tile_z = tvec_tile_final[0], tvec_tile_final[1], tvec_tile_final[2]
    angle_rad = math.asin((math.sqrt(tile_x ** 2 + tile_y ** 2)) / (math.sqrt(tile_x ** 2 + tile_y ** 2 + tile_z ** 2)))
    return angle_rad
def getBox2D(rvec, tvec, size = 0.05, height = scan_box_height, only_base=False):
    objpts = np.float32([[0, 0, 0], [size, 0, 0], [size, size, 0], [0, size, 0], [0, 0, height], [size, 0, height], [size, size, height], [0, size, height]]).reshape(-1, 3)
    if only_base: objpts = objpts[mask_contour_index_list[0]]
    imgpts, jac = cv2.projectPoints(objpts, rvec, tvec, cameraMatrix, dist)
    min_x = int(min(imgpts, key=lambda x: x[0][0]).ravel()[0])
    max_x = int(max(imgpts, key=lambda x: x[0][0]).ravel()[0])
    min_y = int(min(imgpts, key=lambda x: x[0][1]).ravel()[1])
    max_y = int(max(imgpts, key=lambda x: x[0][1]).ravel()[1])
    return (min_x, min_y), (max_x, max_y)
def getValidContour2D(rvec, tvec, size = 0.05, height = scan_box_height, only_base=False):
    objpts = np.float32([[0, 0, 0], [size, 0, 0], [size, size, 0], [0, size, 0], [0, 0, height], [size, 0, height], [size, size, height], [0, size, height]]).reshape(-1, 3)
    imgpts, jac = cv2.projectPoints(objpts, rvec, tvec, cameraMatrix, dist)
    valid_contours = []
    if only_base: valid_contours.append([imgpts[mask_contour_index_list[0][i]] for i in range(len(mask_contour_index_list[0]))])
    else:
        for mask_contour_index in mask_contour_index_list:
            valid_contours.append([imgpts[mask_contour_index[i]] for i in range(len(mask_contour_index))])
    return valid_contours
def getPoly2D(rvec, tvec, size = 0.05):
    objpts = np.float32([[0, 0, 0], [size, 0, 0], [size, size, 0], [0, size, 0]]).reshape(-1, 3)
    imgpts, jac = cv2.projectPoints(objpts, rvec, tvec, cameraMatrix, dist)
    return imgpts
def llr_tile(rvec, tvec, only_base = False):
    rotM = np.zeros(shape=(3, 3))
    rotM, _ = cv2.Rodrigues(rvec, rotM, jacobian=0)
    ### Draw chess piece space ###
    counter = 0
    tile_volume_bbox_list, angle_list, valid_contours_list = [], [], []
    for y in range(3, -5, -1):
        for x in range(-4, 4):
            board_coordinate = np.array([x * 0.05, y * 0.05, 0.0])
            (min_x, min_y), (max_x, max_y) = getBox2D(rvec, tvec + np.dot(board_coordinate, rotM.T), size=0.05, height=scan_box_height, only_base=only_base)
            tile_volume_bbox_list.append([(min_x, min_y), (max_x, max_y)])

            # find angle of each tile
            translated_tvec = tvec + np.dot(board_coordinate, rotM.T)
            poly_tile = getPoly2D(rvec, translated_tvec, size=0.05)
            valid_contours = getValidContour2D(rvec, translated_tvec, size=0.05, height=scan_box_height, only_base=only_base)
            valid_contours_list.append(valid_contours)
            angle_rad = pose2view_angle(rvec, tvec)
            # angle_rad = poly2view_angle(poly_tile)
            angle_deg = angle_rad / 3.14 * 180
            angle_list.append(angle_deg)
            counter += 1
    tile_volume_bbox_list_new, angle_list_new = [], []
    for i in range(64):
        y, x = 7 - int(i / 8), i % 8
        tile_volume_bbox_list_new.append(tile_volume_bbox_list[8*y+x])
        angle_list_new.append(angle_list[8*y+x])
    return tile_volume_bbox_list, angle_list, valid_contours_list
def llr_tile_top(rvec, tvec):
    rotM = np.zeros(shape=(3, 3))
    rotM, _ = cv2.Rodrigues(rvec, rotM, jacobian=0)
    ### Draw chess piece space ###
    counter = 0
    poly_tile_list = []
    for y in range(3, -5, -1):
        for x in range(-4, 4):
            board_coordinate = np.array([x * 0.05, y * 0.05, 0.0])
            # find angle of each tile
            translated_tvec = tvec + np.dot(board_coordinate, rotM.T)
            poly_tile = getPoly2D(rvec, translated_tvec, size=0.05)
            poly_tile_list.append(poly_tile)
    return poly_tile_list
def getCNNinput(img, bbox_list, valid_contours_list):
    CNNinputs = []
    for i in range(len(bbox_list)):
        [(min_x, min_y), (max_x, max_y)] = bbox_list[i]
        if min_x < 0: min_x = 0
        if min_y < 0: min_y = 0
        if max_x >= img.shape[1]: max_x = img.shape[1]-1
        if max_y >= img.shape[0]: max_y = img.shape[0]-1
        cropped = img[min_y:max_y, min_x:max_x].copy()
        valid_contours = valid_contours_list[i]
        mask = np.zeros(cropped.shape[:2], dtype="uint8")
        for valid_contour in valid_contours:
            local_valid_contour = []
            for point in valid_contour:
                x = int(point[0][0] - min_x)
                y = int(point[0][1] - min_y)
                local_valid_contour.append([x, y])
            local_valid_contour = np.array(local_valid_contour).reshape((-1, 1, 2)).astype(np.int32)
            cv2.drawContours(mask, [local_valid_contour], -1, 255, -1)
        CNNinputs.append(cv2.bitwise_and(cropped, cropped, mask=mask))
    return CNNinputs
def resize_and_pad(img, size=300, padding_color=(0,0,0)):
    old_size = img.shape[:2]
    ratio = float(size) / max(old_size)
    new_size = tuple([int(x * ratio) for x in old_size])
    img = cv2.resize(img, (new_size[1], new_size[0]))
    delta_w = size - new_size[1]
    delta_h = size - new_size[0]
    top, bottom = delta_h // 2, delta_h - (delta_h // 2)
    left, right = delta_w // 2, delta_w - (delta_w // 2)
    return cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=padding_color)
def get_tile(img, rvec, tvec, only_base = False):
    tile_volume_bbox_list, angle_list, valid_contours_list = llr_tile(rvec, tvec, only_base=only_base)
    CNNinputs = getCNNinput(img, tile_volume_bbox_list, valid_contours_list)
    CNNinputs_padded = []
    for i in range(64):
        CNNinput_padded = resize_and_pad(CNNinputs[i], size=export_size)
        CNNinputs_padded.append(CNNinput_padded)
    return CNNinputs_padded, angle_list
def get_tile_top(img, rvec, tvec):
    poly_tile_list = llr_tile_top(rvec, tvec)
    CNNinputs = []
    pts2 = np.float32([(0, export_size-1), (export_size-1, export_size-1), (export_size-1, 0), (0, 0)])
    for poly_tile in poly_tile_list:
        M = cv2.getAffineTransform(np.float32(poly_tile).reshape((4, 2))[:3], pts2[:3])
        CNNinputs.append(cv2.warpAffine(img, M, (export_size, export_size)))
    return CNNinputs
def get_tile_ImgPose(img_pose: ChessboardImgPose, only_base = False):
    tvec = img_pose.pose.position
    tvec = np.array([tvec.x, tvec.y, tvec.z])
    rvec = img_pose.pose.orientation
    rvec = q.Quaternion(x=rvec.x, y=rvec.y, z=rvec.z, w=rvec.w)  # Convert to PyQuaternion object
    img = cv_bridge.imgmsg_to_cv2(img_pose.image, desired_encoding='passthrough')
    rvec, _ = cv2.Rodrigues(rvec.rotation_matrix, jacobian=0)
    # cv2.aruco.drawAxis(image=img, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1)
    # cv2.imshow("BBB", img)
    # cv2.waitKey(1)
    return get_tile(img, rvec, tvec, only_base=only_base)
def get_tile_top_ImgPose(img_pose: ChessboardImgPose):
    tvec = img_pose.pose.position
    tvec = np.array([tvec.x, tvec.y, tvec.z])
    rvec = img_pose.pose.orientation
    rvec = q.Quaternion(x=rvec.x, y=rvec.y, z=rvec.z, w=rvec.w)  # Convert to PyQuaternion object
    img = cv_bridge.imgmsg_to_cv2(img_pose.image, desired_encoding='passthrough')
    rvec, _ = cv2.Rodrigues(rvec.rotation_matrix, jacobian=0)
    # cv2.aruco.drawAxis(image=img, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1)
    # cv2.imshow("BBB", img)
    # cv2.waitKey(1)
    return get_tile_top(img, rvec, tvec)
symbol_dict = {1:'b', 2:'k', 3:'n', 4:'p', 5:'q', 6:'r'}
def to_FEN(board, color_array):
    global symbol_dict
    FEN_string = ""
    for y in range(8):
        empty = 0
        for x in range(8):
            if board[y][x] == 0: empty+= 1
            else:
                if empty != 0:
                    FEN_string += str(empty)
                    empty = 0
                char = symbol_dict[board[y][x]]
                FEN_string += char.upper() if color_array[y][x] == 1 else char.lower()
        if empty != 0: FEN_string += str(empty)
        FEN_string += '/'
    return FEN_string[:-1]  # exclude lastest '/'

clustering_mode = {0:''}

class ChessboardClassifier(Node):
    def __init__(self):
        super().__init__('chessboard_classifier')
        # Subscribe chessboard poses
        self.chessboard_pose_top_sub = self.create_subscription(ChessboardImgPose, '/chessboard/top/ImgPose', self.chessboard_pose_top_callback, 10)
        self.chessboard_pose_side_sub = self.create_subscription(ChessboardImgPose, '/chessboard/side/ImgPose', self.chessboard_pose_side_callback, 10)

        self.fen_pub = self.create_publisher(String, 'chessboard/fen', 10)
        self.fen_binary_pub = self.create_publisher(UInt8MultiArray, '/chessboard/fen_binary', 10)
        self.fen_color_pub = self.create_publisher(UInt8MultiArray, '/chessboard/fen_color', 10)
        self.top_cnn_viz_pub = self.create_publisher(Image, '/viz/top_cnn', 10)
        self.side_cnn_viz_pub = self.create_publisher(Image, '/viz/side_cnn', 10)
        self.cluster_viz_pub = self.create_publisher(Image, '/viz/cluster', 10)

        self.lock_srv = self.create_service(ClusterLock, 'command_cluster_lock', self.cluster_lock_callback)

        self.bridge = CvBridge()  # Bridge between "CV (NumPy array)" <-> "ROS sensor_msgs/Image"

        # Load Neural Network model
        self.top_model = tf.keras.models.load_model(os.path.join(model_config['base_path'], model_config['top_classifier']))
        self.side_model = tf.keras.models.load_model(os.path.join(model_config['base_path'], model_config['side_classifier']))
        self.color_model = tf.keras.models.load_model(os.path.join(model_config['base_path'], model_config['color_classifier']))
        self.top_model.summary()
        self.side_model.summary()
        self.color_model.summary()

        self.board_result_binary = np.zeros((8, 8), dtype=np.uint8)
        self.board_result_binary_buffer = []    # store history of self.board_result_binary
        self.board_result = np.zeros((8, 8), dtype=np.uint8)
        self.board_result_buffer = []           # store history of self.board_result
        self.board_result_color = np.zeros((8, 8), dtype=np.uint8)
        self.board_result_color_buffer = []     # store history of self.board_result_color

        self.top_filter = True
        self.side_filter = True
        self.color_filter = True
        self.top_filter_length = 10
        self.side_filter_length = 10
        self.color_filter_length = 5

        self.clustering = None
        self.clustering_lock = False
        self.clustering_flip = False
        self.get_logger().info("CHESSBOARD CLASSIFIER READY!!")
    def chessboard_pose_top_callback(self, img_pose):
        # frame = self.bridge.imgmsg_to_cv2(img_pose.image, desired_encoding='passthrough')
        # self.get_logger().info(str(get_tile(img_pose)))
        # try:
        # CNNinputs_padded, angle_list = get_tile_ImgPose(img_pose, only_base=True)
        CNNinputs_padded = get_tile_top_ImgPose(img_pose)
        Y = self.top_model.predict(np.array(CNNinputs_padded).reshape((-1, 224, 224, 3)))
        Y = np.array(Y).reshape((-1))
        Y = np.where(Y < 0, 0, 1)   # Interpreted prediction
        # Classify only index which not empty (extract color feature)
        tile_index_non_empty = np.argwhere(Y != 0).reshape(-1)
        CNNinputs_padded_non_empty = np.array(CNNinputs_padded)[tile_index_non_empty]
        if len(CNNinputs_padded_non_empty) > 0:
            Y_color = np.array(self.color_model.predict(np.array(CNNinputs_padded_non_empty)))
        else: Y_color = []

        if self.clustering_lock:    # Use saved clustering model
            clustering = self.clustering
            cluster_result = clustering.predict(Y_color)
        else:
            if len(Y_color) < 2: # not enough sample for clustering
                clustering = None
                self.get_logger().warn("Not enough sample for clustering (Minimum is 2)")
            else:
                clustering = KMeans(n_clusters=2, random_state=0).fit(Y_color)
                cluster_result = clustering.labels_
                self.clustering = clustering    # update clustering model
                ## fit to nearest side automatically ##
                white_side, black_side = [], []
                for j in range(len(tile_index_non_empty)):
                    row_index = int(tile_index_non_empty[j]/8)
                    if row_index < 4: black_side.append(cluster_result[j])
                    else: white_side.append(cluster_result[j])
                try:
                    if np.argmax(np.bincount(white_side)) == 0: # Whites already labeled as '0'
                        self.clustering_flip = False
                    else: self.clustering_flip = True
                except: pass

        if clustering is not None:
            canvas1, canvas2 = [], []
            for j in range(len(cluster_result)):
                cluster_label = cluster_result[j]
                if self.clustering_flip == True:
                    cluster_label = abs(cluster_label - 1)  # flip 0 <-> 1
                if cluster_label == 0:
                    canvas1.append(CNNinputs_padded_non_empty[j])
                elif cluster_label == 1:
                    canvas2.append(CNNinputs_padded_non_empty[j])
            # self.board_result_color_buffer.append(self.board_result_color)
            color_array = np.zeros((8, 8), dtype=np.uint8)
            for i in range(len(tile_index_non_empty)):
                non_empty_index = tile_index_non_empty[i]
                color_array[int(non_empty_index/8)][non_empty_index%8] = cluster_result[i]
            self.board_result_color_buffer.append(color_array)
            # print(self.board_result_color_buffer)
            if len(self.board_result_color_buffer) >= self.color_filter_length: # fill buffer first
                for y in range(8):
                    for x in range(8):
                        buffer = []
                        for i in range(self.color_filter_length): buffer.append(self.board_result_color_buffer[i][y][x])
                        # update value to most frequent in buffer
                        self.board_result_color[y][x] = np.argmax(np.bincount(buffer))
                        # if np.all(np.array(buffer) == buffer[0]): self.board_result_color[y][x] = buffer[0]
                while len(self.board_result_color_buffer) >= self.color_filter_length:
                    self.board_result_color_buffer.pop(0)   # remove first element in buffer

            while len(canvas1) < len(canvas2): canvas1.append(np.zeros_like(CNNinputs_padded_non_empty[j]))
            while len(canvas2) < len(canvas1): canvas2.append(np.zeros_like(CNNinputs_padded_non_empty[j]))
            canvas1 = cv2.hconcat(canvas1)
            canvas2 = cv2.hconcat(canvas2)
            canvas = imutils.resize(cv2.vconcat([canvas1, canvas2]), height=120)
            image_msg = self.bridge.cv2_to_imgmsg(canvas, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            self.cluster_viz_pub.publish(image_msg)
            # cv2.imshow("KMeans", canvas)

        self.board_result_binary_buffer.append(Y.reshape((8, 8)))
        if len(self.board_result_binary_buffer) >= self.top_filter_length:  # fill buffer first
            for y in range(8):
                for x in range(8):
                    buffer = []
                    for i in range(self.top_filter_length): buffer.append(self.board_result_binary_buffer[i][y][x])
                    # update value to most frequent in buffer
                    self.board_result_binary[y][x] = np.argmax(np.bincount(buffer))
                    # if np.all(np.array(buffer) == buffer[0]): self.board_result_binary[y][x] = buffer[0]
            while len(self.board_result_binary_buffer) >= self.top_filter_length:
                self.board_result_binary_buffer.pop(0)  # remove first element in buffer
        # self.get_logger().info(str(self.board_result_binary))

        ## Publish FEN binary ##
        fen_binary_msg = UInt8MultiArray()
        fen_binary_msg.data = [int(item) for item in self.board_result_binary.reshape(-1)]
        self.fen_binary_pub.publish(fen_binary_msg)
        fen_color_msg = UInt8MultiArray()
        fen_color_msg.data = [int(item) for item in self.board_result_color.reshape(-1)]
        self.fen_color_pub.publish(fen_color_msg)

        vertical_images = []
        for x in range(8):
            image_list_vertical = []
            for y in range(8):
                canvas = resize_and_pad(CNNinputs_padded[8 * y + x].copy(), size=100)
                # cv2.putText(canvas, str(round(angle_list[8 * y + x])), (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(0, 255, 0))
                ## Fill GREEN/RED overlay
                piece_overlay = np.zeros(canvas.shape, dtype=np.uint8)
                if self.board_result_binary[y][x]: piece_overlay[:] = (0, 255, 0)
                else: piece_overlay[:] = (0, 0, 255)
                canvas = cv2.addWeighted(canvas, 0.7, piece_overlay, 0.3, 0)
                image_list_vertical.append(cv2.copyMakeBorder(canvas, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, (0, 255, 0)))
            vertical_images.append(np.vstack(image_list_vertical))
        combined_images = np.hstack(vertical_images)
        combined_images = imutils.resize(combined_images, height=480)
        image_msg = self.bridge.cv2_to_imgmsg(combined_images, "bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.top_cnn_viz_pub.publish(image_msg)
        # cv2.imshow("Top CNN inputs", combined_images)
        # except Exception as e: self.get_logger().warn(str(e))

        # cv2.imshow("Top", frame)
        # cv2.waitKey(1)
    def chessboard_pose_side_callback(self, img_pose):
        # frame = self.bridge.imgmsg_to_cv2(img_pose.image, desired_encoding='passthrough')
        # self.get_logger().info(str(get_tile(img_pose)))
        try:
            board_not_empty = np.argwhere(self.board_result_binary.reshape(-1) != 0).reshape(-1)
            tile_index_non_empty = board_not_empty
            CNNinputs_padded, angle_list = get_tile_ImgPose(img_pose, only_base=False)
            CNNinputs_padded_non_empty = np.array(CNNinputs_padded)[tile_index_non_empty]
            board_result = np.zeros((8, 8))    # Reset to empty board
            if len(board_not_empty) > 0:
                Y = np.array(self.side_model.predict(np.array(CNNinputs_padded_non_empty)))
                Y = np.argmax(Y, axis=1)  # Use class with max score
                # Remap back to chessboard
                for i in range(len(tile_index_non_empty)):
                    index = tile_index_non_empty[i]
                    board_result[int(index / 8)][index % 8] = Y[i] + 1
            self.board_result_buffer.append(board_result)
            if len(self.board_result_buffer) >= self.side_filter_length:  # fill buffer first
                for y in range(8):
                    for x in range(8):
                        buffer = []
                        for i in range(self.side_filter_length): buffer.append(self.board_result_buffer[i][y][x])
                        # update value to most frequent in buffer
                        self.board_result[y][x] = np.argmax(np.bincount(buffer))
                        # if np.all(np.array(buffer) == buffer[0]): self.board_result[y][x] = buffer[0]
                while len(self.board_result_buffer) >= self.side_filter_length:
                    self.board_result_buffer.pop(0)  # remove first element in buffer
            fen_message = String()
            fen_message.data = to_FEN(self.board_result, self.board_result_color)
            self.fen_pub.publish(fen_message)

            vertical_images = []
            for x in range(8):
                image_list_vertical = []
                for y in range(8):
                    canvas = resize_and_pad(CNNinputs_padded[8 * y + x].copy(), size=100)
                    cv2.putText(canvas, str(round(angle_list[8 * y + x])), (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color=(0, 255, 0))
                    ## Fill CHESS TYPE overlay
                    piece_overlay = np.zeros(canvas.shape, dtype=np.uint8)
                    if self.board_result[y][x] != 0:
                        char = symbol_dict[self.board_result[y][x]]
                        if self.board_result_color[y][x] == 1: char = char.upper()
                        cv2.putText(canvas, char, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 3, color=(0, 255, 0), thickness=3)
                        canvas = cv2.addWeighted(canvas, 0.7, piece_overlay, 0.3, 0)
                    image_list_vertical.append(cv2.copyMakeBorder(canvas, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, (0, 255, 0)))
                vertical_images.append(np.vstack(image_list_vertical))
            combined_images = np.hstack(vertical_images)
            combined_images = imutils.resize(combined_images, height=480)
            image_msg = self.bridge.cv2_to_imgmsg(combined_images, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            self.side_cnn_viz_pub.publish(image_msg)
            # cv2.imshow("Side CNN inputs", combined_images)
        except Exception as e: print(e)

        # cv2.imshow("Side", frame)
        cv2.waitKey(1)
    def cluster_lock_callback(self, request, response):
        lock, flip = request.lock, request.flip
        self.clustering_flip = False if flip == 0 else True
        if lock == 0:   # Lock -> Unlock
            self.clustering_lock = 0
        elif lock == 1: # Unlock -> Lock
            if self.clustering is None: # Don't have clustering model yet
                response.acknowledge = 0
                return response
            self.clustering_lock = 1
        response.acknowledge = 1
        return response

def main():
    rclpy.init()
    chessboard_classifier = ChessboardClassifier()
    rclpy.spin(chessboard_classifier)
    # chessboard_detector.destroy_subscription(chessboard_detector.camera_sub) # Not need camera after init pose
    rclpy.shutdown()

if __name__ == "__main__":
    main()