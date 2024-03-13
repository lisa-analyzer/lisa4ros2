#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32,UInt8,Int64
from module89.srv import StringMessage
from module89.srv import ClusterLock
from module89.srv import PoseLock
from module89.srv import FindBestMove
from cv_bridge import CvBridge
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from module89.srv import ExecuteBestMove
from module89.srv import PseudoBoardSetup
#Communication
import time
import UdpComms as U
import cv2
from Communication import Communication,CRC8
import numpy as np

canvas = None
canvas_tmp = None
four_points = []
pressmode = None
def click_corner(event, x, y, flags, param):
    global img, four_points, canvas, canvas_tmp
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(canvas, (x, y), 5, (255, 0, 0), -1)
        four_points.append((x, y))
    if event == cv2.EVENT_MOUSEMOVE:
        canvas_tmp = canvas.copy()
        cv2.line(canvas_tmp, (x, 0), (x, 1080), (0, 255, 0), 1)
        cv2.line(canvas_tmp, (0, y), (1920, y), (0, 255, 0), 1)



class GameController(Node):

    def __init__(self):
        super().__init__('game_controller')
        self.pyserial_connected = False
        ip = "127.0.0.1"
        self.data_sock = U.UdpComms(udpIP=ip, portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True) #Main Communication
        self.q1 = float(0)
        self.q2= float(0)
        self.q3= float(0)
        self.q4= float(0)
        self.x= float(0)
        self.y= float(0)
        self.z= float(0)
        self.bridge = CvBridge()
        self.status = [0,0,0,0,0,0,0]
        self.name = ['q1', 'q2', 'q3', 'q4', 'x', 'y', 'z']
        self.waitMove = False
        self.robotmode = 0
        self.move_buffer = [0,0,0,0,0,0]
        self.time_delay = 0
        self.chessboard_fen = ""
        self.first_move = True
        self.timer_before_movechess = 0
        self.dl_buffer = []
        self.wait_time = 5
        self.camera0_frame = None
        self.camera1_frame = None
        self.ai_ready = None
        self.autoplay_mode = None
        self.autoplay_fen = None
        self.readyStart = False
        self.piece_height_type = [0.08,0.0731,0.0615,0.048,0.045,0.055]
        # self.img_sock1 = U.UdpComms(udpIP=ip, portTX=8002,portRX=8001, suppressWarnings=True) # Image1 Socket-
        # self.img_sock2 = U.UdpComms(udpIP=ip, portTX=8003,portRX=8001, suppressWarnings=True) # Image2 Socket
        # self.image_bytes = cv2.imencode('.jpg', cv2.imread("2.jpg"))[1].tobytes()
        self.publisher_ = self.create_publisher(Float32, 'chessboard/joint0', 10)
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback_joint0)

        self.encoder = self.create_publisher(Float32, '/chessboard/encoder', 10)
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback_encoder)
        self.time23 = None

        #command_cluster_lock , command_pose_lock

        # self.chessboard_fen_sub = self.create_subscription(
        #     String,
        #     '/chessboard/fen',
        #     self.fen_callback,
        #     10)
        self.chessboard_fen_sub = self.create_subscription(
            String,
            '/chessboard/pseudo_fen',
            self.fen_callback,
            10)

        self.camera0_image = self.create_subscription(
            Image,
            '/camera0/image',
            self.camera0_image_callback,
            10)

        self.camera1_image = self.create_subscription(
            Image,
            '/camera1/image',
            self.camera1_image_callback,
            10)

        self.ai_stat = self.create_subscription(
            UInt8,
            '/chessboard/AI_ready',
            self.ai_stat_callback,
            10)

        self.ai_bestmove = self.create_subscription(
            String,
            '/chessboard/AI_bestmove',
            self.ai_bestmove_callback,
            10)


        # self.chessboard_fen

        self.bestmove_cli = self.create_client(FindBestMove, 'find_best_move')
        # while not self.bestmove_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.bestmove_req = FindBestMove.Request()
        self.bestmove_get = False

        self.pseudoboardsetup_cli = self.create_client(PseudoBoardSetup, 'pseudo_board_setup')
        # while not self.bestmove_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.pseudoboardsetup_req = PseudoBoardSetup.Request()
        self.pseudoboardsetup_get = False

        # self.fen_cli = self.create_client(StringMessage, 'fen')
        # while not self.fen_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.fen_req = StringMessage.Request()
    #/chessboard/fen
        self.pose_lock_cli = self.create_client(PoseLock, 'command_pose_lock')
        # while not self.pose_lock_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.pose_lock_req = PoseLock.Request()
        self.pose_lock_get = False

        self.execute_bestmove_cli = self.create_client(ExecuteBestMove, 'execute_bestmove')
        # while not self.pose_lock_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.execute_bestmove_req = ExecuteBestMove.Request()
        self.execute_bestmove_get = False

        self.cluster_lock_cli = self.create_client(ClusterLock, 'command_cluster_lock')
        # while not self.cluster_lock_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.cluster_lock_req = ClusterLock.Request()
        self.cluster_lock_get = False
        self.get_ready = False
        self.ai_move = None
        self.save_state = 0



        self.get = False
        self.TimeBefore = time.time()
        self.connect_pyserial(com="/dev/Narwhal")
        timer_period = 1 / 30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.run()
        self.camera_timer = self.create_timer(1/10, self.camera_callback)
    def ai_stat_callback(self,msg):
        if msg.data == 2 and not self.get_ready:
            self.get_ready = True
            if self.save_state != 2:
                self.save_state = 2
                self.data_sock.SendData("AI READY")
        elif msg.data == 1:
            if self.save_state != 1:
                self.save_state = 1
                self.data_sock.SendData("AI COMPUTE")
        else:
            if self.save_state != 0:
                self.save_state = 0
                self.data_sock.SendData("AI TRACKING")
            self.get_ready = False


    def ai_bestmove_callback(self,msg):
        # print(self.get_ready, self.autoplay_mode, self.robotmode)
        if self.get_ready and self.autoplay_mode and self.robotmode == 1:
            print(self.chessboard_fen)
            a =self.chessboard_fen.split(" ")
            b = a[0].split("/")
            for i in range(len(b)):
                b[i] = b[i][::-1]
            a[0] = "/".join(b)
            self.chessboard_fen = " ".join(a)
            print(self.chessboard_fen)


            self.autoplay_fen = str(self.chessboard_fen)
            self.ai_move = msg.data
            self.get_ready = False
            board = [[]]
            board_row = 0
            alphabet = "hgfedcba"
            piece_type = "KQBNRP"
            move = self.ai_move
            for i in self.autoplay_fen:
                if i == "/":
                    board_row += 1
                    board.append([])
                    continue
                if i.isnumeric():
                    for j in range(int(i)):
                        board[board_row].append('*')
                    continue
                board[board_row].append(i)
            if board[7 - (int(move[1]) - 1)][alphabet.index(move[0])].lower() == "p":
                if(move[0] != move[2]):
                    if board[7 - (int(move[3]) - 1)][alphabet.index(move[2])].lower() == "*":
                        print("enpassant")
                        # ((int(move[1])))*8 + (alphabet.index(move[2])),65,typep
                        self.narwhal.MoveChess(((int(move[1]))-1)*8 + (alphabet.index(move[2])), 64, self.piece_height_type[5])
                        self.time_delay = time.time()
                        self.move_buffer[0], self.move_buffer[1], self.move_buffer[2] = ((int(move[1])-1))*8 + (alphabet.index(move[0])), ((int(move[3])-1)) * 8 + (alphabet.index(move[2])), self.piece_height_type[5]
                        self.waitMove = True
                        # board[7 - (int(move[1]) - 1)][alphabet.index(move[2])] = "69"  # REMOVE
                    else:
                        self.narwhal.MoveChess(((int(move[3])) - 1) * 8 + (alphabet.index(move[2])), 64,
                                               self.piece_height_type[5])
                        self.time_delay = time.time()
                        self.move_buffer[0], self.move_buffer[1], self.move_buffer[2] = ((int(move[1])) - 1) * 8 + (alphabet.index(move[0])), ((int(move[3])) - 1) * 8 + (alphabet.index(move[2])),self.piece_height_type[piece_type.index(board[7 - (int(move[1]) - 1)][alphabet.index(move[0])].upper())]
                        self.waitMove = True
                else:

                    self.narwhal.MoveChess(((int(move[1])) - 1) * 8 + (alphabet.index(move[0])), ((int(move[3])) - 1) * 8 + (alphabet.index(move[2])),self.piece_height_type[piece_type.index(board[7 - (int(move[1]) - 1)][alphabet.index(move[0])].upper())])

            elif board[7 - (int(move[1]) - 1)][alphabet.index(move[0])].lower() == "k":
                if abs(alphabet.index(move[0]) - alphabet.index(move[2])) == 2:
                    print("castling")
                    if alphabet.index(move[0]) - alphabet.index(move[2]) == 2:
                        # print("MOVE" + str((move[1]-1)*8) +" " + str(((move[1]-1)*8)+(alphabet.index(move[0])-1))+ " " + str(self.piece_height_type[4]))
                        # self.narwhal.MoveChess((move[1]-1)*8,((move[1]-1)*8)+(alphabet.index(move[0])-1),self.piece_height_type[4])
                        # self.time_delay = time.time()
                        # self.move_buffer[0], self.move_buffer[1], self.move_buffer[2] = (((int(move[1])) - 1) * 8 + (alphabet.index(move[0]))),((int(move[3])) - 1) * 8 + (alphabet.index(move[2])), self.piece_height_type[0]
                        # self.waitMove = True

                        # MOVE 0 kingx -1

                        # self.narwhal.MoveChess(dl[1], dl[2], dl[3])
                        # self.move_buffer[0], self.move_buffer[1], self.move_buffer[2] = dl[4], dl[5], dl[6]
                        # self.waitMove = True
                        # self.time_delay = time.time()
                        pass
                    elif alphabet.index(move[0]) - alphabet.index(move[2]) == -2:
                        # print(((int(move[1]) - 1) * 8)+7, ((int(move[1]) - 1) * 8) + (alphabet.index(move[0])+1),self.piece_height_type[4])

                        self.narwhal.MoveChess(((int(move[1]) - 1) * 8)+7, ((int(move[1]) - 1) * 8) + (alphabet.index(move[0])+1),
                                               self.piece_height_type[4])
                        self.time_delay = time.time()
                        self.move_buffer[0], self.move_buffer[1], self.move_buffer[2] = (((int(move[1])) - 1) * 8 + (
                            alphabet.index(move[0]))), ((int(move[3])) - 1) * 8 + (alphabet.index(move[2])), \
                                                                                        self.piece_height_type[0]
                        self.waitMove = True
                else:
                    if board[7 - (int(move[3]) - 1)][alphabet.index(move[2])].lower() != "*":
                        self.narwhal.MoveChess(((int(move[3])) - 1) * 8 + (alphabet.index(move[2])),
                                               64, self.piece_height_type[
                                                   piece_type.index(
                                                       board[7 - (int(move[3]) - 1)][alphabet.index(move[2])].upper())])
                        self.time_delay = time.time()
                        self.move_buffer[0], self.move_buffer[1], self.move_buffer[2] = ((int(move[1])) - 1) * 8 + (
                            alphabet.index(move[0])), ((int(move[3])) - 1) * 8 + (alphabet.index(move[2])), \
                                                                                        self.piece_height_type[
                                                                                            piece_type.index(
                                                                                                board[7 - (int(
                                                                                                    move[1]) - 1)][
                                                                                                    alphabet.index(move[
                                                                                                                       0])].upper())]
                        self.waitMove = True
                    else:
                        self.narwhal.MoveChess(((int(move[1])) - 1) * 8 + (alphabet.index(move[0])),
                                               ((int(move[3])) - 1) * 8 + (alphabet.index(move[2])),
                                               self.piece_height_type[
                                                   piece_type.index(
                                                       board[7 - (int(move[1]) - 1)][alphabet.index(move[0])].upper())])
            else:
                if board[7 - (int(move[3]) - 1)][alphabet.index(move[2])].lower() != "*":
                    self.narwhal.MoveChess( ((int(move[3])) - 1) * 8 + (alphabet.index(move[2])),
                                      64, self.piece_height_type[
                                           piece_type.index(
                                               board[7 - (int(move[3]) - 1)][alphabet.index(move[2])].upper())])
                    self.time_delay = time.time()
                    self.move_buffer[0], self.move_buffer[1], self.move_buffer[2] = ((int(move[1])) - 1) * 8 + (alphabet.index(move[0])),((int(move[3])) - 1) * 8 + (alphabet.index(move[2])), self.piece_height_type[
                                               piece_type.index(
                                                   board[7 - (int(move[1]) - 1)][alphabet.index(move[0])].upper())]
                    self.waitMove = True
                else:
                    self.narwhal.MoveChess(((int(move[1])) - 1) * 8 + (alphabet.index(move[0])),
                                           ((int(move[3])) - 1) * 8 + (alphabet.index(move[2])), self.piece_height_type[
                                               piece_type.index(
                                                   board[7 - (int(move[1]) - 1)][alphabet.index(move[0])].upper())])
            self.time23 = time.time()






    def camera_callback(self):
        global four_points,pressmode,canvas,canvas_tmp
        if self.camera0_frame is not None and self.camera1_frame is not None:
            # cv2.imshow("camera0",self.camera0_frame)
            # cv2.imshow("camera1",self.camera1_frame)
            key = cv2.waitKey(1)
            if key == ord('1') or key == ord('2'):
                four_points = []
                if key == ord('1'):
                    pressmode = 1
                    canvas = self.camera0_frame.copy()
                elif key == ord('2'):
                    pressmode = 2
                    canvas = self.camera1_frame.copy()
                cv2.namedWindow('Assign Corner', cv2.WND_PROP_FULLSCREEN)
                cv2.setWindowProperty('Assign Corner', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_FULLSCREEN)
                cv2.setMouseCallback('Assign Corner', click_corner)
            if key == ord(' '): # Start capture
                self.pose_lock_request(pressmode,[[int(point[0]), int(point[1])] for point in four_points])
            if canvas is not None:
                canvas_tmp = canvas.copy()
                cv2.imshow('Assign Corner',canvas_tmp)
            # key = cv2.waitKey(1)


    def camera0_image_callback(self,image):
        self.camera0_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

    def camera1_image_callback(self, image):
        self.camera1_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

    def timer_callback_joint0(self):
        msg = Float32()
        msg.data = float(self.status[0])
        self.publisher_.publish(msg)

    def timer_callback_encoder(self):
        msg = Float32()
        msg.data = float(self.chess_data)
        self.encoder.publish(msg)

    def fen_callback(self,msg):
        self.chessboard_fen = msg.data


    def execute_request(self,a1,a2):
         msg = String()
         self.execute_bestmove_req.mode = False
         self.execute_bestmove_req.move = msg
         self.execute_bestmove_future = self.execute_bestmove_cli.call_async(self.execute_bestmove_req)

    def swapSide(self,side):
        # msg = Int64()
        # msg.data = int(side)
        self.pseudoboardsetup_req.ai_is = int(side)
        self.pseudoboardsetup_future = self.pseudoboardsetup_cli.call_async(self.pseudoboardsetup_req)


    def bestmove_request(self,fen):
        bestmove = String()
        # print(str(fen) + " - - 1 2")
        bestmove.data = str(fen)  + " - - 1 2"
        # bestmove.data = "8/2p4k/2q1pQ1p/4pN2/3br3/7P/5Pr1/5R1K b - - 1 2"
        self.bestmove_req.fen = bestmove
        self.bestmove_future = self.bestmove_cli.call_async(self.bestmove_req)
        self.bestmove_get = True

    def point_request(self):
        pass

    def fen_request(self):
        pass
        # self.req.input = "REQUEST"
        # self.future = self.cli.call_async(self.req)

    def cluster_lock_request(self,lock,flip):
        self.cluster_lock_req.lock = lock
        self.cluster_lock_req.flip = flip
        self.cluster_lock_future = self.cluster_lock_cli.call_async(self.cluster_lock_req)

    def pose_lock_request(self,mode,corners = []):
        self.pose_lock_req.mode = mode
        croners = UInt16MultiArray()
        croners.data = [int(item) for item in np.array(corners).reshape(-1)]
        self.pose_lock_req.corners = croners
        self.pose_lock_future = self.pose_lock_cli.call_async(self.pose_lock_req)


    def connect_pyserial(self,com="/dev/ttyUSB0",baudrate=1000000):
        if not self.pyserial_connected:
            self.narwhal = Communication(port=com, baudrate=baudrate)
            print("Connection : " + str(self.narwhal.Connection_Test()))
            if self.narwhal.Connection_Test() != False:
                self.pyserial_connected = True;
        else:
            print("Already Connected!")

    def check_pyserial(self):
        return self.pyserial_connected

    def close_pyserial(self):
        self.narwhal.Close()

    def timer_callback(self):

        if self.bestmove_get:
            if self.bestmove_future.done():
                try:
                    response = self.bestmove_future.result()
                except Exception as e:
                    print(e)
                else:
                    self.bestmove_get = False
                    self.data_sock.SendData("BESTMOVE " +str(response.bestmove.data))

        if self.cluster_lock_get:
            if self.cluster_lock_future.done():
                try:
                    response = self.cluster_lock_future.result()
                except Exception as e:
                    print(e)
                else:
                    self.cluster_lock_get = False
                    self.data_sock.SendData(response.acknowledge)

        if self.pose_lock_get:
            if self.pose_lock_future.done():
                try:
                    response = self.pose_lock_future.result()
                except Exception as e:
                    print(e)
                else:
                    self.pose_lock_get = False
                    self.data_sock.SendData(response.acknowledge)

        if self.execute_bestmove_get:
            if self.execute_bestmove_future.done():
                try:
                    response = self.execute_bestmove_future.result()
                except Exception as e:
                    print(e)
                else:
                    self.execute_bestmove_get = False
                    # self.data_sock.SendData(response.acknowledge)

        if(time.time() - self.time_delay >=5):
            if self.waitMove:
                if self.robotmode == 1:
                    self.waitMove = False
                    self.narwhal.MoveChess(self.move_buffer[0],self.move_buffer[1],self.move_buffer[2])

                    self.move_buffer[0],self.move_buffer[1],self.move_buffer[2] = 0,0,0
        if(time.time() - self.timer_before_movechess > self.wait_time and not self.first_move ):
            print("after 5")
            dl = self.dl_buffer
            if len(dl) == 4:
                stat , feedback = self.narwhal.MoveChess(dl[1],dl[2],dl[3])
            else:
                print(len(dl))
                self.narwhal.MoveChess(dl[1], dl[2],dl[3])
                self.move_buffer[0],self.move_buffer[1],self.move_buffer[2] = dl[4],dl[5],dl[6]
                self.waitMove = True
                self.time_delay = time.time()
            self.wait_time = 9999999
        if self.time23 is not None:

            if time.time() - self.time23 >=10:
                self.execute_request(0,0)
                self.time23 = time.time() +99999999



        data = self.data_sock.ReadReceivedData()  # read data
        # if (time.time() - self.TimeBefore >= 0.03):
        #     self.TimeBefore = time.time()
        if self.pyserial_connected:
            status = str(self.narwhal.ReadAll())
            status = status.replace("[", "")
            status = status.replace("]", "")
            status = status.replace(",", "")
            status_unpack = status.split(" ")
            self.status[0:5] = status_unpack[3:8]
            self.status[5:8] = status_unpack[13:16]
            self.status[8:11] = status_unpack[17:20]
            ## publish joint0 state ##
            msg = Float32()
            msg.data = float(self.status[0])
            self.publisher_.publish(msg)
            ## publish encoder state ##
            msg = Float32()
            msg.data = float(status_unpack[0])
            self.encoder.publish(msg)
            # self.chess_data = status_unpack[0]
            # print(status_unpack[13:16])
            if status_unpack[16] == "0":
                if(self.robotmode != 1):
                    self.data_sock.SendData("ALPHA 0")
                    self.data_sock.SendData("ROBOT IDLE")
                    print(self.robotmode)
                self.robotmode = 1
            else:
                if (self.robotmode != 0):
                    self.data_sock.SendData("ALPHA 1")
                    self.data_sock.SendData("ROBOT MOVING")
                    print(self.robotmode)
                self.robotmode = 0

            status = "status " + status
            self.data_sock.SendData(status)
        if data != None:  # if NEW data has been received since last ReadReceivedData function call
            self.get_logger().info('Publishing: "%s"' % data)
            dl = data.split(" ")
            if dl[0] == "SETHOME":
                print(self.narwhal.SetHome())
            elif dl[0] == "STOPPOSE":
                self.narwhal.StopPose()
            elif dl[0] == "SETZERO":
                self.narwhal.SetZero()
            elif dl[0] == "POINT":
                self.point_request()
            elif dl[0] == "GRIP":
                pass
            elif dl[0] == "CHECKREADY":
                if self.ai_ready:
                    self.data_sock.SendData("AIMOVE " + self.ai_move)
                    self.time23 = time.time()
                    self.ai_ready = False
            elif dl[0] == "FEN":
                self.bestmove_request(dl[1] + " " + dl[2])
            elif dl[0] == "ABC":
                self.readyStart = True
            elif dl[0] == "PYSERIAL":
                self.connect_pyserial()
            elif dl[0] == "POSE":
                self.pose_lock_request(int(dl[1]))
            elif dl[0] == "CLUSTER":
                self.cluster_lock_request(int(dl[1]),int(dl[2]))
            elif dl[0] == "SWAP":
                self.swapSide(int(dl[1]))
            elif dl[0] == "AUTOPLAY":
                self.autoplay_mode = bool(dl[1])
            elif dl[0] == "MOVE":
                if self.first_move:
                    self.first_move = False
                    self.timer_before_movechess = time.time()
                    self.narwhal.MoveChess(65,65,0)
                    self.dl_buffer = dl
                else:
                    if len(dl) == 4:
                        stat , feedback = self.narwhal.MoveChess(dl[1],dl[2],dl[3])
                    else:
                        print(len(dl))
                        self.narwhal.MoveChess(dl[1], dl[2],dl[3])
                        self.move_buffer[0],self.move_buffer[1],self.move_buffer[2] = dl[4],dl[5],dl[6]
                        self.waitMove = True
                        self.time_delay = time.time()

            elif dl[0] == "REQUESTFEN":
                # self.bestmove_request()
                # self.get = True
                self.data_sock.SendData("FEN " + self.chessboard_fen)
            elif dl[0] == "SET":
                if dl[1] == "x" or dl[1] == "y" or dl[1] == "z":
                    if dl[1] == "x":
                        self.status[5] = float(dl[2])
                    elif dl[1] == "y":
                        self.status[6] = float(dl[2])
                    elif dl[1] == "z":
                        self.status[7] = float(dl[2])
                    self.narwhal.SetTask(*self.status[5:8])
                    # print(*self.status[5:8])
                else:
                    if dl[1] == "q1":
                        self.status[0] = float(dl[2])
                    elif dl[1] == "q2":
                        self.status[1] = float(dl[2])
                    elif dl[1] == "q3":
                        self.status[2] = float(dl[2])
                    elif dl[1] == "q4":
                        self.status[3] = float(dl[2])
                    self.narwhal.SetJoint(*self.status[:4])

                # for i in range(4):
                #     if abs(all_jt[i]) >=0:
                #         self.narwhal.JogJoint(*all_jt[0:4])
                #         break
                # for i in range(4,7):
                #     if abs(all_jt[i]) >=0:
                #         self.narwhal.JogTask(*all_jt[4:7])
                #         break
            elif dl[0] == "JOG":
                name = ['q1', 'q2', 'q3', 'q4', 'x', 'y', 'z']
                all_jt = [0, 0, 0, 0, 0, 0, 0]  # j1j2j3j4xyz
                all_jt[name.index(dl[1])] = float(dl[2])
                if dl[1] == "x" or dl[1] == "y" or dl[1] == "z":
                    for i in range(4, 7):
                        if (abs(all_jt[i]) >= 0):
                            print(*all_jt[4:7])
                            self.narwhal.JogTask(*all_jt[4:7])
                            break
                else:
                    for i in range(4):
                        if (abs(all_jt[i]) >= 0):
                            print(*all_jt[0:4])
                            self.narwhal.JogJoint(*all_jt[0:4])
                            break


                # if abs(all_jt[0]) >= 0 or abs(all_jt[1]) >= 0 or abs(all_jt[2]) >= 0 or abs(all_jt[3]) >= 0:
                #     self.narwhal.JogJoint(all_jt[0], all_jt[1], all_jt[2], all_jt[3])
                # elif abs(all_jt[4]) >= 0 or abs(all_jt[5]) >= 0 or abs(all_jt[6]) >= 0:
                #     self.narwhal.JogTask(all_jt[4], all_jt[5], all_jt[6])

def main(args=None):
    rclpy.init(args=args)

    game_controller = GameController()

    rclpy.spin(game_controller)



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    game_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()