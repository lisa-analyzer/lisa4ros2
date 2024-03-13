#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Bool, UInt8MultiArray, UInt8, String
from module89.srv import ExecuteBestMove, PseudoBoardSetup

import chess
import chess.engine
engine = chess.engine.SimpleEngine.popen_uci("stockfish")
engine.configure({"Threads": 6})
limit = chess.engine.Limit(time=5)

def fen2binary(fen):
    fen_board = fen.split(' ')[0]
    fen_board = fen_board.split('/')
    board_binary = np.zeros((8, 8), dtype=np.uint8)
    for row in range(8):
        counter = 0
        for char in fen_board[row]:
            if char.isnumeric(): counter += int(char)
            else:
                board_binary[row][counter] = 1
                counter += 1
    return board_binary
def fen2color(fen):
    fen_board = fen.split(' ')[0]
    fen_board = fen_board.split('/')
    board_color = np.zeros((8, 8), dtype=np.uint8)
    for row in range(8):
        counter = 0
        for char in fen_board[row]:
            if char.isnumeric():
                counter += int(char)
            else:
                board_color[row][counter] = char.isupper()
                counter += 1
    return board_color

class PseudoStateController(Node):
    def __init__(self):
        super().__init__('psuedo_state_controller')
        self.fen_binary_sub = self.create_subscription(UInt8MultiArray, '/chessboard/fen_binary', self.fen_binary_callback, 10)
        self.fen_color_sub = self.create_subscription(UInt8MultiArray, '/chessboard/fen_color', self.fen_color_callback, 10)
        # self.camera0_hand_sub = self.create_subscription(Bool, '/camera0/hand', self.camera0_hand_callback, 10)
        # self.camera1_hand_sub = self.create_subscription(Bool, '/camera1/hand', self.camera1_hand_callback, 10)

        self.turn_pub = self.create_publisher(UInt8, '/chessboard/turn', 10)
        self.AI_ready_pub = self.create_publisher(UInt8, '/chessboard/AI_ready', 10)
        self.AI_bestmove_pub = self.create_publisher(String, '/chessboard/AI_bestmove', 10)
        self.illegal_pub = self.create_publisher(Bool, '/chessboard/illegal_move', 10)
        self.pseudo_fen = self.create_publisher(String, '/chessboard/pseudo_fen', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)   # 10 Hz

        self.execute_bestmove_srv = self.create_service(ExecuteBestMove, 'execute_bestmove', self.execute_bestmove_callback)
        self.setting_srv = self.create_service(PseudoBoardSetup, 'pseudo_board_setup', self.setup_callback)

        self.board = chess.Board()  # init Board() object
        self.fen_binary_old = fen2binary(self.board.fen())
        self.fen_binary = None
        self.fen_color = None
        # self.hand_detected = False
        self.AI_side = 0    # 0=White, 1=Black
        self.turn = 0       # 0=White, 1=Black (White always play first)
        self._AI_ready = 0  # 0=Idle, 1=Busy, 2=Ready
        self.best_move = None
        self.illegal_move = False
        self.first_turn = True
    @property
    def AI_turn(self):      # AI turn status
        return True if self.AI_side == self.turn else False
    @property
    def human_turn(self):   # human turn status (opposite to AI)
        return False if self.AI_side == self.turn else True
    @property
    def AI_ready(self):
        return self._AI_ready
    @AI_ready.setter
    def AI_ready(self, ready:int):  # publish AI state every time it changed
        self.AI_ready_pub.publish(UInt8(data=ready))
        self._AI_ready = ready
    def any_change(self):
        if self.fen_binary is None: return False  # Not recieved fen_binary yet
        if (self.fen_binary == self.fen_binary_old).all():
            return False # no change yet
        else: return True
    def valid_move(self):  # search for matched binary from legal move and return (valid, move_obj)
        if self.fen_binary is None or self.fen_color is None: return False, None  # Not recieved fen_binary yet
        for legal_move in self.board.legal_moves:
            self.board.push(legal_move)
            candidate_binary = fen2binary(self.board.fen())
            candidate_color = fen2color(self.board.fen())
            binary_matched = (candidate_binary == self.fen_binary).all()
            color_matched = True
            for y in range(8):
                for x in range(8):
                    if self.fen_binary[y][x]:
                        if self.fen_color[y][x] != candidate_color[y][x]: color_matched = False # color not matched
            if binary_matched and color_matched:  # found match binary board
                self.board.pop()    # undo true move
                return True, legal_move
            self.board.pop()    # undo wrong move
        return False, None  # No matched move


    def fen_binary_callback(self, fen_binary):  # Main state machine controller
        self.fen_binary = np.array(fen_binary.data, dtype=np.uint8).reshape((8, 8))
        # consider only in human turn or first turn of white AI
        AI_first_turn = self.first_turn and self.AI_turn
        if AI_first_turn: # First turn of White AI
            self.get_logger().info("Update board state to:\n{}".format(str(self.board)))
            self.AI_ready = 1  # Change AI state to busy
            ## Start searching ##
            result = engine.play(self.board, limit)
            self.best_move = result.move
            self.AI_bestmove_pub.publish(String(data=self.best_move.uci()))  # publish bestmove before set status to ready
            self.AI_ready = 2  # Change AI state to ready
            self.first_turn = False
        else:
            if self.human_turn:
                if self.any_change() and self.fen_color is not None:   # Detect movement in fen_binary
                    ret, truemove = self.valid_move()
                    if ret or AI_first_turn: # change came from valid move
                        self.illegal_move = False
                        self.turn = not self.turn   # Change turn from Human -> AI
                        self.AI_ready = 1           # Change AI state to busy
                        self.board.push(truemove)   # push board with truemove
                        self.fen_binary_old = fen2binary(self.board.fen())  # update old binary after Human move
                        self.get_logger().info("Update board state to:\n{}".format(str(self.board)))
                        ## Start searching ##
                        result = engine.play(self.board, limit)
                        self.best_move = result.move
                        self.AI_bestmove_pub.publish(String(data=self.best_move.uci()))   # publish bestmove before set status to ready
                        self.AI_ready = 2  # Change AI state to ready
                    else:   # change came from illegal move
                        self.illegal_move = True
    def fen_color_callback(self, fen_color):
        self.fen_color = np.array(fen_color.data, dtype=np.uint8).reshape((8, 8))
    def execute_bestmove_callback(self, request, response):
        if self.AI_turn:
            if request.mode:    # Manual mode
                move_string = request.move
                try:
                    if chess.Move.from_uci(move_string) in self.board.legal_moves:
                        self.get_logger().info("Execute using bestmove from user input")
                        response.valid = True
                        self.best_move = chess.Move.from_uci(move_string)   # overwrite bestmove
                    else:
                        self.get_logger().warn("Manual command {} is invalid".format(move_string))
                        response.valid = False
                        return response
                except: # illegal move command
                    self.get_logger().warn("Manual command {} is invalid".format(move_string))
                    response.valid = False
                    return response
            else:               # AI mode
                if self.AI_ready == 2:  # AI in ready state
                    self.get_logger().info("Execute using bestmove from AI")
                    response.valid = True
                else:   # AI not ready yet
                    self.get_logger().warn("AI not ready yet!")
                    response.valid = False
                    return response
            # Execute bestmove
            print(self.best_move)
            self.board.push(self.best_move)
            self.get_logger().info("Update board state to:\n{}".format(str(self.board)))
            self.fen_binary_old = fen2binary(self.board.fen())  # update old binary after AI move
            self.AI_ready = 0  # reset AI state to idle
            self.turn = not self.turn  # Switch to Human turn
        else:   # not AI turn (invalid request)
            self.get_logger().warn("Not AI turn!")
            response.valid = False
        return response
    def setup_callback(self, request, response):
        self.AI_side = request.ai_is
        return response
    # def camera0_hand_callback(self, hand):
    #     if hand.data and self.turn == self.human_turn: self.hand_detected = True # only updated in human turn
    # def camera1_hand_callback(self, hand):
    #     if hand.data and self.turn == self.human_turn: self.hand_detected = True  # only updated in human turn
    def timer_callback(self):
        if self.best_move is not None: self.AI_bestmove_pub.publish(String(data=str(self.best_move.uci())))
        self.illegal_pub.publish(Bool(data=self.illegal_move))  # publish illegal action indicator
        self.pseudo_fen.publish(String(data=self.board.fen()))  # publish board state as FEN
        self.AI_ready_pub.publish(UInt8(data=self.AI_ready))    # publish AI status
        self.turn_pub.publish(UInt8(data=self.turn))            # publish turn status
        # self.get_logger().info("Update board state to:\n{}".format(str(self.board)))
def main():
    rclpy.init()
    pseudo_state_controller = PseudoStateController()
    rclpy.spin(pseudo_state_controller)
    # chessboard_detector.destroy_subscription(chessboard_detector.camera_sub) # Not need camera after init pose
    rclpy.shutdown()

if __name__ == "__main__":
    main()