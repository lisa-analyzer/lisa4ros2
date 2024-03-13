#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python

from module89.srv import FindBestMove
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

import chess.engine

engine = chess.engine.SimpleEngine.popen_uci("stopfish")    # available options: Debug Log File, Threads, Hash, Clear Hash, Ponder, MultiPV, Skill Level, Move Overhead, Slow Mover, nodestime, UCI_Chess960, UCI_AnalyseMode, UCI_LimitStrength, UCI_Elo, UCI_ShowWDL, SyzygyPath, SyzygyProbeDepth, Syzygy50MoveRule, SyzygyProbeLimit, Use NNUE, EvalFile
engine.configure({"Threads": 6})

class ChessEngine(Node):
    def __init__(self):
        super().__init__('chess_engine')
        self.engine_srv = self.create_service(FindBestMove, 'find_best_move', self.find_best_move_callback)
    def find_best_move_callback(self, request, response):

        fen = request.fen.data
        board = chess.Board(fen)
        limit = chess.engine.Limit(time=3)
        result = engine.play(board, limit)
        bestmove = String()
        bestmove.data = str(result.move)
        print(result.move)
        response.bestmove = bestmove
        return response

def main():
    rclpy.init()
    engine_service = ChessEngine()
    rclpy.spin(engine_service)
    rclpy.shutdown()

if __name__ == "__main__":
    main()