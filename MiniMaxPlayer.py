from Player import Player
from copy import deepcopy


class MiniMaxPlayer(Player):
    MAX_DEPTH = 2
    INFINITY = 9999

    def max_value(self, opponent, alpha: int, beta: int, depth: int):
        if self.is_winner():
            return self.MAX_DEPTH - depth, None
        if depth == 1:
            return self.evaluate(opponent), None
        v = -self.INFINITY
        act = None
        for action in self.get_legal_actions(opponent):
            self.play(action, is_evaluating=True)
            if self.board.states.__contains__(self.board.get_state()):
                self.undo_last_action()
                continue
            r = opponent.min_value(self, alpha, beta, depth - 1)
            if v < r[0]:
                v = r[0]
                act = action
            if v >= beta:
                self.undo_last_action()
                return v, act
            alpha = max(alpha, v)
            self.undo_last_action()
        return v, act

    def min_value(self, opponent, alpha: int, beta: int, depth: int):
        if self.is_winner():
            return self.MAX_DEPTH - depth, None
        if depth == 1:
            return self.evaluate(opponent), None
        v = self.INFINITY
        act = None
        for action in self.get_legal_actions(opponent):
            self.play(action, is_evaluating=True)
            if self.board.states.__contains__(self.board.get_state()):
                self.undo_last_action()
                continue
            r = opponent.max_value(self, alpha, beta, depth - 1)
            if v > r[0]:
                v = r[0]
                act = action
            if v <= alpha:
                self.undo_last_action()
                return v, act
            beta = min(beta, v)
            self.undo_last_action()
        return v, act

    def bfs(self, opponent: Player):
        for player in [self, opponent]:
            destination = (
                self.board.get_white_goal_pieces()
                if player.color == "white"
                else self.board.get_black_goal_pieces()
            )
            visited = {}
            distances = {}
            for row in self.board.map:
                for piece in row:
                    visited[piece] = False
                    distances[piece] = self.INFINITY

            player_piece = self.board.get_piece(*player.get_position())

            queue = []
            queue.append(player_piece)
            visited[player_piece] = True
            distances[player_piece] = 0

            while queue:
                piece = queue.pop(0)

                for i in self.board.get_piece_neighbors(piece):
                    if visited[i] == False:
                        distances[i] = distances[piece] + 1
                        visited[i] = True
                        queue.append(i)

            min_distance = self.INFINITY
            for piece, dist in distances.items():
                if piece in destination:
                    if dist < min_distance:
                        min_distance = dist

            if player == self:
                self_distance = min_distance
            else:
                opponent_distance = min_distance

        return self_distance, opponent_distance

    def evaluate(self, opponent):
        self_distance, opponent_distance = self.bfs(opponent)
        if self.MAX_DEPTH % 2 == 0:
            total_score = self_distance - opponent_distance
        else:
            total_score = opponent_distance - self_distance
        return total_score

    def get_best_action(self, opponent):
        best_action_value, best_action = self.max_value(opponent, -self.INFINITY, self.INFINITY, self.MAX_DEPTH)
        return best_action
