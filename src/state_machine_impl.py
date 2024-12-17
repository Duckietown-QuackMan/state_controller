from enum import IntEnum
import time
from typing import Callable


class State(IntEnum):
    IDLE = 0
    LANE_FOLLOWING = 1
    WAIT_TRAFFIC = 2
    X_SEC_NAV = 3
    WAIT_X_SEC = 4
    GAME_OVER = 5
    GAME_WON = 6


TRAFFIC_WAIT_TIME_S = 5
X_SEC_WAIT_TIME_S = 5

class StateMachine:
    def __init__(self, priority: bool, lane_following_cb : Callable[[bool], None]):
        self.start: bool = False
        self.game_over: bool = False
        self.game_won: bool = False
        self.ghost_bot_b: bool = False
        self.quack_man: bool = False
        self.x_sec: bool = False
        self.x_sec_navigating: bool = False
        self.priority = priority

        self.wait_start_time = None

        self.state: State = State.IDLE

        self.lane_following_cb = lane_following_cb
    

    def step(self):
        print(self.state)
        if self.state == State.IDLE:
            self.state = self.handle_idle()
        elif self.state == State.LANE_FOLLOWING:
            self.state = self.handle_lf()
        elif self.state == State.WAIT_TRAFFIC:
            self.state = self.handle_wait_traffic()
        elif self.state == State.X_SEC_NAV:
            self.state = self.handle_x_sec_nav()
        elif self.state == State.WAIT_X_SEC:
            self.state = self.handle_x_sec_wait()
        elif self.state == State.GAME_OVER:
            self.state = self.handle_game_over()
        elif self.state == State.GAME_WON:
            self.state = self.handle_game_won()
        return self.get_outputs()
    
    def get_outputs(self):
        if self.state == State.IDLE:
            return {"x-sec-go": False, "game-over": False}
        if self.state == State.LANE_FOLLOWING:
            return {"x-sec-go": False, "game-over": False}
        if self.state == State.WAIT_TRAFFIC:
            return {"x-sec-go": False, "game-over": False}
        if self.state == State.X_SEC_NAV:
            return {"x-sec-go": True, "game-over": False}
        if self.state == State.WAIT_X_SEC:
            return {"x-sec-go": False, "game-over": False}
        if self.state == State.GAME_OVER:
            return {"x-sec-go": False, "game-over": True}
        if self.state == State.GAME_WON:
            return {"x-sec-go": False, "game-over": False}
    
    def handle_idle(self):
        next_state = State.IDLE
        if self.start:
            # TODO: check if quackman detected?
            next_state = State.LANE_FOLLOWING
            self.lane_following_cb(True)
        return next_state

    def handle_lf(self):
        next_state = State.LANE_FOLLOWING
        if self.x_sec:
            next_state = State.WAIT_X_SEC
            self.x_sec = False
            self.wait_start_time = time.time()
            self.lane_following_cb(False)
        if self.ghost_bot_b:
            next_state = State.WAIT_TRAFFIC
            self.wait_start_time = time.time()
            self.lane_following_cb(False)
        if self.quack_man or self.game_over:
            next_state = State.GAME_OVER
            self.lane_following_cb(False)
        if self.game_won:
            next_state = State.GAME_WON
            self.lane_following_cb(False)
        return next_state

    def handle_wait_traffic(self):
        next_state = State.WAIT_TRAFFIC
        if self.wait_start_time + TRAFFIC_WAIT_TIME_S < time.time():
            next_state = State.LANE_FOLLOWING
            self.ghost_bot_b = False
            self.wait_start_time = None
            self.lane_following_cb(True)
        if self.quack_man or self.game_over:
            next_state = State.GAME_OVER
        if self.game_won:
            next_state = State.GAME_WON
        # TODO: check if ghost bot is still there, DEPENDENT ON PRIORITY LEVEL OF BOT
        return next_state

    def handle_x_sec_nav(self):
        next_state = State.X_SEC_NAV
        if not self.x_sec_navigating:
            next_state = State.LANE_FOLLOWING
            self.lane_following_cb(True)
        if self.quack_man or self.game_over:
            next_state = State.GAME_OVER
        if self.game_won:
            next_state = State.GAME_WON
        return next_state
    
    def handle_x_sec_wait(self):
        next_state = State.WAIT_X_SEC
        if self.wait_start_time + TRAFFIC_WAIT_TIME_S < time.time():
            next_state = State.X_SEC_NAV
            self.x_sec_navigating = True
            self.wait_start_time = None
        if self.quack_man or self.game_over:
            next_state = State.GAME_OVER
        if self.game_won:
            next_state = State.GAME_WON
        return next_state

    def handle_game_over(self):
        next_state = State.GAME_OVER
        return next_state

    def handle_game_won(self):
        next_state = State.GAME_WON
        return next_state

    def set_game_state(self, val: str) -> None:
        if val == "RUNNING":
            self.start = True
        elif val == "GAME_OVER":
            self.game_over = True
        elif val == "GAME_WON":
            self.game_won = True

    def set_ghost_bot(self, val: bool) -> None:
        if self.priority:
            return
        if self.state == State.WAIT_X_SEC:
            self.wait_start_time = time.time()

    def set_ghost_bot_b(self, val: bool) -> None:
        if self.state == State.LANE_FOLLOWING or self.state == State.WAIT_TRAFFIC:
            self.ghost_bot_b = val

    def set_quack_man(self, val: bool) -> None:
        self.quack_man = val

    def set_x_sec(self, val: bool) -> None:
        if self.state == State.LANE_FOLLOWING:
            self.x_sec = val

    
    def set_x_sec_navigating(self, val: bool) -> None:
        print(f"setting x-sec-navigating flag through cb to: {val}")
        self.x_sec_navigating = val

    

