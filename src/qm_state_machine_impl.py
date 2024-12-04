from enum import IntEnum
import time

class State(IntEnum):
    IDLE = 0
    CP_DETECTION = 1
    GAME_OVER = 2
    GAME_WON = 3

# TODO: move these to a config file
TIME_DELTA_CHECKPOINTS = 30
TOTAL_CHECKPOINTS = 5


class QMStateMachine:
    def __init__(self):
        # external events
        self.start: bool = False
        self.cp_delta_time_elapsed: bool = False
        self.all_cp_detected: bool = False

        # internal state
        self.state: State = State.IDLE


        # create a set to contain all the identified checkpoints
        self.checkpoints = set()
        # player score
        # right now, simply add 1 for each checkpoint detected
        self.score = 0
        
        # the time at which the last checkpoint was detected
        # TODO: could initialise as none, and get it to be set upon the first checkpoint detection
        # currently, this is reset inside the handle_idle function
        self.last_cp_time = time.time()


    def step(self):
        # update the state, after having delt with the current state's implication and thus its outcome
        if self.state == State.IDLE:
            self.state = self.handle_idle()
        elif self.state == State.CP_DETECTION:
            self.state = self.handle_cp_detection()
        elif self.state == State.GAME_OVER:
            self.state = self.handle_game_over()
        elif self.state == State.GAME_WON:
            self.state = self.handle_game_won()
        return self.get_outputs()
    
    def get_outputs(self):
        # TODO
        if self.state == State.IDLE:
            # TOASK: 
            # - do we want to also publish the score this way, or thorugh the callback, upon checkpoint detection?
            #   - this would mean we can streamline the cb_checkpoint. 
            #       But this would mean we mustmake sure that the score is only published when it is updated, 
            #       and not every time the step function is called, such that the gamemaster is not overwhelmed with messages
            # - do we want to publish the fact that we are currently in idle or cp_detect state?
            return{"game-over": False, "game-won": False}#, "score": self.score}
        if self.state == State.CP_DETECTION:
            return{"game-over": False, "game-won": False}
        if self.state == State.GAME_OVER:
            return{"game-over": True, "game-won": False}
        if self.state == State.GAME_WON:
            return{"game-over": False, "game-won": True}
          
    def handle_idle(self):
        next_state = State.IDLE
        if self.start:
            self.last_cp_time = time.time()
            next_state = State.CP_DETECTION
        return next_state
    
    def handle_cp_detection(self):
        next_state = State.CP_DETECTION
        curr_time = time.time()
        # check if the time elapsed since the last checkpoint detection is greater than the threshold
        if self.score!= 0 and curr_time - self.last_cp_time > TIME_DELTA_CHECKPOINTS:
            self.cp_delta_time_elapsed = True 
        if self.cp_delta_time_elapsed:
            next_state = State.GAME_OVER
        elif self.all_cp_detected:
            next_state = State.GAME_WON
        return next_state
    
    def handle_game_over(self):
        next_state = State.GAME_OVER
        return next_state  
    
    def handle_game_won(self):
        next_state = State.GAME_WON
        return next_state  
    
    def set_game_state(self, val: str) -> None:
        self.start = val == "RUNNING"

    # TODO: 
    # - if we change the message type to stamped int and bool, we could use 
    # the message time, to evaluate at what time the checkpoint was detected
    # TOASK: 
    # - baseically, this function gets called everytime a checkpoint gets publisehd. 
    # Isn't there a risk that it get soverwhlemed and called too many times, 
    # preventing the rest from occuring?
    # - we should maybe make sure that the checkpoints are not published at a too high frequency, 
    # or woudl this not be a problem?
    def set_cp_status(self, val: int) -> None:
        curr_time = time.time()
        # check if the time elapsed since the last checkpoint detection is greater than the threshold
        if self.score!= 0 and curr_time - self.last_cp_time > TIME_DELTA_CHECKPOINTS:
            self.cp_delta_time_elapsed = True    
        # discovered a new checkpoint, within the time window
        elif val not in self.checkpoints:
            self.score += 1
            self.last_cp_time = curr_time
            self.checkpoints.add(val)

            if len(self.checkpoints) == TOTAL_CHECKPOINTS:
                self.all_cp_detected = True
            # redundant, would never occur
            # self.cp_delta_time_elapsed = False
    
    def get_score(self) -> int:
        return self.score