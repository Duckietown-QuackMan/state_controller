#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int32, String

from qm_state_machine_impl import QMStateMachine

class QMStateMachineNode:

    def __init__(self):
        

        self.state_machine = QMStateMachine()

        self.setup_params()
        self.setup_publishers_and_subscribers()
        step_timer = rospy.Timer(rospy.Duration(1), self.step)

    def step(self, _event):
        print("state of state_machine:", self.state_machine.state)
        outputs = self.state_machine.step()
        # TO ASK: 
        # - I think this may be redundant, as we can simply publish the state of the game in the cb_checkpoint
        # RESOLVED:I think that done this way is fine, as we can more easily differentiate parts
        # - do we want to publish the score here or through the callback, upon checkpoint detection? 
        # TODO: - ONLY PUBLISH IF DIFFERENT FROM BEFORE
        self.pub_cp_timeout.publish(Bool(outputs["game-over"]))
        # BUG A:
        # - as it currently is, it works. BUT: we get the LEDS set to red, if the timeout is reached or if the quackamn is detected
        # - this is because we are publishing the game-over state in both cases.
        # - we need to find a way to differentiate between the two cases
        self.pub_all_checkpoints_collected.publish(Bool(outputs["game-won"]))

        # TODO: find a way to cleanly stop the game
    
    def print_info(self) -> None:
        print()
        print()
        rospy.loginfo(f"Running Quack Man State Machine Node")
        print()
        print()

    def setup_params(self) -> None:
        """
        Setup the parameters for the node reading them from the ROS parameter server.
        - self.name_sub_game_state: name of the game state channel
        - self.name_sub_ghost_bot: name of the channel informing if a GhostBot is seen
        - self.name_sub_ghost_bot_b: name of the channel informing if the back a GhostBot is seen
        - self.name_sub_quack_man: name of the channel informing if the QuackMan is seen
        - self.name_sub_x_sec: name of the channel for detecting x-sec
        - self.name_pub_lane_following: if set to true the lane following node should start driving
        - self.name_pub_x_sec_go: if set set to true the x-sec navigation should start navigating through the cross section
        - self.name_pub_game_over: if set to true the QuackMan was detected and the game is over
        """

        def get_rosparam(name):
            if rospy.has_param(name):
                param = rospy.get_param(name)
            else:
                txt_error = f"Parameter '{name}' is not set."
                rospy.logerr(txt_error)
                raise KeyError(txt_error)
            return param
        
        # score
        self.score = 0

        # topics params
        self.name_sub_game_state = get_rosparam("~topics/sub/game_state")
        self.name_sub_checkpoint = get_rosparam("~topics/sub/checkpoint")

        self.name_pub_score_updates = get_rosparam("~topics/pub/score_update")
        self.name_pub_all_checkpoints_collected = get_rosparam("~topics/pub/all_checkpoints_collected")
        self.name_pub_cp_timeout = get_rosparam("~topics/pub/checkpoint_timeout")
        # self.name_pub_quackman_found = get_rosparam("~topics/pub/quack_man/quack_man")

        
    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node.
        """
        self.sub_game_state       = rospy.Subscriber(self.name_sub_game_state,      String,  self.game_state_cb,      queue_size=10)
        self.sub_checkpoint       = rospy.Subscriber(self.name_sub_checkpoint,      Int32,  self.cb_checkpoint,                     queue_size=10)

        self.pub_score_updates = rospy.Publisher(self.name_pub_score_updates,       Int32,                                          queue_size=10)
        self.pub_all_checkpoints_collected = rospy.Publisher(self.name_pub_all_checkpoints_collected,       Bool,                   queue_size=10)
        self.pub_cp_timeout      = rospy.Publisher(self.name_pub_cp_timeout,          Bool,                                           queue_size=10)
        # self.pub_quackman_found = rospy.Publisher(self.name_pub_quackman_found,       Bool,                                           queue_size=10)


    def cb_checkpoint(self, msg: int) -> None:
        self.state_machine.set_cp_status(msg.data)
        # Publishe the score, if updated
        # TOASK: - do we want to publish the score this way, or through the step function?
        if self.score != self.state_machine.get_score():
            self.score = self.state_machine.get_score()
            msg = Int32()
            msg.data = self.score
            self.pub_score_updates.publish(msg)

        # # check if all checkpoints were collected
        # # TOASK: - we don't need to do this here right. We can simply do this above, upon call of the step function
        # if self.state_machine.all_checkpoints_collected():
        #     self.pub_all_checkpoints_collected.publish(Bool(True))
        #     # TODO: find a way to cleanly stop the game
        #     self.state_machine.set_game_state(False)

    # TODO: need to find a way to publish the game over in case of no checkpoints collected within time frame
    # RESOLVED: we can simply do this above, upon call of the step function

    def game_state_cb(self, msg: String) -> None:
        self.state_machine.set_game_state(msg.data)
    



if __name__ == "__main__":
    rospy.init_node("qm_state_machine", anonymous=True)

    state_machine = QMStateMachineNode()
    state_machine.print_info()

    rospy.spin()