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
        self.pub_cp_timeout.publish(Bool(outputs["game-over"]))
        self.pub_all_checkpoints_collected.publish(Bool(outputs["game-won"]))
    
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
        - self.name_sub_checkpoint: name of the channel containing the AprilTag IDs detected
        - self.name_pub_score_updates: name of the channel publishing the current QuackMan score
        - self.name_pub_all_checkpoints_collected: If set to true, all checkpoints were collected
        - self.name_pub_cp_timeout: if set to true the nect checkpoint was not deteced fast enough 
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
        
    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node.
        """
        self.sub_game_state       = rospy.Subscriber(self.name_sub_game_state,      String,  self.game_state_cb,      queue_size=1)
        self.sub_checkpoint       = rospy.Subscriber(self.name_sub_checkpoint,      Int32,  self.cb_checkpoint,                     queue_size=1)
        
        self.pub_score_updates = rospy.Publisher(self.name_pub_score_updates,       Int32,                                          queue_size=1)
        self.pub_all_checkpoints_collected = rospy.Publisher(self.name_pub_all_checkpoints_collected,       Bool,                   queue_size=1)
        self.pub_cp_timeout      = rospy.Publisher(self.name_pub_cp_timeout,          Bool,                                           queue_size=1)

    def cb_checkpoint(self, msg: int) -> None:
        self.state_machine.set_cp_status(msg.data)
        # Publishe the score, if updated
        if self.score != self.state_machine.get_score():
            self.score = self.state_machine.get_score()
            msg = Int32()
            msg.data = self.score
            self.pub_score_updates.publish(msg)

    def game_state_cb(self, msg: String) -> None:
        self.state_machine.set_game_state(msg.data)
    
if __name__ == "__main__":
    rospy.init_node("qm_state_machine", anonymous=True)

    state_machine = QMStateMachineNode()
    state_machine.print_info()

    rospy.spin()
