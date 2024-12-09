#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String

from state_machine_impl import StateMachine

class StateMachineNode:

    def __init__(self):

        self.state_machine = StateMachine()

        self.setup_params()
        self.setup_publishers_and_subscribers()
        step_timer = rospy.Timer(rospy.Duration(1), self.step)

    def step(self, _event):
        outputs = self.state_machine.step()
        self.pub_lane_following.publish(Bool(outputs["lf"]))
        self.pub_x_sec_go.publish(Bool(outputs["x-sec-go"]))
        self.pub_game_over.publish(Bool(outputs["game-over"]))
    
    def print_info(self) -> None:
        print()
        print()
        rospy.loginfo(f"Running State Machine Node")
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

        # topics params
        self.name_sub_game_state = get_rosparam("~topics/sub/game_state")
        self.name_sub_ghost_bot = get_rosparam("~topics/sub/ghost_bot")
        self.name_sub_ghost_bot_b = get_rosparam("~topics/sub/ghost_bot_back")
        self.name_sub_quack_man = get_rosparam("~topics/sub/quack_man")
        self.name_sub_x_sec = get_rosparam("~topics/sub/x_sec")
        self.name_sub_x_sec_navigating = get_rosparam("~topics/sub/x_sec_navigating")

        self.name_pub_lane_following = get_rosparam("~topics/pub/lane_following")
        self.name_pub_x_sec_go = get_rosparam("~topics/pub/x_sec_go")
        self.name_pub_game_over = get_rosparam("~topics/pub/game_over")
        
    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node.
        """
        self.sub_game_state       = rospy.Subscriber(self.name_sub_game_state,       String, self.game_state_cb,       queue_size=10)
        self.sub_ghost_bot        = rospy.Subscriber(self.name_sub_ghost_bot,        Bool,   self.ghost_bot_cb,        queue_size=10)
        self.sub_ghost_bot_b      = rospy.Subscriber(self.name_sub_ghost_bot_b,      Bool,   self.ghost_bot_b_cb,      queue_size=10)
        self.sub_quack_man        = rospy.Subscriber(self.name_sub_quack_man,        Bool,   self.quack_man_cb,        queue_size=10)
        self.sub_x_sec            = rospy.Subscriber(self.name_sub_x_sec,            Bool,   self.x_sec_cb,            queue_size=10)
        self.sub_x_sec_navigating = rospy.Subscriber(self.name_sub_x_sec_navigating, Bool,   self.x_sec_navigating_cb, queue_size=10)

        self.pub_lane_following = rospy.Publisher(self.name_pub_lane_following, Bool, queue_size=10)
        self.pub_x_sec_go       = rospy.Publisher(self.name_pub_x_sec_go,       Bool, queue_size=10)
        self.pub_game_over      = rospy.Publisher(self.name_pub_game_over,      Bool, queue_size=10)

    def game_state_cb(self, msg):
        self.state_machine.set_game_state(msg.data)

    def ghost_bot_cb(self, msg):
        self.state_machine.set_ghost_bot(msg.data)

    def ghost_bot_b_cb(self, msg):
        self.state_machine.set_ghost_bot_b(msg.data)

    def quack_man_cb(self, msg):
        self.state_machine.set_quack_man(msg.data)

    def x_sec_cb(self, msg):
        self.state_machine.set_x_sec(msg.data)

    def x_sec_navigating_cb(self, msg):
        self.state_machine.set_x_sec_navigating(msg.data)



if __name__ == "__main__":
    rospy.init_node("state_machine", anonymous=True)

    state_machine = StateMachineNode()
    state_machine.print_info()

    rospy.spin()