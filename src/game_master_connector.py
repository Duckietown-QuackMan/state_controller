#!/usr/bin/env python3
import os

import rospy
from std_msgs.msg import Bool, Int32, String

from game_master_connector_impl import GameMasterConnector

class GameMasterConnectorNode:

    def __init__(self):
        rospy.init_node("game_master_connector", anonymous=True)
        print("Game Master Connector Node Initialised")

        self.setup_params()

        self.game_master_connector = GameMasterConnector(self.game_state_update_cb, self.vehicle_name, f"{self.game_master_url}/{self.bot_type}")
        
        self.setup_publishers_and_subscribers()
        print("Game Master Connector Node Setup Complete")

    
    def print_info(self) -> None:
        print()
        print()
        rospy.loginfo("Running game master connector")
        print()
        print()

    def setup_params(self) -> None:
        """
        Setup the parameters for the node reading them from the ROS parameter server
        - self.name_sub_all_chekpoints_collected: name of the channel indicating if all checkpoints were collected, only applicable for the QuackMan
        - self.name_sub_score_update: name of the channel publishing the current score, only applicable for the QuackMan
        - self.name_sub_checkpoint_timeout: name of the channel indicating if the checkpoint timeout was reached, only applicable for the QuackMan
        - self.name_sub_game_over: name of the channel publishing if the game is over
        - self.name_pub_game_state: name of the channel propagating game state messages
        """

        def get_rosparam(name):
            if rospy.has_param(name):
                param = rospy.get_param(name)
            else:
                txt_error = f"Parameter '{name}' is not set"
                rospy.logerr(txt_error)
                raise KeyError(txt_error)
            return param
        self.bot_type = rospy.get_param("~bot_type")

        self.vehicle_name = get_rosparam("~vehicle_name")
        
        # bot type config, either 'quackman' or 'ghostbot'
        # self.bot_type = get_rosparam("~bot_type")


        # game master config
        self.game_master_url = get_rosparam("~game_master/url")

        # topic params
        self.name_sub_all_checkpoints_collected = get_rosparam("~topics/sub/all_checkpoints_collected")
        self.name_sub_score_update = get_rosparam("~topics/sub/score_update")
        self.name_sub_checkpoint_timeout = get_rosparam("~topics/sub/checkpoint_timeout")
        self.name_sub_game_over = get_rosparam("~topics/sub/game_over")

        self.name_pub_game_state = get_rosparam("~topics/pub/game_state")

    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node
        """
        self.sub_all_checkpoints_collected = rospy.Subscriber(self.name_sub_all_checkpoints_collected, Bool, self.all_chekpoints_collected_cb, queue_size=10)
        self.sub_score_update = rospy.Subscriber(self.name_sub_score_update, Int32, self.score_update_cb, queue_size=10)
        self.sub_checkpoint_timeout = rospy.Subscriber(self.name_sub_checkpoint_timeout, Bool, self.checkpoint_timeout_cb, queue_size=10)
        self.sub_game_over = rospy.Subscriber(self.name_sub_game_over, Bool, self.game_over_cb, queue_size=10)

        self.pub_game_state = rospy.Publisher(self.name_pub_game_state, String, queue_size=10)
    
    def game_state_update_cb(self, game_state: str) -> None:
        print(f"Game state update: {game_state}")
        msg: String = String()
        msg.data = game_state
        self.pub_game_state.publish(msg)
    
    def all_chekpoints_collected_cb(self, msg: Bool) -> None:
        if msg.data:
            self.game_master_connector.send_all_checkpoints_collected()

    def checkpoint_timeout_cb(self, msg: Bool) -> None:
        if msg.data:
            self.game_master_connector.send_checkpoint_timeout()
    
    def score_update_cb(self, msg: Int32) -> None:
        self.game_master_connector.send_score(msg.data)

    def game_over_cb(self, msg: Bool) -> None:
        if msg.data:
            self.game_master_connector.send_quackman_detected()


if __name__ == "__main__":
    # rospy.init_node("game_master_connector", anonymous=True)

    game_master_connector = GameMasterConnectorNode()
    game_master_connector.print_info()

    rospy.spin()
        