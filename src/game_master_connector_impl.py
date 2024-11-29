from typing import Callable
from websocket import WebSocketApp
import rel 
import json

class GameMasterConnector:
    def __init__(self, game_state_callback: Callable[[str], None], bot_name: str, game_master_connection_url: str):
        self.game_state_callback = game_state_callback
        self.bot_name = bot_name
        print("Connecting to Game Master at: ", game_master_connection_url)
        self.ws = WebSocketApp(game_master_connection_url,
                               on_message=self.on_message,
                               on_close=self.on_close,
                               on_open=self.on_open)
        print("Connected to Game Master")
        self.ws.run_forever(dispatcher=rel, reconnect=5)
        rel.signal(2,rel.abort)
        rel.dispatch()
        print("Running connection")

    def send_quackman_detected(self):
        msg_dict = {
            "bot": self.bot_name,
            "type": "DETECTION",
            "data": {
                "detected": True
            }
        }
        self.ws.send(json.dumps(msg_dict))

    def send_score(self, score):
        msg_dict = {
            "bot": self.bot_name,
            "type": "SCORE",
            "data": {
                "score": score
            }
        }
        self.ws.send(json.dumps(msg_dict))

    def send_all_checkpoints_collected(self):
        msg_dict = {
            "bot": self.bot_name,
            "type": "CHECKPOINT",
            "data": {
                "all_collected": True
            }
        }
        self.ws.send(json.dumps(msg_dict))

    def send_checkpoint_timeout(self):
        msg_dict = {
            "bot": self.bot_name,
            "type": "TIMEOUT",
            "data": {
                "checkpointTimeout": True
            }
        }
        self.ws.send(json.dumps(msg_dict))

    def on_message(self, ws, message):
        print("Received message from Game Master: ", message)
        msg_dict = json.loads(message)
        if msg_dict["type"] == "GAME_STATE":
            print("Received game state: ", msg_dict["data"]["state"])
            self.game_state_callback(msg_dict["data"]["state"])
        else:
            print("Unknown message type: ", msg_dict["type"])

    def on_close(self, ws):
        print("Connection to Game Master closed")

    def on_open(self, ws):
        print("Connection to Game Master opened")

