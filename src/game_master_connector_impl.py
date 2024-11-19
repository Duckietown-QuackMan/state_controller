from websocket import WebSocketApp
import json

class GameMasterConnector:
    def __init__(self, game_state_callback: callable[str, None], bot_name: str, game_master_connection_url: str):
        self.game_state_callback = game_state_callback
        self.bot_name = bot_name
        self.ws = WebSocketApp(game_master_connection_url,
                               on_message=self.on_message,
                               on_close=self.on_close,
                               on_open=self.on_open)
        self.ws.run_forever(reconnect=5)

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
            "type": "CHECKPOINT"
            "data": {
                "all_collected": True
            }
        }
        self.ws.send(json.dumps(msg_dict))

    def on_message(self, ws, message):
        msg_dict = json.loads(message)
        if msg_dict["type"] == "GAME_STATE":
            self.game_state_callback(msg_dict["data"]["state"])
        else:
            print("Unknown message type: ", msg_dict["type"])

    def on_close(self, ws):
        print("Connection to Game Master closed")

    def on_open(self, ws):
        print("Connection to Game Master opened")

