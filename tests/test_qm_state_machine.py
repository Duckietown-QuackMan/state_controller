import unittest
# import sys
# import os
# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from src.qm_state_machine_impl import QMStateMachine, State

import time

class TestQMStateMachine(unittest.TestCase):

    def setUp(self):
        self.state_machine = QMStateMachine()

    def test_initial_state(self):
        self.assertEqual(self.state_machine.state, State.IDLE)
        self.assertEqual(self.state_machine.score, 0)

    def test_idle_to_cp_detection(self):
        self.state_machine.set_game_state(1)  # Start the game
        self.state_machine.step()
        self.assertEqual(self.state_machine.state, State.CP_DETECTION)

    def test_cp_detection_to_game_over(self):
        self.state_machine.set_game_state(1)  # Start the game
        self.state_machine.step()

        # Simulate timeout for checkpoints
        time.sleep(1)  # Assume time delta is small for this test
        self.state_machine.set_cp_status(-1)  # Trigger step logic
        self.state_machine.cp_delta_time_elapsed = True
        self.state_machine.step()
        self.assertEqual(self.state_machine.state, State.GAME_OVER)

    def test_cp_detection_to_game_won(self):
        self.state_machine.set_game_state(1)  # Start the game
        self.state_machine.step()

        # Simulate collecting all checkpoints
        for i in range(1, 6):  # TOTAL_CHECKPOINTS = 5
            self.state_machine.set_cp_status(i)

        self.state_machine.step()
        self.assertEqual(self.state_machine.state, State.GAME_WON)
        self.assertEqual(self.state_machine.score, 5)

    def test_checkpoint_score_update(self):
        self.state_machine.set_game_state(1)  # Start the game
        self.state_machine.step()
        self.state_machine.set_cp_status(1)
        self.assertEqual(self.state_machine.score, 1)

        # Adding a duplicate checkpoint should not increase the score
        self.state_machine.set_cp_status(1)
        self.assertEqual(self.state_machine.score, 1)

    def test_outputs(self):
        self.state_machine.set_game_state(1)  # Start the game
        self.state_machine.step()
        outputs = self.state_machine.get_outputs()
        self.assertEqual(outputs, {"game-over": False, "game-won": False})

        # Simulate game over
        self.state_machine.cp_delta_time_elapsed = True
        self.state_machine.step()
        outputs = self.state_machine.get_outputs()
        self.assertEqual(outputs, {"game-over": True, "game-won": False})

if __name__ == '__main__':
    unittest.main()
