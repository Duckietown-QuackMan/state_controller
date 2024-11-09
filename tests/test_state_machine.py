import unittest
import time

from src.state_machine_impl import StateMachine, State, TRAFFIC_WAIT_TIME_S, X_SEC_WAIT_TIME_S
import unittest
import time

class TestStateMachine(unittest.TestCase):
    
    def setUp(self):
        # Create a new StateMachine instance before each test to ensure independence
        self.machine = StateMachine()
        self.machine.set_game_state(0)  # Ensure machine is in IDLE state by default
    
    def test_idle_to_lane_following(self):
        # Set the start signal and check transition to LANE_FOLLOWING
        self.machine.set_game_state(1)  # Start the machine
        self.machine.step()  # Run the state machine step
        self.assertEqual(self.machine.state, State.LANE_FOLLOWING)
    
    def test_lane_following_to_game_over(self):
        # Start the machine and transition to LANE_FOLLOWING
        self.machine.set_game_state(1)
        self.machine.step()
        # Trigger QuackMan and check transition to GAME_OVER
        self.machine.set_quack_man(True)
        self.machine.step()
        self.assertEqual(self.machine.state, State.GAME_OVER)
        self.assertTrue(self.machine.get_outputs()["game-over"])

    def test_lane_following_to_wait_traffic_and_back(self):
        # Start the machine and transition to LANE_FOLLOWING
        self.machine.set_game_state(1)
        self.machine.step()
        # Trigger GhostBot b and transition to WAIT_TRAFFIC
        self.machine.set_ghost_bot_b(True)
        self.machine.step()
        self.assertEqual(self.machine.state, State.WAIT_TRAFFIC)
        
        # Simulate the traffic wait time and check if it transitions back to LANE_FOLLOWING
        time.sleep(TRAFFIC_WAIT_TIME_S)
        self.machine.step()  # Run another step
        self.assertEqual(self.machine.state, State.LANE_FOLLOWING)

    def test_lane_following_to_x_sec_nav(self):
        # Start the machine and transition to LANE_FOLLOWING
        self.machine.set_game_state(1)
        self.machine.step()
        # Trigger x-sec and check transition to X_SEC_NAV
        self.machine.set_x_sec(True)
        self.machine.step()
        self.assertEqual(self.machine.state, State.X_SEC_NAV)
        self.assertTrue(self.machine.get_outputs()["x-sec-go"])

    def test_x_sec_nav_to_game_over(self):
        # Start the machine and transition to X_SEC_NAV
        self.machine.set_game_state(1)
        self.machine.set_x_sec(True)
        self.machine.step()
        # Trigger QuackMan and check transition to GAME_OVER
        self.machine.set_quack_man(True)
        self.machine.step()
        self.assertEqual(self.machine.state, State.GAME_OVER)
        self.assertTrue(self.machine.get_outputs()["game-over"])

    def test_x_sec_nav_to_wait_x_sec_and_back(self):
        # Start the machine and transition to X_SEC_NAV
        self.machine.set_game_state(1)
        self.machine.set_x_sec(True)
        self.machine.step()
        # Trigger GhostBot and transition to WAIT_X_SEC
        self.machine.set_ghost_bot(True)
        self.machine.step()
        self.assertEqual(self.machine.state, State.WAIT_X_SEC)
        
        # Simulate the X_SEC wait time and check if it transitions back to LANE_FOLLOWING
        time.sleep(X_SEC_WAIT_TIME_S)
        self.machine.step()  # Run another step
        self.assertEqual(self.machine.state, State.LANE_FOLLOWING)

    def test_x_sec_nav_to_lane_following_when_x_sec_navigating_false(self):
        # Start the machine and transition to X_SEC_NAV
        self.machine.set_game_state(1)
        self.machine.set_x_sec(True)
        self.machine.step()
        # Disable x_sec_navigating and verify transition to LANE_FOLLOWING
        self.machine.set_x_sec_navigating(False)
        self.machine.step()
        self.assertEqual(self.machine.state, State.LANE_FOLLOWING)
    
    def test_game_over_no_transition(self):
        # Trigger QuackMan to directly enter GAME_OVER state
        self.machine.set_quack_man(True)
        self.machine.step()
        self.assertEqual(self.machine.state, State.GAME_OVER)
        self.assertTrue(self.machine.get_outputs()["game-over"])
        
        # Ensure GAME_OVER state remains unchanged
        self.machine.step()  # Run another step
        self.assertEqual(self.machine.state, State.GAME_OVER)

if __name__ == '__main__':
    unittest.main()
