# state_controller
ROS package for high level control of the GhostBots.

## Nodes
These package consists of the following nodes, each having a specific functionality.

### state_machine
The state machine node is responsible to manage the state of the GhostBots and send out commands to other nodes depending on the current state.

This diagram shows the state machine implemented in the "state_machine" node.
```mermaid
graph LR;
wx(wait x-sec)
xn(x-sec-nav)
lf(lane-following)
go(game-over)
gw(game-won)
i(idle)
wt(wait-traffic)
lf -- "GhostBot b" --> wt --> lf
lf -- x-sec --> xn
xn -- "GhostBot" --> wx --> xn
xn -- !XSecNavigating --> lf
i -- start --> lf
wx -- QuackMan, GameOver --> go
xn -- QuackMan, GameOver --> go
i -- QuackMan --> go
lf -- QuackMan, GameOver --> go
wt -- QuackMan, GameOver --> go
go --> go
wx -- GameWon --> gw
xn -- GameWon --> gw
lf -- GameWon --> gw
wt -- GameWon --> gw
gw --> gw
```
:exclamation: add `game_state` (received from game master) transitions to game over

:exclamation: add `game-won` state

:warning: Consider transitions from lane following to wait states

The node is listening on these channels for boolean flags for the inputs
- `ghost_bot`
- `ghost_bot_back`
- `quack_man`
- `x_sec`
- `x_sec_navigating`

The node is listening on the `game_state` channel for the current game state.
The game state is published by the GameMasterConnector as string and can be one of the following values
- `IDLE`: The game is not running, waiting for bots to join.
- `RUNNING`: The game is running, bots can send status updates, QuackMan can send the current score.
- `GAME_OVER`: QuackMan has been caught by a bot.
- `GAME_WON`: QuackMan has collected all checkpoints and won the game.


The node will send out boolean flag commands on these channels
- `lane_following`, if set to true the lane following node should start driving
- `x_sec_go`, if set to true the x-sec navigation should start navigating through the cross-section
- `game_over`, if set to true the QuackMan was detected and the game is over

To run the test cases execute `python -m unittest discover tests` at the project root.
### qm_state_machine
This node is responsible to manage the state of the QuackMan and send out commands to other nodes depending on the current state.

This diagram shows the state machine implemented in the "qm_state_machine" node.
```mermaid
graph LR;
cd(checkpoint detection)
go(game-over)
gw(game-won)
i(idle)
i -- start --> cd
cd -- timeout --> go
cd -- collected all checkpoint --> gw
gw --> gw
go --> go
```
:exclamation: maybe add as a transition state when quackman is detected by ghostbots
### GameMasterConnector
This node is responsible for the communication with the game master.
It is reusable for the GhostBots and the QuackMan by starting it with the appropriate launch file.
- `game_master_connector_quackmann.launch`
- `game_master_connector_ghostbot.launch`

How communication with the game master works is documented [here](https://github.com/Duckietown-QuackMan/game_master).

This node will publish the game state as string on the topic `game_state`.
The node listens on these channels
- `all_chekpoints_collected`
- `score_update`
- `checkpoint_timeout`
- `game_over`
and informs the game master if all checkpoints were collected, the QuackMan achieved a new score, or the QuackMan was detected by a GhostBot and the game is over (when running on the appropriate bot type).

## docker run command
Replace the `ROBOT_HOSTNAME` with your actual robot name.
``` bash
docker run -it --rm --name=lane-following --net host --privileged  --memory "800m" --memory-swap="2800m" -v /data:/data -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket duckietown/dt-core:daffy-arm64v8 roslaunch duckietown_demos lane_following.launch veh:=ROBOT_HOSTNAME
```
### Adjusting the lane following robot speed
1. Create a directory and file for the parameters you want to replace
Run the following # (IN SSH Terminal on your robot, not main-workspace)

``` bash
mkdir -p ~/lane_control
touch ~/lane_control/default.yaml
```
2. copy the default parameters to the file, and edit the values you want to change. https://github.com/duckietown/dt-core/blob/daffy/packages/lane_control/config/lane_controller_node/default.yaml

3. Start the lane-following container again, with the command (the only extra section mounts the newly edited parameters file inside the container)
``` bash
 docker run \
 -it --rm --name=lane-following --net host --privileged --memory "800m" --memory-swap="2800m" \
 -v /data:/data -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
 -v /home/duckie/lane_control/default.yaml:/code/catkin_ws/src/dt-core/packages/lane_control/config/lane_controller_node/default.yaml \
 duckietown/dt-core:daffy-arm64v8 \
 roslaunch duckietown_demos lane_following.launch veh:=ROBOT_HOSTNAME
```
