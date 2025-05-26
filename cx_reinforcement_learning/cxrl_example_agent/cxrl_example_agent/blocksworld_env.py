from cxrl_gym.cxrl_gym import CXRLGym
from rclpy.node import Node
import rclpy


class BlocksworldEnv(CXRLGym):
    def __init__(self, node: Node, mode: str, number_robots: int):
        self.reward_in_episode = 0
        super().__init__(node, mode, number_robots)

    def step(self, action):
        with open("cxrl-bw-log-episode-reward.txt", 'a+') as f:
            f.write(f"{self.action_dict[action]} \n")
        state, reward, done, truncated, info = super().step(action)
        self.reward_in_episode += reward
        return state, reward, done, truncated, info
    
    def reset(self, seed: int = None, options: dict[str, any] = None):
        with open("cxrl-bw-log-episode-reward.txt", 'a+') as f:
            f.write(f"{self.reward_in_episode} \n")
        self.reward_in_episode = 0
        return super().reset(seed=seed)
    
    def generate_action_space(self):
        self.node.get_logger().info("Generating action space...")
        action_space =  ["STACK#upper#block1#lower#block2",
                         "STACK#upper#block1#lower#block3",
                         "STACK#upper#block2#lower#block1",
                         "STACK#upper#block2#lower#block3",
                         "STACK#upper#block3#lower#block1",
                         "STACK#upper#block3#lower#block2"
                        ]       
        return action_space

    def render(self):
        pass
