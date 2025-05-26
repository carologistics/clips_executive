#!/usr/bin/env python3

from cxrl_mrmppo.MultiRobotMaskablePPO import MultiRobotMaskablePPO
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from stable_baselines3.common.callbacks import StopTrainingOnMaxEpisodes, CheckpointCallback
from stable_baselines3.common.logger import configure
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
import importlib

import rclpy
from rclpy.node import Node
import numpy as np

import os
import signal
import time


class CXRLNode(Node):
    def __init__(self):
        super().__init__("cxrl_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ("package_dir", rclpy.Parameter.Type.STRING),
                ("agent_name", rclpy.Parameter.Type.STRING),
                ("rl_mode", rclpy.Parameter.Type.STRING),
                ("number_of_robots", rclpy.Parameter.Type.INTEGER),
                ("training.retraining", rclpy.Parameter.Type.BOOL),
                ("training.max_episodes", rclpy.Parameter.Type.INTEGER),
                ("env.entrypoint", rclpy.Parameter.Type.STRING),
                ("model.learning_rate", rclpy.Parameter.Type.DOUBLE),
                ("model.gamma", rclpy.Parameter.Type.DOUBLE),
                ("model.gae_lambda", rclpy.Parameter.Type.DOUBLE),
                ("model.ent_coef", rclpy.Parameter.Type.DOUBLE),
                ("model.vf_coef", rclpy.Parameter.Type.DOUBLE),
                ("model.max_grad_norm", rclpy.Parameter.Type.DOUBLE),
                ("model.batch_size", rclpy.Parameter.Type.INTEGER),
                ("model.n_steps", rclpy.Parameter.Type.INTEGER),
                ("model.seed", rclpy.Parameter.Type.INTEGER),
                ("model.verbose", rclpy.Parameter.Type.INTEGER),
                ("model.n_robots", rclpy.Parameter.Type.INTEGER),
                ("model.wait_for_all_robots", rclpy.Parameter.Type.BOOL),
                ("training.timesteps", rclpy.Parameter.Type.INTEGER)
            ])
        self.set_dirs()
        self.shutdown = False
        self.get_logger().info("CXRLNode initialised")
        self.env = None
        self.model = None

    def set_dirs(self):
        home_dir = os.path.expanduser("~")
        package_dir = self.get_parameter("package_dir").value
        self.save_dir = os.path.join(home_dir, package_dir, "trained_agents")
        self.log_dir = os.path.join(home_dir, package_dir, "logs")
        self.checkpoint_dir = os.path.join(
            home_dir, package_dir, "checkpoint_agents")

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        if not os.path.exists(self.checkpoint_dir):
            os.makedirs(self.checkpoint_dir)

    def create_env(self):

        mod_name, attr_name = self.get_parameter(
            "env.entrypoint").value.split(":")
        mod = importlib.import_module(mod_name)
        env_class = getattr(mod, attr_name)
        self.env = env_class(self, self.get_parameter("rl_mode").value.upper(), 
                             self.get_parameter("number_of_robots").value)
        return self.env

    def create_new_model(self) -> MultiRobotMaskablePPO:
        self.model = MultiRobotMaskablePPO(
            policy=MaskableActorCriticPolicy,
            env=self.env,
            learning_rate=self.get_parameter("model.learning_rate").value,
            gamma=self.get_parameter("model.gamma").value,
            gae_lambda=self.get_parameter("model.gae_lambda").value,
            ent_coef=self.get_parameter("model.ent_coef").value,
            vf_coef=self.get_parameter("model.vf_coef").value,
            max_grad_norm=self.get_parameter("model.max_grad_norm").value,
            batch_size=self.get_parameter("model.batch_size").value,
            n_steps=self.get_parameter("model.n_steps").value,
            seed=self.get_parameter("model.seed").value,
            verbose=self.get_parameter("model.verbose").value,
            n_robots=self.get_parameter("number_of_robots").value,
            time_based=False,
            n_time=300,
            deadzone=5,
            wait_for_all_robots=self.get_parameter(
                "model.wait_for_all_robots").value
        )
        sb3_logger = configure(self.log_dir, ["stdout", "csv", "log"])
        self.model.set_logger(sb3_logger)
        self.env.set_rl_model(self.model)
        return self.model

    def load_model(self) -> MultiRobotMaskablePPO:
        agent_path = os.path.join(self.save_dir, str(
            self.get_parameter("agent_name").value)+".zip")
        self.model = MultiRobotMaskablePPO.load(agent_path, env=self.env)
        self.env.set_rl_model(self.model)
        self.get_logger().info("Agent loaded")
        return self.model

    def shutdown(self, sig, frame):
        self.shutdown = True
        if self.env != None:
            self.env.shutdown = True
        if self.model != None:
            self.model.shutdown = True


def main(args=None):
    rclpy.init()
    node = CXRLNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    Thread(target=executor.spin).start()

    signal.signal(signal.SIGINT, node.shutdown)

    node.get_logger().info("Creating environment")
    env = node.create_env()

    if node.get_parameter("rl_mode").value.upper() == "TRAINING":

        if node.get_parameter("training.retraining").value:
            node.get_logger().info("Retraining existing agent...")
            model = node.load_model()
        else:
            node.get_logger().info("Creating new agent...")
            model = node.create_new_model()

        callback_max_episodes = StopTrainingOnMaxEpisodes(
            max_episodes=node.get_parameter("training.max_episodes").value, verbose=1)
        checkpoint_callback = CheckpointCallback(
            save_freq=200, save_path=node.checkpoint_dir)

        model.learn(total_timesteps=node.get_parameter("training.timesteps").value, callback=[
                    callback_max_episodes, checkpoint_callback], log_interval=1)
        model.save(os.path.join(node.save_dir, str(
            node.get_parameter("agent_name").value)))

        node.get_logger().info("Finished training, closing node")

    elif node.get_parameter("rl_mode").value == "EXECUTION":
        node.get_logger().info("Executing existing agent...")
        model = node.load_model()
        while (not node.shutdown):
            time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()
