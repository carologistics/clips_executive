# Copyright (c) 2025 Carologistics
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""/***************************************************************************
 *  MultiRobotMaskablePPO.py -
 *
 *  Created:
 *  Copyright
 ****************************************************************************/"""


import threading
from threading import Thread
import time
from typing import Any, ClassVar, Dict, Optional, Type, Union

from cx_rl_multi_robot_mppo.MultiRobotMaskableRolloutBuffer import (
    MultiRobotMaskableDictRolloutBuffer,
    MultiRobotMaskableRolloutBuffer,
)
from gymnasium import spaces
import numpy as np
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from sb3_contrib.common.maskable.utils import get_action_masks, is_masking_supported
from sb3_contrib.ppo_mask import MaskablePPO
from sb3_contrib.ppo_mask.policies import CnnPolicy, MlpPolicy, MultiInputPolicy
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.policies import BasePolicy
from stable_baselines3.common.type_aliases import GymEnv, Schedule
from stable_baselines3.common.utils import get_schedule_fn, obs_as_tensor
from stable_baselines3.common.vec_env import VecEnv
import torch as th


class MultiRobotMaskablePPO(MaskablePPO):
    """
    Proximal Policy Optimization algorithm (PPO) (clip version) with Invalid Action Masking for Muli-Robot Scenarios in the CLIPS Executive.

    Based on the original Stable Baselines 3 implementation and the maskable PPO implementation in Stable Baselines3 contrib.

    Introduction to PPO: https://spinningup.openai.com/en/latest/algorithms/ppo.html
    Background on Invalid Action Masking: https://arxiv.org/abs/2006.14171

    :param policy: The policy model to use (MlpPolicy, CnnPolicy, ...)
    :param env: The environment to learn from (if registered in Gym, can be str)
    :param learning_rate: The learning rate, it can be a function
        of the current progress remaining (from 1 to 0)
    :param n_steps: The number of steps to run for each environment per update
        (i.e. batch size is n_steps * n_env where n_env is number of environment copies running in parallel)
    :param batch_size: Minibatch size
    :param n_epochs: Number of epoch when optimizing the surrogate loss
    :param gamma: Discount factor
    :param gae_lambda: Factor for trade-off of bias vs variance for Generalized Advantage Estimator
    :param clip_range: Clipping parameter, it can be a function of the current progress
        remaining (from 1 to 0).
    :param clip_range_vf: Clipping parameter for the value function,
        it can be a function of the current progress remaining (from 1 to 0).
        This is a parameter specific to the OpenAI implementation. If None is passed (default),
        no clipping will be done on the value function.
        IMPORTANT: this clipping depends on the reward scaling.
    :param normalize_advantage: Whether to normalize or not the advantage
    :param ent_coef: Entropy coefficient for the loss calculation
    :param vf_coef: Value function coefficient for the loss calculation
    :param max_grad_norm: The maximum value for the gradient clipping
    :param target_kl: Limit the KL divergence between updates,
        because the clipping is not enough to prevent large update
        see issue #213 (cf https://github.com/hill-a/stable-baselines/issues/213)
        By default, there is no limit on the kl div.
    :param stats_window_size: Window size for the rollout logging, specifying the number of episodes to average
        the reported success rate, mean episode length, and mean reward over
    :param tensorboard_log: the log location for tensorboard (if None, no logging)
    :param policy_kwargs: additional arguments to be passed to the policy on creation
    :param verbose: the verbosity level: 0 no output, 1 info, 2 debug
    :param seed: Seed for the pseudo random generators
    :param device: Device (cpu, cuda, ...) on which the code should be run.
        Setting it to auto, the code will be run on the GPU if possible.
    :param _init_setup_model: Whether or not to build the network at the creation of the instance
    """

    policy_aliases: ClassVar[Dict[str, Type[BasePolicy]]] = {
        'MlpPolicy': MlpPolicy,
        'CnnPolicy': CnnPolicy,
        'MultiInputPolicy': MultiInputPolicy,
    }

    def __init__(
        self,
        policy: Union[str, Type[MaskableActorCriticPolicy]],
        env: Union[GymEnv, str],
        learning_rate: Union[float, Schedule] = 3e-4,
        n_steps: int = 2048,
        batch_size: Optional[int] = 64,
        n_epochs: int = 10,
        gamma: float = 0.99,
        gae_lambda: float = 0.95,
        clip_range: Union[float, Schedule] = 0.2,
        clip_range_vf: Union[None, float, Schedule] = None,
        normalize_advantage: bool = True,
        ent_coef: float = 0.0,
        vf_coef: float = 0.5,
        max_grad_norm: float = 0.5,
        target_kl: Optional[float] = None,
        stats_window_size: int = 100,
        tensorboard_log: Optional[str] = None,
        policy_kwargs: Optional[Dict[str, Any]] = None,
        verbose: int = 0,
        seed: Optional[int] = None,
        device: Union[th.device, str] = 'auto',
        _init_setup_model: bool = True,
        n_robots: int = 3,
        time_based: bool = False,
        n_time: int = 450,
        deadzone: int = 10,
        wait_for_all_robots: bool = True,
    ):
        self.time_based = time_based
        super().__init__(
            policy,
            env,
            learning_rate=learning_rate,
            n_steps=n_steps,
            batch_size=batch_size,
            n_epochs=n_epochs,
            gamma=gamma,
            gae_lambda=gae_lambda,
            clip_range=clip_range,
            clip_range_vf=clip_range_vf,
            normalize_advantage=normalize_advantage,
            ent_coef=ent_coef,
            vf_coef=vf_coef,
            max_grad_norm=max_grad_norm,
            target_kl=target_kl,
            stats_window_size=stats_window_size,
            tensorboard_log=tensorboard_log,
            policy_kwargs=policy_kwargs,
            verbose=verbose,
            seed=seed,
            device=device,
            _init_setup_model=_init_setup_model,
        )
        self.n_robots = n_robots

        self.n_time = n_time
        self.deadzone = deadzone
        self.wait_for_all_robots = wait_for_all_robots
        self.n_current_steps = 0
        self.no_callback = False
        self.rollouts_gathered = False
        self.shutdown = False

    def _setup_model(self) -> None:
        self._setup_lr_schedule()
        self.set_random_seed(self.seed)

        buffer_cls = (
            MultiRobotMaskableDictRolloutBuffer
            if isinstance(self.observation_space, spaces.Dict)
            else MultiRobotMaskableRolloutBuffer
        )

        self.policy = self.policy_class(
            self.observation_space,
            self.action_space,
            self.lr_schedule,
            **self.policy_kwargs,  # pytype:disable=not-instantiable
        )
        self.policy = self.policy.to(self.device)

        if not isinstance(self.policy, MaskableActorCriticPolicy):
            raise ValueError('Policy must subclass MaskableActorCriticPolicy')

        if self.time_based:
            buffer_size = 1000
        else:
            buffer_size = self.n_steps

        self.rollout_buffer = buffer_cls(
            buffer_size,
            self.observation_space,
            self.action_space,
            self.device,
            gamma=self.gamma,
            gae_lambda=self.gae_lambda,
            n_envs=self.n_envs,
        )

        # Initialize schedules for policy/value clipping
        self.clip_range = get_schedule_fn(self.clip_range)
        if self.clip_range_vf is not None:
            if isinstance(self.clip_range_vf, (float, int)):
                assert self.clip_range_vf > 0, (
                    '`clip_range_vf` must be positive, ' 'pass `None` to deactivate vf clipping'
                )

            self.clip_range_vf = get_schedule_fn(self.clip_range_vf)

    def collect_rollouts(
        self,
        env: VecEnv,
        callback: BaseCallback,
        rollout_buffer: RolloutBuffer,
        n_rollout_steps: int,
        use_masking: bool = True,
    ) -> bool:
        """
        Collect experiences using the current policy and fill a ``RolloutBuffer``.
        The term rollout here refers to the model-free notion and should not
        be used with the concept of rollout used in model-based RL or planning.

        This method is largely identical to the implementation found in the parent class.

        :param env: The training environment
        :param callback: Callback that will be called at each step
            (and at the beginning and end of the rollout)
        :param rollout_buffer: Buffer to fill with rollouts
        :param n_steps: Number of experiences to collect per environment
        :param use_masking: Whether or not to use invalid action masks during training
        :return: True if function returned with at least `n_rollout_steps`
            collected, False if callback terminated rollout prematurely.
        """

        assert isinstance(
            rollout_buffer, (MultiRobotMaskableRolloutBuffer, MultiRobotMaskableDictRolloutBuffer)
        ), "RolloutBuffer doesn't support action masking"
        assert self._last_obs is not None, 'No previous observation was provided'

        # Switch to eval mode (this affects batch norm / dropout)
        self.policy.set_training_mode(False)
        self.n_current_steps = 0
        threads = []
        self.rollouts_gathered = False
        self.no_callback = False
        rollout_buffer.reset()

        if use_masking and not is_masking_supported(env):
            raise ValueError(
                'Environment does not support action masking. Consider using ActionMasker wrapper'
            )

        callback.on_rollout_start()

        if self.time_based:
            start_time = time.time()
            while time.time() - start_time < self.n_time and not self.shutdown:
                if (
                    len(threads) < self.n_robots
                    and time.time() - start_time < self.n_time - self.deadzone
                ):
                    t = Thread(
                        target=self._do_step, args=(env, callback, rollout_buffer, use_masking)
                    )
                    threads.append(t)
                    t.start()
                threads_alive = []
                for thread in threads:
                    if not thread.is_alive():
                        thread.join()
                        if self.no_callback:
                            return False
                    else:
                        threads_alive.append(thread)
                threads = threads_alive
        else:
            while self.n_current_steps < n_rollout_steps and not self.shutdown:
                if len(threads) < self.n_robots:
                    t = Thread(
                        target=self._do_step, args=(env, callback, rollout_buffer, use_masking)
                    )
                    threads.append(t)
                    t.start()
                threads_alive = []
                for thread in threads:
                    if not thread.is_alive():
                        thread.join()
                        if self.no_callback:
                            return False
                    else:
                        threads_alive.append(thread)
                threads = threads_alive

        if self.shutdown:
            for thread in threads:
                thread.join()
            return False

        self.rollouts_gathered = True
        if self.wait_for_all_robots:
            while len(threads) > 0:
                for thread in threads:
                    threads_alive = []
                    if not thread.is_alive():
                        thread.join()
                        if self.no_callback:
                            return False
                    else:
                        threads_alive.append(thread)
                    threads = threads_alive
        with th.no_grad():
            # Compute value for the last timestep
            # Masking is not needed here, the choice of action doesn't matter.
            # We only want the value of the current observation.
            values = self.policy.predict_values(obs_as_tensor(self._last_obs, self.device))

        rollout_buffer.compute_returns_and_advantage(
            last_values=values, dones=self._last_episode_starts
        )

        callback.on_rollout_end()

        return True

    def _do_step(
        self,
        env: VecEnv,
        callback: BaseCallback,
        rollout_buffer: RolloutBuffer,
        use_masking: bool = True,
    ):
        current_thread = threading.get_ident()
        with th.no_grad():
            # Convert to pytorch tensor or to TensorDict
            obs_tensor = obs_as_tensor(self._last_obs, self.device)
            # This is the only change related to invalid action masking

            if use_masking:
                action_masks = get_action_masks(env)
                if np.sum(action_masks) == 0:
                    return
            actions, values, log_probs = self.policy(obs_tensor, action_masks=action_masks)
            actions = actions.cpu().numpy()
        new_obs, rewards, dones, infos = env.step(actions)
        if infos[0].get('outcome') == 'NO-GOAL-ID':
            return
        self.num_timesteps += env.num_envs
        if self.no_callback:
            return
        # Give access to local variables
        callback.update_locals(locals())

        if not callback.on_step():
            self.no_callback = True
            return
        if infos[0].get('outcome') == 'RESET':
            return

        self._update_info_buffer(infos)
        self.n_current_steps += 1

        if isinstance(self.action_space, spaces.Discrete):
            # Reshape in case of discrete action
            actions = actions.reshape(-1, 1)

        for idx, done in enumerate(dones):
            if (
                done
                and infos[idx].get('terminal_observation') is not None
                and infos[idx].get('TimeLimit.truncated', False)
            ):
                terminal_obs = self.policy.obs_to_tensor(infos[idx]['terminal_observation'])[0]
                with th.no_grad():
                    terminal_value = self.policy.predict_values(terminal_obs)[0]
                rewards[idx] += self.gamma * terminal_value

        if not self.rollouts_gathered:
            rollout_buffer.add(
                self._last_obs,
                actions,
                rewards,
                self._last_episode_starts,
                values,
                log_probs,
                action_masks=action_masks,
            )
        self._last_obs = new_obs
        self._last_episode_starts = dones
