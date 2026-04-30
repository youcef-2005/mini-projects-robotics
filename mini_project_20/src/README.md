# Mini-Project 20: Reinforcement Learning (Q-Learning Agent)

## Overview
Moving beyond deterministic and predictive control algorithms, this final project explores the domain of **Artificial Intelligence**. It implements a model-free Reinforcement Learning (RL) algorithm known as **Q-Learning**, allowing an autonomous agent to independently discover the optimal obstacle-avoidance policy inside a simulated GridWorld environment.

## Mathematical Model: Markov Decision Process (MDP)
The environment is modeled as a discrete MDP consisting of:
* A set of states $S$ representing spatial grid coordinates $(x, y)$.
* A set of actions $A$ containing $\{Up, Down, Left, Right\}$.
* A reward function $R(s, a)$ that penalizes collisions ($-100$), penalizes time spent ($-1$), and heavily rewards reaching the target state ($+100$).

## The Bellman Equation
The agent's "brain" is a 3D matrix (the Q-Table) that stores the expected utility of taking a given action in a given state. Over $600$ simulated training episodes, the agent updates this matrix using the temporal-difference formulation of the Bellman Equation:

$$Q^{new}(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha \cdot \left[ r_t + \gamma \cdot \max_a Q(s_{t+1}, a) - Q(s_t, a_t) \right]$$

Where:
* $\alpha$: Learning Rate (determines to what extent newly acquired information overrides old information).
* $\gamma$: Discount Factor (determines the importance of future rewards).
* $\max_a Q(s_{t+1}, a)$: The maximum predicted reward for the next state.

## Epsilon-Greedy Strategy
To ensure the agent doesn't get stuck in local minima during early training, an $\epsilon$-greedy exploration strategy is utilized. Initially, the agent acts completely randomly ($\epsilon = 1.0$) to explore the physical constraints of the map. As training progresses, $\epsilon$ decays, shifting the agent from **exploration** (learning the map) to **exploitation** (utilizing the optimal path).

## Execution
This ROS 2 node is fully self-contained. Upon initialization, it processes the $600$ training lifecycles computationally. Once convergence is achieved, it enters Execution Mode, leveraging the optimized Q-Table to navigate the maze flawlessly. The optimal learned trajectory is continuously published to the `/ai_robot_pose` topic.
