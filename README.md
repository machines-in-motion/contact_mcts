# Contact MCTS
This repo hosts the source code for the paper [Efficient Object Manipulation Planning with Monte Carlo Tree Search](https://arxiv.org/abs/2206.09023)

## Dependencies
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [Bullet Utils](https://github.com/machines-in-motion/bullet_utils)
- [Robot Properties NYU Finger](https://github.com/open-dynamic-robot-initiative/robot_properties_nyu_finger)
- Other standard dependencies can be found in ``requirements.txt``.

## Demos
Play with the Jupyter notebooks in ``/notebooks``.
- [mcts_playground.ipynb](https://github.com/huaijiangzhu/contact_mcts/blob/46afe116c3f19d3b338428792d4ba1d511e1a4de/notebooks/mcts_playground.ipynb) shows step-by-step how to generate a contact plan for a given motion with force plots and plan visualization.
- [simulation.ipynb](https://github.com/huaijiangzhu/contact_mcts/blob/46afe116c3f19d3b338428792d4ba1d511e1a4de/notebooks/simulation.ipynb) generates end-effector motion to realize the contact plan and simulate it with an impedance controller in PyBullet.
- [data_generation.ipynb](https://github.com/huaijiangzhu/contact_mcts/blob/46afe116c3f19d3b338428792d4ba1d511e1a4de/notebooks/data_generation.ipynb) generates training data with random planar manipulation tasks.
- [train.ipynb](https://github.com/huaijiangzhu/contact_mcts/blob/46afe116c3f19d3b338428792d4ba1d511e1a4de/notebooks/train.ipynb) trains and saves the neural networks.
- [planar_tasks_evaluation.ipynb](https://github.com/huaijiangzhu/contact_mcts/blob/46afe116c3f19d3b338428792d4ba1d511e1a4de/notebooks/planar_tasks_evaluation.ipynb) reproduces the experiment results for a comparison between the trained and untrained model.