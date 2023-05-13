# Contact Trajectory Optimization via MCTS
This repo hosts the source code for the paper [Efficient Object Manipulation Planning with Monte Carlo Tree Search](https://arxiv.org/abs/2206.09023)

## Dependencies
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) for rigid body dynamics
- [Bullet Utils](https://github.com/machines-in-motion/bullet_utils) for interfacing PyBullet with Pinocchio
- [Robot Properties NYU Finger](https://github.com/open-dynamic-robot-initiative/robot_properties_nyu_finger) for NYU finger URDFs and configuration files
- Other standard dependencies can be found in ``requirements.txt``.

## Notebooks
We prepared a few notebooks
- ``/demos``
``mcts[miqp]_playground.ipynb`` shows step-by-step how to generate a contact plan for a given motion with force plots and plan visualization.
``mcts[miqp]_bullet_simulation.ipynb`` simulates the contact plan with an impedance controller in PyBullet.
- ``/experiments`` contains notebooks to reproduce the respective experiments in the paper.
- ``/train`` contains notebooks to generate the training data and to train the neural networks.

## Citing

```
@article{zhu2022efficient,
  title={Efficient Object Manipulation Planning with Monte Carlo Tree Search},
  author={Zhu, Huaijiang and Meduri, Avadesh and Righetti, Ludovic},
  journal={arXiv preprint arXiv:2206.09023},
  year={2022}
}
```

## Maintainer
- Huaijiang Zhu

## Copyrights

Copyright(c) 2023 New York University

## License

BSD 3-Clause License



