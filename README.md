# hello_tsid

Reproducing tsid/exercizes/ex_1_ur5.py in C++ using a panda robot urdf:
reach a fixed target with the end effector with:  
- $\mathbb{R}^3$ end-effector tracking task
- $\mathbb{R}^7$ posture task
- torque limits
- joint velocity limits

TODO:
- cleaner plot script
- read tsid problem config from yaml (for C++)


## dependencies
`conda install pinocchio tsid`  

For the simulation:  
`pip install git+https://github.com/MedericFourmy/unified_simulators`