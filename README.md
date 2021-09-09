# GeneralizedTAMP

Generalized Task and Motion Planner is an algorithm for few-shot learning abstract policies from examples which generalize to new problem instances and accelerate discovery of continuous task and motion plans.

## Installation
```
git clone https://github.com/generalizedtamp/GeneralizedTAMP.git
cd GeneralizedTAMP
git submodule update --init --recursive
python -m pip install -r requirements.txt
cd ..

```

## Dependencies
This project has the following additional dependencies: 

### 1. IKFast: A library for computing inverse kinematics for the PR2
```
cd pddlstream/examples/pybullet/utils/pybullet_tools/ikfast/pr2
python setup.py
```

* The following dependencies require [docker](https://www.docker.com/)

### 2. QNP2FOND: A C++ library for converting qualitative numeric planning specifications to FOND planning problems
```
cd QNP2FOND
docker build -t <docker-username>/qnp2fond .
```
Then add the following to your .bashrc/.zshrc
```
qnp2fond() {
   docker run --rm -v $(pwd):/GeneralizedTAMP -w /QNP2FOND/build <docker-username>/qnp2fond qnp2fond $*
}
```

### 3. FOND_PRP: A C++ library for doing fully observable nondeterministic planning
```
cd FOND_PRP
docker build -t <docker-username>/fond_prp .
```
Then add the following to your .bashrc/.zshrc
```
prp() {
   docker run --rm -v $(pwd):/GeneralizedTAMP -w /FOND_PRP/build <docker-username>/fond_prp prp $*
}
```

### 4. open-wbo: A C++ MaxSAT solver
```
cd open-wbo
docker build -t <docker-username>/open-wbo .
```
Then add the following to your .bashrc/.zshrc
```
open-wbo() {
   docker run --rm -v $(pwd):/GeneralizedTAMP -w /open-wbo/build <docker-username>/open-wbo open-wbo $*
}
```

### 5. Fast Downward: A STRIPS planning library
```
CXX=/usr/local/Cellar/gcc@8/<gcc@8-version>/bin/g++-8 CC=/usr/local/Cellar/gcc@8/<gcc@8-version>/bin/gcc-8 ./pddlstream/FastDownward/build.py release64
CXX=/usr/local/Cellar/gcc@8/<gcc@8-version>/bin/g++-8 CC=/usr/local/Cellar/gcc@8/<gcc@8-version>/bin/gcc-8 ./downward/build.py release64
```
TODO: Merge the two downward builds into one

Then add the following to .bashrc/.zshrc
```
export PYTHONPATH=$PYTHONPATH:$HOME/GeneralizedTAMP/pddlstream/FastDownward/src/translate
export PYTHONPATH=$PYTHONPATH:$HOME/GeneralizedTAMP/pddlstream
export PYTHONPATH=$PYTHONPATH:$HOME/GeneralizedTAMP/pddlstream/pybullet_planning
export PYTHONPATH=$PYTHONPATH:$HOME/GeneralizedTAMP
```

## Codebase Layout
What follows is a description of the layout of the codebase.

### ``genqnp/experiments.yaml``
This file details all of the possible experiments you can run in this codebase.
Each experiment consists of 
1. A set of small task and motion planning problems which can be solved in a reasonable amount of time by PDDLStream on the input domain.
2. A set of larger task and motion planning problems for testing and evaluation 

The experiment name (ex. stack_0) is used as an argument to every subsequent file in the pipeline

### ``genqnp/generate_dataset.py --exp-name=stack_0``
Running this file will call PDDLStream for each of the training problems in the specified environment and store
the results in that tasks results folder

### ``genqnp/tamp_feature_pool.py --exp-name=stack_0``
Running this file will generate a feature pool from the task's domain, and create an abstraction of the domain with respect to the training examples
The abstraction is stored in the task results folder under the files 
- `abstraction.qnp`: The qualitative numeric planning specification
- `features.txt`/`features.pkl`: The human readable and object versions of the features associated with the abstraction

### ``genqnp/make_policy.py --exp-name=stack_0``
This file converts the qnp specification into a policy in three steps
1. Converts the QNP specification to a fond planning specification
2. Solves the FOND planning problem using PRP
3. Visualizes the policy in a graphviz graph

The resulting policy visualization can by viewed by running `xdot genqnp/tasks/<exp-name>/graph.dot`

### ``genqnp/embed_policy.py --exp-name=stack_0``
This file takes in the original domain file and the policy and outputs a new constrained task-specific domain file 
with the contraints of the policy embedded within it.

### ``genqnp/profile_policy --exp-name=stack_0``
This file plans for each of the testing problem instances using the constrained domain and original domain
The results are stored in the task results folder.


### ``genqnp/visualize_profiling --exp-name=stack_0``
This file plots the most recent profiling results of a particular experiment

