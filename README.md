# Contact state and SLAM

## Get Started

### System Requirement

- This simulation framework has been tested in the following operating systems:

	- Ubuntu 18.04, 20.04
	- Mac OS

- Other systems

	- Windows

		- Mujoco 2.0+ has deprecated its support in Windows and therefore cannot be installed in Windows.
		- Old version Mujoco 1.5 can run in Windows, but using old version may cause unpredictable problems and is thus STRONGLY not recommended.
		- As a geek in Computer Science, it is time to start your Linux career :D!

### Software Requirement

- Before using this repository, you need to install these software:

	- [Git](https://git-scm.com/), as version control
	- [Anaconda](https://www.anaconda.com/), to handle software dependencies, install and run simulation in a virtual environment. Other Conda apps are also fine.
	- [Mujoco](http://www.mujoco.org/), a state of the art physical simulation engine

		- Mujoco is a physics engine written in C++, which is originally developed at the Movement Control Laboratory, University of Washington. OpenAI implemented a Python interface (mujoco-py) for this engine, which is utilized by our simulation framework. Since it is tricky to install Mujoco, please follow our install instruction below. 

### Installation

- Clone repository from Github

	- ```git clone https://github.com/ALRhub/SimulationFrameworkPublic```

- Get Mujoco

	- Before start, you should know

		- Note Mujoco is a C++ library, while mujoco-py is a Python package.
		- You need to firstly download and unzip Mujoco and place it in a desired folder. Later When you install mujoco-py, it will find and install Mujoco together.

	- Download Mujoco

		- Choose [Linux](https://www.roboti.us/download/mujoco200_linux.zip), or for [MacOS](https://www.roboti.us/download/mujoco200_macos.zip)
		- Unzip your package and put everything (bin, doc...) under "$HOME/.mujoco/mujoco200/"

	- Get a Mujoco license

		- You can apply for a free license, if your are a university student. Or you can use the license of our lab, in case of any publication purpose. For more details of mujoco license, click [here](https://www.roboti.us/license.html).
		- Put your downloaded Mujoco license (mjkey.txt) under "$HOME/.mujoco/" AND under "$HOME/.mujoco/mujoco200/bin"
		- Desired folder structure looks like:  
```
.mujoco/  
├── mjkey.txt  
└── mujoco200  
    ├── bin  
    ├── doc  
    ├── include  
    ├── mjkey.txt  
    ├── model  
    └── sample  
```

- Install system dependency (because this cannot be installed through conda unfortunately)

	- ```sudo apt-get install libosmesa6-dev```

- Prepare virtual environment and install everything

	- Install the latest Anaconda3 or other Conda apps

		- For more support, click [Anaconda](https://www.anaconda.com/)

	- Create virtual environment and install dependencies (C++, python)

		- Open a terminal and export the environment variable LD_LIBRARY_PATH
		 with the absolute directory of your mujoco binary. For example
		  (replace DonaldTrump with your username and path!):  
```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/DonaldTrump/.mujoco/mujoco200/bin```

		- Set the current directory to the cloned repository. Then type 
```conda env create -f conda_env.yml```

	- Activate virtual environment

		- ```conda activate SimulationFramework```

			- You will find the env title (base) changes to (SimulationFramework)

	- Add environment variables to your VIRTUAL environment

		- Open a terminal and activate virtual environment (see above).
		- Set LD_LIBRARY_PATH the absolute directory of your mujoco binary.    
For example(replace DonaldTrump with your username!):  
```conda env config vars set LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/DonaldTrump/.mujoco/mujoco200/bin```
		- Set LD_PRELOAD the absolute directory of your GLEW library (should be in your virtural environment).   
For example(replace DonaldTrump with your username and path to Anaconda!):  
```conda env config vars set LD_PRELOAD=$LD_PRELOAD:/home/DonaldTrump/anaconda3/envs/SimulationFramework/lib/libGLEW.so```
		- Reactivate your virtual environment by 
```conda activate SimulationFramework```
		- (Troubleshooting) If you make any mistakes when setting up variables
		, you can delete them, e.g.   
```conda env config vars unset LD_LIBRARY_PATH``` 

	- Install SimultionFramework into Python path

		- Open a terminal, go to your repository, activate your virtual environment
		- Then  
```pip install -e . ```

	- Ready to go

		- Now your SimulationFramework is ready, you can run a demo under [Demo folder](./demos) to see if everything works. For example:  
```python Demo_Pick_and_Place_IK_Control_Mujoco.py```
		- If you want to use IDEs, like Pycharm and VSCode, we recommend you always:

			- Open a terminal and activate the virtual environment
			- Start your IDE from this terminal, otherwise your IDE may lose some environment variables and cannot run the demo.
			- Set the Python interpreter to the one in the Conda virtual environment

	- Uninstall

		- Since everything is installed in your virtual environment, SimulationFramework will not mess up your global system's dependencies. When you decide to delete Simulation framework, just go with 
```conda env remove --name SimulationFramework```  
then everything is gone.

### File Structure

- The repository mainly contains four parts: Classic Framework, Demonstrations, Gym Framework, and Environments.
- Classic Framework

	- In [classic_framework](./classic_framework) you can find basic level programming modules as well as robot control modules.
	- Better to be used if you want to define your task from scratch.

- Demonstrations

	- In [demos](./demos) you can find some examples of how to define a basic robot learning tasks. Understand all the procedures to define a robot, a scene, a camera system as well as different ways to control a robot.

- Gym Framework

	- In [gym_framework](./gym_framework) you can find high level robot Reinforcement Learning modules where you can quickly define your Reinforcement learning tasks and deploy some benchmark RL algorithms.
	- Inspired by OpenAI Gym.

- Environments

	- In [envs](./envs) you can find the geometry, CAD meshes, textures and configuration files for the robot, camera and other objects. You can use them to define your own task.
	- If you want to create and define your own simulation objects, such as obstacles, bananas, please follow the examples there.

## SLAM demo
-	There are 2 maze model provided in this repository, chose one you like for demonstration.
-	Change the `URDF_NAME` in `main.py` to the model you want to load, and run the script to see the demonstration.

## Global Variable Explain
- `MAZE_GRID` is the virtual map of current maze. As we rasterized the maze, the location in the map can be described by a 2d array, in which 1 means unreachable and 0 means reachable.
- `robot_grid_pos` Current end effector position in the grid.
- `MAZE_POS` The cartesian position of the maze.
- `CS1_OFFSET` The cartesian offset of cs1 regarding the maze position
- `*_CART_STEP_SIZE` Step size to define how far is between 2 grid.
- `CONSTRAINT` Constraint following parameter, 0 for clockwise, 1 for counter-clockwise
- `URDF_NAME` The urdf filename to define the maze.