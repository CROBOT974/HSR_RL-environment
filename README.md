# HSR_RL-environment
This project contains three environments of Toyota HSR for RL research. Three environments are corresponding to food-searching, maze and grasping tasks. 

## Install
Here is the code installing testing in Windows 10.

* **Downloading the codes moving to its content**
```
git clone https://github.com/CROBOT975/HSR_RL-environment.git
cd HSR_RL-enviroment/PyHSR
```
* **Activating the venv**
```
cd venv/Scripts
activate
cd ../..
```
* **Installing the required packages**
```
pip install numpy
pip install gym
pip install pybullet
pip install stable-baselines3
```
You can download the Human Support Robot model, and yet the grippers of HSR model is not appropriate for grasping. Therefore, you'd better make some change in the original code of HSR urdf model,like following:

Download the model:
```
git clone https://github.com/ToyotaResearchInstitute/hsr_meshes.git
cp -rp hsr_meshes venv/lib/python3.9/site-packages/pybullet_data
cp -p HSR_RL_env/model/hsrb4s.urdf venv/lib/python3.9/site-packages/pybullet_data
```

Copy the object files to pybullet_data,and change the original code of hsrb4s.urdf like follwing image(from '.stl' to '.obj' for two fingers and two finger tips):
```
cp -p HSR_RL_env/model/l_finger.obj venv/lib/python3.9/site-packages/pybullet_data
cp -p HSR_RL_env/model/l_finger_tip.obj venv/lib/python3.9/site-packages/pybullet_data
```
![image](https://user-images.githubusercontent.com/74949016/158175114-b5d332d0-d1c8-454c-8748-2076c1e45474.png)

Downloading some environment models
```
cp -p HSR_RL_env/model/food_cube.urdf venv/lib/python3.9/site-packages/pybullet_data
cp -p HSR_RL_env/model/food_cube_for_grasping.urdf venv/lib/python3.9/site-packages/pybullet_data
cp -p HSR_RL_env/model/food_sphere.urdf venv/lib/python3.9/site-packages/pybullet_data
cp -p HSR_RL_env/model/maze.urdf venv/lib/python3.9/site-packages/pybullet_data
cp -p HSR_RL_env/model/wall.urdf venv/lib/python3.9/site-packages/pybullet_data

cd HSR_RL_env
pip install -e .
cd ..
```

## Training example
### Run training
* **Foodsearhing model**
```
python examples/rl.py --env_name="FoodHuntingHSR-v0" --total_timesteps=206083 --filename="saved_model_F"
```
* **Mazenavigating model**
```
python examples/rl.py --env_name="MazeNavigatingHSR-v0" --total_timesteps=206083 --filename="saved_model_M"
```
* **Grasping model**
```
python examples/rl.py --env_name="GraspingHSR-v0" --total_timesteps=206083 --filename="saved_model_G"
```
### The result of training three models by PPO(Proximal Policy Optimization) and SAC(Soft Actor-Crtic)
![image](https://user-images.githubusercontent.com/74949016/158178385-d3486bed-b6c0-4861-9b4c-3135e591ec91.png)
![image](https://user-images.githubusercontent.com/74949016/158179067-ff4fc10b-38b0-4eee-b46e-d25792936fcd.png)
![image](https://user-images.githubusercontent.com/74949016/158179211-55f1e75e-48e2-466d-8f6a-8fed1f599302.png)


## Testing the trained model
### Run testing
After taining the models, you can demonstrate them due to following codes:
```
python examples/rl.py --env_name="FoodHuntingHSRGUI-v0" --total_timesteps=10000 --filename="saved_model_F" --play
```
```
python examples/rl.py --env_name="MazeNavigatingHSRGUI-v0" --total_timesteps=10000 --filename="saved_model_M" --play
```
```
python examples/rl.py --env_name="GraspingHSRGUI-v0" --total_timesteps=1000 --filename="saved_model_G" --play
```
### Testing Result
You can see the demonstration which denotes the dramatic performance of RL algorithm due to the difference between 2e3 timesteps traning and  2e5 timesteps training as following:
[food-searching]
[maze]
[grasping]
