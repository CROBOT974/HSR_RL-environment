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
```
git clone https://github.com/ToyotaResearchInstitute/hsr_meshes.git
cp -rp hsr_meshes venv/lib/python3.9/site-packages/pybullet_data
cp -p HSR_RL_env/urdf/hsrb4s.urdf venv/lib/python3.9/site-packages/pybullet_data
```

### Training example
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
### Testing the trained model
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
