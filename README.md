# HSR_RL-environment
This project contains three environments of Toyota HSR for RL research. Three environments are corresponding to food-searching, maze and grasping tasks. 



### Training example
* **Training foodsearhing model**
```
python examples/rl.py --env_name="FoodHuntingHSR-v0" --total_timesteps=206083 --filename="saved_model_F"
```
* **Training mazenavigating model**
```
python examples/rl.py --env_name="MazeNavigatingHSR-v0" --total_timesteps=206083 --filename="saved_model_M"
```
* **Training Learn grasping model**
```
python examples/rl.py --env_name="GraspingHSR-v0" --total_timesteps=206083 --filename="saved_model_G"
```
### Testing the trained model
After taining the models, you can demonstrate them due to following codes:
```
python examples/rl.py --env_name="FoodHuntingHSRGUI-v0" --total_timesteps=10000 --filename="saved_model_F" --play
python examples/rl.py --env_name="MazeNavigatingHSRGUI-v0" --total_timesteps=10000 --filename="saved_model_M" --play
python examples/rl.py --env_name="GraspingHSRGUI-v0" --total_timesteps=1000 --filename="saved_model_G" --play
```
