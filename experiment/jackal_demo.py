import sys
import os
# Add the project root directory to the Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from environment.rigid_terrain import RigidTerrainEnvironment
from model_demo.jackal import Jackal

# Create the environment
env = RigidTerrainEnvironment()
# Create the Jackal robot
jackal = Jackal(env.system)

# initialize the environment
env.initialize()

# Run the simulation
while env.vis.Run():
    jackal.control(speed=1, steering=0)
    env.simulate()
    

