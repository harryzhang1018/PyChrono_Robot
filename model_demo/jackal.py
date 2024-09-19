#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Harry Zhang
#
# Created:     03/09/2024
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------
 
 
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import math
import os

class Jackal:
    def __init__(self, system):
        self.system = system
        self.chassis = None
        self.wheels = {}
        self.motors = {}
        self.steering_motors = {}

        # Get the directory where the script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the path to the data folder
        self.data_dir = os.path.join(script_dir, '..', 'data')
        # Normalize the path
        self.data_dir = os.path.normpath(self.data_dir)

        self._load_model()
        self._setup_wheels()
        self._setup_motors()

    def _load_model(self):
        print("Loading Jackal model...")
        exported_items = chrono.ImportSolidWorksSystem(os.path.join(self.data_dir, 'Jackal_1.py'))
        for item in exported_items:
            self.system.Add(item)
        print("...done!")

    def _setup_wheels(self):
        material = chrono.ChContactMaterialNSC()
        
        self.chassis = self.system.SearchBody('Part15-1')
        wheel_names = ['fr', 'fl', 'br', 'bl']
        part_numbers = ['16-1', '16-5', '16-2', '16-6']

        for name, part in zip(wheel_names, part_numbers):
            wheel = self.system.SearchBody(f'Part{part}')
            
            # Create cylinder shape for collision
            radius = 0.1  # Adjust this value to match your wheel's radius
            width = 0.06  # Adjust this value to match your wheel's width
            
            cylinder = chrono.ChCollisionShapeCylinder(material, radius, width)
            
            # The cylinder's axis is along Y by default, so we need to rotate it
            rotation = chrono.Q_ROTATE_X_TO_Y
            cylinder_pos = chrono.ChVector3d(0, 0, 0)  # Adjust if the cylinder needs to be offset
            
            wheel.AddCollisionShape(cylinder, chrono.ChFramed(cylinder_pos, rotation))
            wheel.EnableCollision(True)
            self.wheels[name] = wheel

    def _setup_motors(self):
        # Setup drive motors
        self.motors['br'] = chrono.ChLinkMotorRotationSpeed()
        self.motors['bl'] = chrono.ChLinkMotorRotationSpeed()
        self.motors['fr'] = chrono.ChLinkMotorRotationSpeed()
        self.motors['fl'] = chrono.ChLinkMotorRotationSpeed()

        # Initialize motors
        for wheel in ['fr', 'fl', 'br', 'bl']:
            joint = self.system.SearchLink(f'Concentric{1 if wheel == "fr" else 4 if wheel == "fl" else 2 if wheel == "br" else 3}')
            frame = joint.GetVisualModelFrame()

            self.motors[wheel].Initialize(self.chassis, self.wheels[wheel], frame)
            self.system.Add(self.motors[wheel])

    def control(self, speed=1, steering=0):
        
        
        linear_to_angular_velocity = speed / 0.1
        # set motor velocity function based on speed and skid steering
        wheel_motor_func1 = chrono.ChFunctionConst(-linear_to_angular_velocity * (1 + steering))
        wheel_motor_func2 = chrono.ChFunctionConst(linear_to_angular_velocity * (1 - steering))
        
        self.motors['fr'].SetMotorFunction(wheel_motor_func1)
        self.motors['fl'].SetMotorFunction(wheel_motor_func2)
        self.motors['br'].SetMotorFunction(wheel_motor_func1)
        self.motors['bl'].SetMotorFunction(wheel_motor_func2)

my_system = chrono.ChSystemNSC()
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

jackal = Jackal(my_system)

# Create rigid ground 
# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
# patch_mat.SetFriction(0.1)
# patch_mat.SetRollingFriction(0.001)
terrain = veh.RigidTerrain(my_system)
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.3, 0), chrono.Q_ROTATE_Z_TO_Y), 
    100, 100)

patch.SetTexture(chrono.GetChronoDataPath()+"vehicle/terrain/textures/tile4.jpg", 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

### Create visualization for the gripper fingers
vis = chronoirr.ChVisualSystemIrrlicht(my_system, chrono.ChVector3d(-2, 1, -1))
vis.EnableCollisionShapeDrawing(True)
timestep = 0.001
render_step_size = 1.0 / 25  # FPS = 50
render_steps = math.ceil(render_step_size / timestep)
step_number = 0
render_frame = 0

rt_timer = chrono.ChRealtimeStepTimer()

my_system.GetSolver().AsIterative().SetMaxIterations(500)


while vis.Run():
    sim_time = my_system.GetChTime()
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        # filename = './IMG_jackal/img_' + str(render_frame) +'.jpg' 
        # vis.WriteImageToFile(filename)
        # render_frame += 1
    terrain.Synchronize(sim_time)
    my_system.DoStepDynamics(timestep)
    terrain.Advance(timestep)
    if 0.5<sim_time < 5.0:
        jackal.control(speed=0.2,steering=0.0)
        print("forward")
    elif sim_time > 5.0 and sim_time < 10.5:
        jackal.control(speed=0.2,steering=0.5)
        print("left")
    elif sim_time > 10.5 and sim_time < 20.0:
        jackal.control(speed=0.2,steering=-0.5)
        print("right")
    elif sim_time > 20.0:
        jackal.control(speed=0.2,steering=0.0)
        print("forward")
    #jackal.control_steering(steering_angle=0.0)
    
    rt_timer.Spin(timestep)
    step_number += 1

 
