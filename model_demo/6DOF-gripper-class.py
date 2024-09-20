import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math
import os

class RobotArm:
    def __init__(self, system):

        self.system = system
        # self.system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        # self.system = system
        self.material = chrono.ChContactMaterialNSC()
        self.material.SetFriction(0.5)

        # Get the directory where the script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the path to the data folder
        self.data_dir = os.path.join(script_dir, '..', 'data')
        # Normalize the path
        self.data_dir = os.path.normpath(self.data_dir)

        self.bodies = {}
        self.motors = {}
        self.locks = {}
        self.motor_funcs = {}

        self._load_model()
        self._setup_gripper_collision()
        self._setup_motors()
        self._setup_locks()
        self._setup_ball()

        # Initialize the flag for ball lock
        self.ball_gripper_lock_flag = False

    def _load_model(self):
        print("Loading Robot Arm model...")
        exported_items = chrono.ImportSolidWorksSystem(os.path.join(self.data_dir, 'robot_arm.py'))
        for item in exported_items:
            self.system.Add(item)
        print("...done!")

        # Extract bodies
        self.bodies['base'] = self.system.SearchBody('Block1-1-1^industrial robotic arm-1')
        self.bodies['base_motor_block'] = self.system.SearchBody('Assem7^industrial robotic arm_2-1')
        self.bodies['base_arm'] = self.system.SearchBody('Assem6^industrial robotic arm_2-1')
        self.bodies['second_arm'] = self.system.SearchBody('Assem5^industrial robotic arm_2-1')
        self.bodies['arm_gripper_connector'] = self.system.SearchBody('Assem4^industrial robotic arm_2-1')
        self.bodies['gripper'] = self.system.SearchBody('Assem3^industrial robotic arm_2-1')
        self.bodies['gripper_block_1'] = self.system.SearchBody('mob clap-1')
        self.bodies['gripper_block_2'] = self.system.SearchBody('mob clap-2')
        self.bodies['gripper_finger_1'] = self.system.SearchBody('Mod clmp ND-1')
        self.bodies['gripper_finger_2'] = self.system.SearchBody('Mod clmp ND-2')

    def _setup_gripper_collision(self):
        # Load collision mesh
        mesh = chrono.ChTriangleMeshConnected()
        mesh.LoadWavefrontMesh(os.path.join(self.data_dir, 'robot_arm_shapes/body_9_1.obj'))
        
        # Create a collision shape
        finger_shape = chrono.ChCollisionShapeTriangleMesh(self.material, mesh, False, False, 0.0)
        
        # Set collision margins
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)
        
        # Enable collision for gripper fingers
        self.bodies['gripper_finger_1'].AddCollisionShape(finger_shape)
        self.bodies['gripper_finger_2'].AddCollisionShape(finger_shape)
        self.bodies['gripper_finger_1'].EnableCollision(True)
        self.bodies['gripper_finger_2'].EnableCollision(True)

    def _setup_motors(self):
        # Create motors for various parts of the arm
        # self._create_motor('motor_base', 'base', 'base_motor_block', 1.0, None)
        motor_base = chrono.ChLinkMotorRotationAngle()
        motor_base_func  = chrono.ChFunctionConst(1.0)
        frame_1 = chrono.ChFramed(self.bodies['base'].GetPos(),chrono.Q_ROTATE_Z_TO_Y)
        motor_base.Initialize(self.bodies['base'], self.bodies['base_motor_block'],frame_1)
        self.system.Add(motor_base)
        self.motors['motor_base'] = motor_base

        self._create_motor('motor_base_arm', 'base_motor_block', 'base_arm', 1.0, 'Distance2')
        self._create_motor('motor_arms', 'base_arm', 'second_arm', -2.0, 'Distance4')

        self._create_motor('motor_arm_gripper', 'second_arm', 'arm_gripper_connector', 2.0, 'Concentric8')
        self._create_motor('motor_gripper', 'arm_gripper_connector', 'gripper', 1.7, 'Concentric7')
        
        # Create linear motors for the gripper blocks
        # self._create_linear_motor('motor_gripper_block_1', 'gripper', 'gripper_block_1', 0.03, 'Coincident12')
        # self._create_linear_motor('motor_gripper_block_2', 'gripper', 'gripper_block_2', -0.03, 'Coincident17')

        ### Create a motor between the gripper and the gripper_block_1
        motor_gripper_block_1 = chrono.ChLinkMotorLinearPosition()
        motor_gripper_block_1_func  = chrono.ChFunctionConst(0.03)
        joint_5 = my_system.SearchLink('Coincident12')
        joint_5_frame = chrono.ChFramed(joint_5.GetVisualModelFrame().GetPos(),chrono.Q_ROTATE_Z_TO_X)
        motor_gripper_block_1.Initialize(self.bodies['gripper'], self.bodies['gripper_block_1'], joint_5_frame)
        motor_gripper_block_1.SetMotionFunction(motor_gripper_block_1_func)
        self.system.Add(motor_gripper_block_1)
        self.motors['motor_gripper_block_1'] = motor_gripper_block_1

        motor_gripper_block_2 = chrono.ChLinkMotorLinearPosition()
        motor_gripper_block_2_func  = chrono.ChFunctionConst(-0.03)
        joint_6 = my_system.SearchLink('Coincident17')
        joint_6_frame = chrono.ChFramed(joint_6.GetVisualModelFrame().GetPos(),chrono.Q_ROTATE_Z_TO_X)
        motor_gripper_block_2.Initialize(self.bodies['gripper'], self.bodies['gripper_block_2'], joint_6_frame)
        motor_gripper_block_2.SetMotionFunction(motor_gripper_block_2_func)
        self.system.Add(motor_gripper_block_2)
        self.motors['motor_gripper_block_2'] = motor_gripper_block_2


    def _create_motor(self, name, body_a, body_b, speed, link_name):
        motor = chrono.ChLinkMotorRotationAngle()
        motor_func = chrono.ChFunctionConst(speed)
        link_frame = self.system.SearchLink(link_name).GetVisualModelFrame()
        motor.Initialize(self.bodies[body_a], self.bodies[body_b], link_frame)
        # motor.SetMotorFunction(motor_func)
        self.system.Add(motor)
        self.motors[name] = motor
        self.motor_funcs[name] = motor_func

    # def _create_linear_motor(self, name, body_a, body_b, position, link_name):
    #     motor = chrono.ChLinkMotorLinearPosition()
    #     motor_func = chrono.ChFunctionConst(position)
    #     link_frame = self.system.SearchLink(link_name).GetVisualModelFrame()
    #     motor.Initialize(self.bodies[body_a], self.bodies[body_b], link_frame)
    #     motor.SetMotionFunction(motor_func)
    #     self.system.Add(motor)
    #     self.motors[name] = motor

    def _setup_locks(self):
        # Create locks between gripper blocks and fingers
        self._create_lock('lock_gripper_finger_1', 'gripper_block_1', 'gripper_finger_2', 'Coincident10')
        self._create_lock('lock_gripper_finger_2', 'gripper_block_2', 'gripper_finger_1', 'Coincident14')

    def _create_lock(self, name, body_a, body_b, link_name):
        lock = chrono.ChLinkLockLock()
        link_frame = self.system.SearchLink(link_name).GetVisualModelFrame()
        lock.Initialize(self.bodies[body_a], self.bodies[body_b], link_frame)
        self.system.Add(lock)
        self.locks[name] = lock

    def _setup_ball(self):
        # Create a ball to grab
        self.ball = chrono.ChBodyEasySphere(0.05, 1, True, True, chrono.ChContactMaterialNSC())
        self.ball.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
        self.ball.SetPos(chrono.ChVector3d(-0.534622, 0.989506, 0.154854))
        self.ball.SetFixed(True)
        self.ball.EnableCollision(False)
        self.system.Add(self.ball)

        # Create lock for gripper-ball connection
        self.ball_gripper_lock = chrono.ChLinkLockLock()
        lock_frame = self.system.SearchLink('Coincident10').GetVisualModelFrame()
        self.ball_gripper_lock.Initialize(self.bodies['gripper'], self.ball, lock_frame)

    def control_gripper(self, action='pick'):
        if action == 'pick':
            self._gripper_pick()
        elif action == 'place':
            self._gripper_place()

    def _gripper_pick(self):
        func1 = chrono.ChFunctionConst(0.03)
        func2 = chrono.ChFunctionConst(-0.03)
        self.motors['motor_gripper_block_1'].SetMotionFunction(func1)
        self.motors['motor_gripper_block_2'].SetMotionFunction(func2)

    def _gripper_place(self):
        func1 = chrono.ChFunctionConst(0.05)
        func2 = chrono.ChFunctionConst(-0.05)
        self.motors['motor_gripper_block_1'].SetMotionFunction(func1)
        self.motors['motor_gripper_block_2'].SetMotionFunction(func2)

    def ball_contact_check(self):
        dist = (self.ball.GetPos() - self.bodies['gripper_finger_1'].GetPos()).Length()
        if self.ball_gripper_lock_flag and dist > 0.05:
            self.system.RemoveLink(self.ball_gripper_lock)
            self.ball_gripper_lock_flag = False
        if (not self.ball_gripper_lock_flag) and dist < 0.05:
            self.system.Add(self.ball_gripper_lock)
            self.ball_gripper_lock_flag = True

my_system = chrono.ChSystemNSC()
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

robot_arm = RobotArm(my_system)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht(robot_arm.system, chrono.ChVector3d(-2, 1, -1))
vis.EnableCollisionShapeDrawing(True)

timestep = 0.0005
render_step_size = 1.0 / 50  # FPS = 50
render_steps = math.ceil(render_step_size / timestep)
step_number = 0
render_frame = 0

rt_timer = chrono.ChRealtimeStepTimer()

# Set up the solver
robot_arm.system.GetSolver().AsIterative().SetMaxIterations(500)

# Main simulation loop
while vis.Run():
    robot_arm.ball_contact_check()

    sim_time = robot_arm.system.GetChTime()
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        filename = './IMG/img_' + str(render_frame) + '.jpg'
        vis.WriteImageToFile(filename)
        render_frame += 1
    
    robot_arm.system.DoStepDynamics(timestep)
    robot_arm.motors['motor_gripper'].SetAngleFunction(robot_arm.motor_funcs['motor_gripper'])
    
    if sim_time > 0.5:
        robot_arm.ball.EnableCollision(False)
        robot_arm.ball.SetFixed(False)
        robot_arm.motors['motor_base_arm'].SetAngleFunction(robot_arm.motor_funcs['motor_base_arm'])

    
    if sim_time > 1.0:
        robot_arm.motors['motor_base'].SetMotorFunction(chrono.ChFunctionConst(1.0))
    
    if sim_time > 1.5:
        robot_arm.control_gripper(action='place')
    
    rt_timer.Spin(timestep)
    step_number += 1
