import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

class RigidTerrainEnvironment:
    def __init__(self):
        self.system = chrono.ChSystemNSC()
        self.system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        self.terrain = None
        self.vis = None
        self.timestep = 0.001
        self.render_step_size = 1.0 / 25  # FPS = 50
        self.render_steps = math.ceil(self.render_step_size / self.timestep)
        self.step_number = 0
        self.render_frame = 0
        self.rt_timer = chrono.ChRealtimeStepTimer()

    def create_terrain(self):
        patch_mat = chrono.ChContactMaterialNSC()
        self.terrain = veh.RigidTerrain(self.system)
        patch = self.terrain.AddPatch(patch_mat, 
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.3, 0), chrono.Q_ROTATE_Z_TO_Y), 
            100, 100)

        patch.SetTexture(chrono.GetChronoDataPath()+"vehicle/terrain/textures/tile4.jpg", 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        self.terrain.Initialize()

    def setup_visualization(self):
        self.vis = chronoirr.ChVisualSystemIrrlicht(self.system, chrono.ChVector3d(-2, 1, -1))
        self.vis.EnableCollisionShapeDrawing(True)

    def setup_solver(self):
        self.system.GetSolver().AsIterative().SetMaxIterations(500)

    def simulate(self):
        # self.initialize()
        # while self.vis.Run():
        sim_time = self.system.GetChTime()
        if self.step_number % self.render_steps == 0:
            self.vis.BeginScene()
            self.vis.Render()
            self.vis.EndScene()
            # Uncomment the following lines if you want to save frames
            # filename = f'./IMG_jackal/img_{self.render_frame}.jpg'
            # self.vis.WriteImageToFile(filename)
            # self.render_frame += 1

        self.terrain.Synchronize(sim_time)
        self.system.DoStepDynamics(self.timestep)
        self.terrain.Advance(self.timestep)

        self.rt_timer.Spin(self.timestep)
        self.step_number += 1

    def initialize(self):
        self.create_terrain()
        self.setup_visualization()
        self.setup_solver()
