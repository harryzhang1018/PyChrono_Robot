# PyChrono model automatically generated using Chrono::SolidWorks add-in
# Assembly: C:\Users\sbel\Downloads\robotiq_hand_e\robotiq_hand_e\hand_e_asd.SLDASM


import pychrono as chrono 
import builtins 

# Some global settings 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'robot_hand_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0 = chrono.ChBodyAuxRef()
body_0.SetName('SLDW_GROUND')
body_0.SetFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1 = chrono.ChBodyAuxRef()
body_1.SetName('HAND_e_finger-1-1')
body_1.SetPos(chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428))
body_1.SetRot(chrono.ChQuaterniond(0.707106781186548,1.74315279842105e-32,-1.37140828284124e-16,0.707106781186547))
body_1.SetMass(0.0938874355859725)
body_1.SetInertiaXX(chrono.ChVector3d(3.31466514037026e-05,2.45304814291191e-05,3.68005499125014e-05))
body_1.SetInertiaXY(chrono.ChVector3d(4.73448371552155e-06,-1.6596537866072e-06,7.80573433834422e-06))
body_1.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(0.00724735083156416,0.020647536888227,-0.022421500455029),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChVisualShapeModelFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_1)



# Rigid body part
body_2 = chrono.ChBodyAuxRef()
body_2.SetName('Hand_base_and_p07-1')
body_2.SetPos(chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428))
body_2.SetRot(chrono.ChQuaterniond(-6.8570414142062e-17,0.707106781186548,-0.707106781186547,6.8570414142062e-17))
body_2.SetMass(0.38232347669012)
body_2.SetInertiaXX(chrono.ChVector3d(0.000560005407272658,0.000562334354846062,0.000200649640246989))
body_2.SetInertiaXY(chrono.ChVector3d(6.41346215573423e-10,1.55107482746673e-07,-1.57001286264211e-06))
body_2.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(1.94804546979808e-07,-0.0397712704827772,7.29741759192392e-05),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChVisualShapeModelFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_2)



# Rigid body part
body_3 = chrono.ChBodyAuxRef()
body_3.SetName('HAND_e_finger-2-1')
body_3.SetPos(chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428))
body_3.SetRot(chrono.ChQuaterniond(0.707106781186548,-1.37140828284124e-16,-8.71576399210525e-33,0.707106781186547))
body_3.SetMass(0.0938874355859725)
body_3.SetInertiaXX(chrono.ChVector3d(3.31467659305197e-05,2.45303153855084e-05,3.6800601429295e-05))
body_3.SetInertiaXY(chrono.ChVector3d(-4.73444367209372e-06,-1.65947246859143e-06,-7.80569347513243e-06))
body_3.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-0.00724737975140957,0.0206478134281555,0.0224215440569809),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChVisualShapeModelFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3.AddVisualShape(body_3_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_3)




# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_0 , SW name: Hand_base_and_p07-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_0 , SW name: HAND_e_finger-1-1 ,  SW ref.type:4 (4)
link_1 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(1.93946419314488e-16,0,-1)
dB = chrono.ChVector3d(-1.93946419314488e-16,-1.93946419314488e-16,1)
link_1.Initialize(body_2,body_1,False,cA,cB,dB)
link_1.SetDistance(0)
link_1.SetName("Coincident7")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(1.93946419314488e-16,0,-1)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dB = chrono.ChVector3d(-1.93946419314488e-16,-1.93946419314488e-16,1)
link_2.SetFlipped(True)
link_2.Initialize(body_2,body_1,False,cA,cB,dA,dB)
link_2.SetName("Coincident7")
exported_items.append(link_2)


# Mate constraint: Coincident8 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_0 , SW name: Hand_base_and_p07-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_0 , SW name: HAND_e_finger-1-1 ,  SW ref.type:4 (4)
link_3 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(-1,-6.15242222082669e-17,-1.93946419314488e-16)
dB = chrono.ChVector3d(-1,-6.15242222082669e-17,-1.93946419314488e-16)
link_3.Initialize(body_2,body_1,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident8")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(-1,-6.15242222082669e-17,-1.93946419314488e-16)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dB = chrono.ChVector3d(-1,-6.15242222082669e-17,-1.93946419314488e-16)
link_4.Initialize(body_2,body_1,False,cA,cB,dA,dB)
link_4.SetName("Coincident8")
exported_items.append(link_4)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_0 , SW name: Hand_base_and_p07-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_0 , SW name: HAND_e_finger-1-1 ,  SW ref.type:4 (4)
link_5 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(6.15242222082669e-17,-1,0)
dB = chrono.ChVector3d(-6.1524222208267e-17,1,1.93946419314488e-16)
link_5.Initialize(body_2,body_1,False,cA,cB,dB)
link_5.SetDistance(0)
link_5.SetName("Coincident9")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(6.15242222082669e-17,-1,0)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dB = chrono.ChVector3d(-6.1524222208267e-17,1,1.93946419314488e-16)
link_6.SetFlipped(True)
link_6.Initialize(body_2,body_1,False,cA,cB,dA,dB)
link_6.SetName("Coincident9")
exported_items.append(link_6)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_0 , SW name: Hand_base_and_p07-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_0 , SW name: HAND_e_finger-2-1 ,  SW ref.type:4 (4)
link_7 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(1.93946419314488e-16,0,-1)
dB = chrono.ChVector3d(-1.93946419314488e-16,1.93946419314488e-16,1)
link_7.Initialize(body_2,body_3,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("Coincident10")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(1.93946419314488e-16,0,-1)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dB = chrono.ChVector3d(-1.93946419314488e-16,1.93946419314488e-16,1)
link_8.SetFlipped(True)
link_8.Initialize(body_2,body_3,False,cA,cB,dA,dB)
link_8.SetName("Coincident10")
exported_items.append(link_8)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_0 , SW name: Hand_base_and_p07-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_0 , SW name: HAND_e_finger-2-1 ,  SW ref.type:4 (4)
link_9 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(-1,-6.15242222082669e-17,-1.93946419314488e-16)
dB = chrono.ChVector3d(-1,-6.15242222082669e-17,-1.93946419314488e-16)
link_9.Initialize(body_2,body_3,False,cA,cB,dB)
link_9.SetDistance(0)
link_9.SetName("Coincident11")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(-1,-6.15242222082669e-17,-1.93946419314488e-16)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dB = chrono.ChVector3d(-1,-6.15242222082669e-17,-1.93946419314488e-16)
link_10.Initialize(body_2,body_3,False,cA,cB,dA,dB)
link_10.SetName("Coincident11")
exported_items.append(link_10)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_0 , SW name: Hand_base_and_p07-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_0 , SW name: HAND_e_finger-2-1 ,  SW ref.type:4 (4)
link_11 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(6.15242222082669e-17,-1,0)
dB = chrono.ChVector3d(-6.15242222082669e-17,1,-1.93946419314488e-16)
link_11.Initialize(body_2,body_3,False,cA,cB,dB)
link_11.SetDistance(0)
link_11.SetName("Coincident12")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dA = chrono.ChVector3d(6.15242222082669e-17,-1,0)
cB = chrono.ChVector3d(-0.449700001879166,0.629999999999998,-0.00974999999995428)
dB = chrono.ChVector3d(-6.15242222082669e-17,1,-1.93946419314488e-16)
link_12.SetFlipped(True)
link_12.Initialize(body_2,body_3,False,cA,cB,dA,dB)
link_12.SetName("Coincident12")
exported_items.append(link_12)

