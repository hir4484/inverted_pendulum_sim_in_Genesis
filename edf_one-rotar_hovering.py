import genesis as gs
import time # time module import
from scipy.spatial.transform import Rotation as R
import numpy as np
from genesis.engine.solvers.rigid.rigid_solver_decomp import RigidSolver

########################## init ##########################
gs.init(backend=gs.cuda)
    #backend             = gs.cuda,


########################## create a scene ##########################
scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt=0.01,      # 1msec
        gravity=(0, 0, -9.8), # gravity set -9.8   0.001/10 = 1/10000
    ),
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        
        res           = (1000, 1000),
        camera_pos    = (3.5, 0.2, 2.0),
        camera_lookat = (-0.25, 0.2, 0.2),
        camera_fov    = 15,
        max_FPS=60,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = False,
        world_frame_size = 0.05,
        show_link_frame  = True,
        link_frame_size  = 0.06,
        plane_reflection = False,
        ambient_light    = (0.4, 0.4, 0.4),
    ),
    renderer=gs.renderers.Rasterizer(), # ラスタライザを使用
)

########################## entities ##########################
# 平面の床を設定
plane = scene.add_entity(gs.morphs.Plane())

# inv-pendulum本体の設定
edf = scene.add_entity(
    gs.morphs.URDF(
    	file='urdf/edf_rotar_2_test/Robot.urdf',
    	pos    = (0, 0, 0.2), #0.062
    ),
)

cam = scene.add_camera(
    res    = (1000, 1000),
    pos    = (3.5, 0.2, 2.0),
    lookat = (-0.25, 0.2, 0.2),
    fov    = 15,
    GUI    = False,
)

########################## build ##########################
scene.build()

#cam.start_recording()

time.sleep(1.0) # 待機
print("Starting simulation...")

jnt_names = [
    'Joint_frame_rotar',
]
dofs_idx = [edf.get_joint(name).dof_idx_local for name in jnt_names]

# URDFファイル内の推力を発生させるリンクの正確な名前を指定する。
edf_propeller_link_name = 'l_part_rotar'
# print(edf_propeller_link_name)



# PD 制御係数
xdeg_now = 0.0
rpm_value = 1.0
rad_per_sec = 1.0

# EDF propeller speed set
for i in range(1000):
    if i < 50: #50
        rpm_value = 0
    elif i < 100:
        rpm_value = 100
    elif i < 200:
        rpm_value = 6000
    elif i < 400:
        rpm_value = 7000
    elif i < 500:
        rpm_value = 6800
    elif i < 600:
        rpm_value = 6790
    else:
        rpm_value = 6930 # 6940 hover, 6800 stall

        # 14468 is hover rpm ==> 49600 is hover rpm
        #drone.set_propellels_rpm((1 + 0.05 * traj[i]) * 14468.429183500699)
    rad_per_sec = (rpm_value * 2 * 3.1415) / 60

    edf.control_dofs_velocity(np.array([rad_per_sec]), dofs_idx)


    # 推力を計算: F_thrust = Ct * rho * n^2 * D^4
    #thrust_force_magnitude = thrust_coefficient * air_density * (rps_value**2) * (propeller_diameter**4)
    thrust_force_magnitude = 4.9 * rpm_value / 49680

    # 推力はEDFのローカルZ軸方向に作用すると仮定し、世界座標系で適用
    # EDFの現在の姿勢を取得し、ローカルZ軸を世界座標に変換
    # genesis の auaternion は wxyz の順、 phthon の quat(scipy_quat) は xyzw の順
    link_quaternion = edf.get_links_quat()[0].cpu() # Numpyに渡すためにcpuへ転送
    # SciPyのRotationはxyzw順なので変換
    scipy_quat = np.array([link_quaternion[1], link_quaternion[2], link_quaternion[3], link_quaternion[0]])
    r = R.from_quat(scipy_quat)
    
    # ローカルZ軸 (0,0,1) を現在の回転で世界座標に変換
    local_z_axis = np.array([0, 0, 1])
    world_z_axis_transformed = r.apply(local_z_axis)

    # 世界座標系における推力ベクトルに変換
    thrust_vector_world = world_z_axis_transformed * thrust_force_magnitude

    # 力を適用するリンクのローカルインデックスを取得
    try:
        propeller_link_idx = edf.get_link(edf_propeller_link_name).idx_local
        #print(propeller_link_idx) # ==> entity INDEX; 0, plane, 1, link name="l_part_rotar", 2, link name="l_part_frame"

        # scene.sim.rigid_solver.apply_links_external_force(force = thrust_vector_world, links_idx = [propeller_link_idx])
        scene.sim.rigid_solver.apply_links_external_force(force = thrust_vector_world[np.newaxis, :], links_idx = [propeller_link_idx])
    except AttributeError:
        print(f"Error: Link '{edf_propeller_link_name}' not found in EDF URDF. Please check the 'edf_propeller_link_name' variable.")
        break
    # --- 推力の計算と適用 終わり ---

    # invpend.control_dofs_force(np.array([0.0]), dofs_idx) 
    scene.step()
    
    try:
     # 'Link_frame'の座標を取得
        link_position = edf.get_links_pos()[0].cpu()
        #link_quaternion = edf.get_links_quat()[0].cpu() # Numpyに渡すためにcpuへ転送
        # genesis の auaternion は wxyz の順、 phthon の quat(scipy_quat) は xyzw の順
        #scipy_quat = np.array([link_quaternion[1], link_quaternion[2], link_quaternion[3], link_quaternion[0]])
        #r = R.from_quat(scipy_quat)
        euler_angles_deg = r.as_euler('xyz', degrees=True)
        xdeg_now = euler_angles_deg[0]
        # euler_angles_rad = r.as_euler('xyz', degrees=False)

        # output parameter
        print(f"{i}, {rpm_value:.1f}, {rad_per_sec:.1f}, {thrust_force_magnitude:.4f}, {euler_angles_deg[0]:.2f}, {euler_angles_deg[1]:.2f}, {euler_angles_deg[2]:.2f}")
        # output position
        print(f"{link_position[0]:.4f}, {link_position[1]:.4f}, {link_position[2]:.4f}")

    except AttributeError:
        print("error: can not find the method!")
    except Exception as e:
        print(f"An error has occurred!: {e}")

    
    #cam.render()
#cam.stop_recording(save_to_filename='edf_video_hover_0722.mp4', fps=60)