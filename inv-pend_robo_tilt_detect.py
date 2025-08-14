import genesis as gs
import time # time module import
from scipy.spatial.transform import Rotation as R
import numpy as np

########################## init ##########################
gs.init(
    seed                = None,
    precision           = '32',
    debug               = False,
    eps                 = 1e-12,
    logging_level       = None,
    backend             = gs.cuda,
    theme               = 'dark',
    logger_verbose_time = False
)

########################## create a scene ##########################
scene = gs.Scene(
    sim_options=gs.options.SimOptions(
        dt=0.01,      # 10msec
        gravity=(0, 0, -0.8), # gravity set
    ),
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        res           = (1000, 800),
        camera_pos    = (3.5, 0.0, 2.0),
        camera_lookat = (0.0, 0.0, 0.0),
        camera_fov    = 4,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 0.1,
        show_link_frame  = False,
        plane_reflection = True,
        ambient_light    = (0.4, 0.4, 0.4),
    ),
    renderer=gs.renderers.Rasterizer(), # ラスタライザを使用
)

########################## entities ##########################
# 平面の床を設定
plane = scene.add_entity(gs.morphs.Plane())

# inv-pendulum本体の設定
invpend = scene.add_entity(
    gs.morphs.URDF( # MJCF, URDF
    	file='urdf/pendulum_robot_renew/Robot.urdf',

    	pos   = (0, 0, 0.1),
    ),
)

cam = scene.add_camera(
    res    = (1000, 800),
    pos    = (3.5, 0.0, 2.0),
    lookat = (0.0, 0.0, 0.0),
    fov    = 4,
    GUI    = False,
)

########################## build ##########################
scene.build()

#cam.start_recording()

print("Viewer is showing. \nWaiting for 1sec before starting simulation...")
time.sleep(1.0) # 待機
print("Starting simulation...")

jnt_names = [
    'Joint_gearbox_shaft',
]
dofs_idx = [invpend.get_joint(name).dof_idx_local for name in jnt_names]


# PD 制御係数
xdeg_now = 0.0
xdeg_last = 0.0
xdeg_target = 17.30  # 18.55 # 19.5 21.3 25.5


# Hard reset
for i in range(300):
    xdeg_last = xdeg_now
    scene.step()
    
    try:
        # 'Link_frame'の姿勢を取得
        link_position = invpend.get_links_pos()[0].cpu()
        link_quaternion = invpend.get_links_quat()[0].cpu() # Numpyに渡すためにcpuへ転送
        # genesis の auaternion は wxyz の順、 phthon の quat(scipy_quat) は xyzw の順
        scipy_quat = np.array([link_quaternion[1], link_quaternion[2], link_quaternion[3], link_quaternion[0]])
        r = R.from_quat(scipy_quat)
        euler_angles_deg = r.as_euler('xyz', degrees=True)
        xdeg_now = euler_angles_deg[0]
#        euler_angles_rad = r.as_euler('xyz', degrees=False)

        print(f"{i}, {euler_angles_deg[0]:.2f}, {euler_angles_deg[1]:.2f}, {euler_angles_deg[2]:.2f}")
        print(f"{link_position[0]:.4f}, {link_position[1]:.4f}, {link_position[2]:.4f}")

    except AttributeError:
        print("error: can not find the method!")
    except Exception as e:
        print(f"An error has occurred!: {e}")

    
    #cam.render()
#cam.stop_recording(save_to_filename='pend_tilt_0812.mp4', fps=60)

