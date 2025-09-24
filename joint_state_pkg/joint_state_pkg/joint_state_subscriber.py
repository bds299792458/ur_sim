import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import numpy as np
import glfw

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("joint_state_subscriber")
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10
        )
        
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path(
            '/home/bds/proj/mujoco/mujoco_menagerie/universal_robots_ur5e/scene.xml'
        )
        self.data = mujoco.MjData(self.model)

        # 初始化关节位置存储 (UR5e 6个关节)
        self.positions = np.zeros(6)

        # 打印所有 body 的 ID 和名称
        print("All bodies in the model:")
        for i in range(self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            print(f"ID: {i}, Name: {body_name}")

        # 初始化 GLFW
        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW")

        self.window = glfw.create_window(1200, 900, 'UR5e Control', None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window")

        glfw.make_context_current(self.window)

        # 鼠标状态
        self.last_x = 0
        self.last_y = 0
        self.button_left = False
        self.button_right = False
        self.button_middle = False

        # GLFW 鼠标回调
        glfw.set_scroll_callback(self.window, self.scroll_callback)
        glfw.set_mouse_button_callback(self.window, self.mouse_button_callback)
        glfw.set_cursor_pos_callback(self.window, self.cursor_pos_callback)

        # 初始化渲染器
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultCamera(self.cam)
        mujoco.mjv_defaultOption(self.opt)
        self.pert = mujoco.MjvPerturb()
        self.con = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)

        # 找到末端执行器的 body id
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'wrist_3_link')
        if self.end_effector_id == -1:
            print("Warning: Could not find the end effector with the given name.")

        # 初始关节角度
        self.initial_q = self.data.qpos[:6].copy()

        # 定时器 100Hz 更新
        self.timer1 = self.create_timer(0.01, self.timer_callback1)

    # ---------- 鼠标回调 ----------
    def scroll_callback(self, window, xoffset, yoffset):
        self.cam.distance *= 1 - 0.1 * yoffset

    def mouse_button_callback(self, window, button, action, mods):
        if button == glfw.MOUSE_BUTTON_LEFT:
            self.button_left = (action == glfw.PRESS)
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            self.button_right = (action == glfw.PRESS)
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            self.button_middle = (action == glfw.PRESS)
        self.last_x, self.last_y = glfw.get_cursor_pos(window)

    def cursor_pos_callback(self, window, xpos, ypos):
        dx = xpos - self.last_x
        dy = ypos - self.last_y
        self.last_x, self.last_y = xpos, ypos

        if self.button_left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif self.button_right:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_middle:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        else:
            return

        mujoco.mjv_moveCamera(self.model, action, dx / 120, dy / 120, self.scene, self.cam)

    # ---------- ROS 回调 ----------
    def callback(self, msg: JointState):
        ur5_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.positions = [msg.position[msg.name.index(j)] for j in ur5_joints if j in msg.name]

        for name, pos in zip(ur5_joints, self.positions):
            self.get_logger().info(f"{name}: {pos:.4f}")

    # ---------- 定时器回调 ----------
    def timer_callback1(self):
        if not glfw.window_should_close(self.window):
            # 更新关节
            self.data.qpos[:6] = self.positions
            mujoco.mj_forward(self.model, self.data)
            mujoco.mj_step(self.model, self.data)

            # 更新渲染场景
            viewport = mujoco.MjrRect(0, 0, 1200, 900)
            mujoco.mjv_updateScene(
                self.model, self.data, self.opt, self.pert, self.cam,
                mujoco.mjtCatBit.mjCAT_ALL.value, self.scene
            )
            mujoco.mjr_render(viewport, self.scene, self.con)

            glfw.swap_buffers(self.window)
            glfw.poll_events()

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        glfw.terminate()

if __name__ == "__main__":
    main()
