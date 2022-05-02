import sys
import pybullet as physics

import HarfangHighLevel as hl
from math import pi

# Initialize PyBullet
clid = physics.connect(physics.DIRECT)
physics.setPhysicsEngineParameter(enableConeFriction=0)
physics.setAdditionalSearchPath(".")

# All Reachy's information needed to calculate the inverse kinematic.
# The Reachy structure is Loaded from the URDF
class Reachy_Robot():
    reachy_path = "models/reachy2.URDF"

    reachyId = physics.loadURDF(reachy_path, useFixedBase=True)
    for jointIndex in range(physics.getNumJoints(reachyId)):
        physics.resetJointState(reachyId, jointIndex, 0)

    # Debug log
    # for i in range(physics.getNumJoints(reachyId)):
    #     print(physics.getJointInfo(reachyId, i))

    numJoints = physics.getNumJoints(reachyId)

    right_arm_end_effector_index = 8
    left_arm_end_effector_index = 17


# Match the URDF joints names with the Blender Reachy model nodes names
reachy_3D_link_to_join = {
    "pedestal": "chest",
    "r_shoulder_pitch":"right_shoulder_pitch_joint",
    "r_shoulder_roll":"right_shoulder_roll_joint",
    "r_arm_yaw":"right_arm_yaw_joint", 
    "r_elbow_pitch":"right_elbow_pitch_joint", 
    "r_forearm_yaw":"right_forearm_yaw_joint", 
    "r_wrist_pitch":"right_wrist_pitch_joint",
    "r_wrist_roll":"right_wrist_roll_joint", 
    "r_gripper":"right_gripper_joint",
    "l_shoulder_pitch":"left_shoulder_pitch_joint",
    "l_shoulder_roll":"left_shoulder_roll_joint",
    "l_arm_yaw":"left_arm_yaw_joint",
    "l_elbow_pitch":"left_elbow_pitch_joint",
    "l_forearm_yaw":"left_forearm_yaw_joint",
    "l_wrist_pitch":"left_wrist_pitch_joint",
    "l_wrist_roll":"left_wrist_roll_joint",
    "l_gripper":"left_gripper_joint"
    }

# Angular biases applied to the Blender Reachy's structure which is not based on the URDF
joint_node_offset = {
    "pedestal": {"r":-pi/2.0, "inverse_angle":False},
    "r_shoulder_pitch": {"r":pi/2.0, "inverse_angle":False},
    "r_shoulder_roll": {"r":pi/2.0, "inverse_angle":False},
    "r_arm_yaw":{"r":0, "inverse_angle":False},
    "r_elbow_pitch":{"r":0, "inverse_angle":False},
    "r_forearm_yaw":{"r":0, "inverse_angle":False},
    "r_wrist_pitch":{"r":0, "inverse_angle":False},
    "r_wrist_roll":{"r":0, "inverse_angle":False},
    "r_gripper":{"r":0, "inverse_angle":False},
    "l_shoulder_pitch": {"r":pi/2.0, "inverse_angle":True},
    "l_shoulder_roll": {"r":pi/2.0, "inverse_angle":True},
    "l_arm_yaw": {"r":pi, "inverse_angle":False},
    "l_elbow_pitch":{"r":0, "inverse_angle":False},
    "l_forearm_yaw":{"r":0, "inverse_angle":False},
    "l_wrist_pitch":{"r":0, "inverse_angle":False},
    "l_wrist_roll":{"r":0, "inverse_angle":False},
    "l_gripper":{"r":0, "inverse_angle":False},
    }
    

def clamp(v, v_min, v_max):
    return max(v_min, min(v, v_max))


def gui_debug_model_rotation():
    # GUI debug panel to adjust the rotation of all 3D parts of the Reachy. Allows to debug every rotation problems with ik
    if hl.ImGuiBegin("Angles", False, hl.ImGuiWindowFlags_AlwaysAutoResize):
        for name, val in joint_node_offset.items():
            change, joint_node_offset[name]["r"] = hl.ImGuiSliderFloat(f"angle {name}", val["r"], -pi, pi)
            hl.ImGuiSameLine()
            change, joint_node_offset[name]["inverse_angle"] = hl.ImGuiCheckbox(
                "invert angle##right" + name, val["inverse_angle"])

        for name, val in joint_node_offset.items():
            change, joint_node_offset[name]["r"] = hl.ImGuiSliderFloat(f"angle {name}", val["r"], -pi, pi)
            hl.ImGuiSameLine()
            change, joint_node_offset[name]["inverse_angle"] = hl.ImGuiCheckbox(
                "invert angle##left" + name, val["inverse_angle"])
    hl.ImGuiEnd()


def gui_debug_controller_input(controllers):
    hl.ImGuiText("controller test")
    connected_controller = update_connected_controller(controllers)
    for controller_name in connected_controller:
        controller = controllers[controller_name]

        pos = hl.GetT(controller["world"])
        rot = hl.GetR(controller["world"])
        hl.ImGuiText(f"pos: {pos.x}, {pos.y}, {pos.z}")
        hl.ImGuiText(f"rot: {rot.x}, {rot.y}, {rot.z}")

        key_name = ['VRCB_DPad_Up', 'VRCB_DPad_Down', 'VRCB_DPad_Left', 'VRCB_DPad_Right', 'VRCB_System',
                    'VRCB_AppMenu', 'VRCB_Grip', 'VRCB_A',
                    'VRCB_ProximitySensor', 'VRCB_Axis0', 'VRCB_Axis1', 'VRCB_Axis2', 'VRCB_Axis3',
                    'VRCB_Axis4', 'VRCB_Count']

        controller_input = controller["input"]
        for i in range(hl.VRCB_Count):
            hl.ImGuiText(f"{key_name[i]}: Down: {controller_input.Down(i)}, Pressed: {controller_input.Pressed(i)}, Released: {controller_input.Released(i)}")
            hl.ImGuiText(f"			   Touch: {controller_input.Touch(i)}, TouchStart: {controller_input.TouchStart(i)}, TouchEnd: {controller_input.TouchEnd(i)}")

        for i in range(5):
            Surface = controller_input.Surface(i)
            hl.ImGuiText(f"Surface{i}: x: {Surface.x}, y: {Surface.y}")
            DtSurface = controller_input.DtSurface(i)
            hl.ImGuiText(f"DtSurface{i}: x: {DtSurface.x}, y: {DtSurface.y}")


def update_keyboard_command_right_arm(target_pos, angle_wrist_pitch, angle_wrist_roll, pinch):
    if hl.gVal.keyboard.Down(hl.K_U):
        target_pos.x += 0.01
    elif hl.gVal.keyboard.Down(hl.K_J):
        target_pos.x -= 0.01
    if hl.gVal.keyboard.Down(hl.K_H):
        target_pos.z += 0.01
    elif hl.gVal.keyboard.Down(hl.K_K):
        target_pos.z -= 0.01
    if hl.gVal.keyboard.Down(hl.K_Y):
        target_pos.y += 0.01
    elif hl.gVal.keyboard.Down(hl.K_I):
        target_pos.y -= 0.01

    if hl.gVal.keyboard.Down(hl.K_Numpad8):
        angle_wrist_roll += 0.05
    elif hl.gVal.keyboard.Down(hl.K_Numpad2):
        angle_wrist_roll -= 0.05
    if hl.gVal.keyboard.Down(hl.K_Numpad4):
        angle_wrist_pitch -= 0.05
    elif hl.gVal.keyboard.Down(hl.K_Numpad6):
        angle_wrist_pitch += 0.05
    angle_wrist_roll = clamp(angle_wrist_roll, -0.5, 0.6)
    angle_wrist_pitch = clamp(angle_wrist_pitch, -0.5, 0.5)

    if hl.gVal.keyboard.Down(hl.K_Numpad7):
        pinch -= 0.05
    elif hl.gVal.keyboard.Down(hl.K_Numpad9):
        pinch += 0.05
    pinch = clamp(pinch, -0.290, 0.7)

    return target_pos, angle_wrist_pitch, angle_wrist_roll, pinch


def update_connected_controller(controllers):
    # Check if a new controller just came in
    connected_controller = []

    vr_controller_names = hl.GetVRControllerNames()
    for n in vr_controller_names:
        # Get all possible controllers and add them to the main controller's dict
        if n not in controllers:
            controller = hl.VRController(n)
            controllers[n] = {"input": controller, "world": hl.Mat4()}


        # If the controller is connected, we need to setup it
        controller = controllers[n]["input"]
        controller.Update()
        if controller.IsConnected():
            #
            connected_controller.append(n)
            controller_mat = controller.World()
            controllers[n]["world"] = controller_mat

    return connected_controller


def calibrate_VR_head():
    # Align the VR body on the Reachy. The VR head is aligned on the Reachy's head,
    # and the 3D model of the head is hidden
    actual_head_pos = hl.GetTranslation(hl.gVal.vr_state.head)
    ideal_head_pos = hl.GetT(hl.gVal.scene.GetNode("front_head").GetTransform().GetWorld())
    head_calibration_offset = ideal_head_pos - actual_head_pos
    body_mtx = hl.gVal.ground_vr_mat + hl.TranslationMat4(head_calibration_offset)
    body_pos = hl.GetT(body_mtx)
    hl.SetVRGroundAnchor(body_pos.x, body_pos.y, body_pos.z, angle_y=0)
    hide_reachy_head()


def calibrate_camera():
    ideal_head_pos = hl.GetT(hl.gVal.scene.GetNode("front_head").GetTransform().GetWorld())
    hl.gVal.camera.GetTransform().SetPos(ideal_head_pos)
    hl.gVal.camera.GetTransform().SetRot(hl.Vec3(0, pi/2.0, 0))


def create_vr_controller():
    vive_left = hl.CreateInstanceFromAssets(hl.gVal.scene, hl.Mat4(), "vive_controller/vive_controller.scn",
                                            hl.gVal.res, hl.GetForwardPipelineInfo())
    vive_left[0].SetName("vive_controller_left")
    vive_left[0].Disable()
    vive_right = hl.CreateInstanceFromAssets(hl.gVal.scene, hl.Mat4(), "vive_controller/vive_controller.scn",
                                             hl.gVal.res, hl.GetForwardPipelineInfo())
    vive_right[0].SetName("vive_controller_right")
    vive_right[0].Disable()
    oculus_left = hl.CreateInstanceFromAssets(hl.gVal.scene, hl.Mat4(), "oculus_controller/oculus_controller_left.scn",
                                              hl.gVal.res, hl.GetForwardPipelineInfo())
    oculus_left[0].Disable()
    oculus_right = hl.CreateInstanceFromAssets(hl.gVal.scene, hl.Mat4(),
                                               "oculus_controller/oculus_controller_right.scn", hl.gVal.res,
                                               hl.GetForwardPipelineInfo())
    oculus_right[0].Disable()
    controller_instances = [{"left": oculus_left, "right": oculus_right},
                            {"left": vive_left, "right": vive_right}]

    return controller_instances


def hide_reachy_head():
    disable_children_node(hl.gVal.scene.GetNode("neck_interface"))


def show_reachy_head():
    enable_children_node(hl.gVal.scene.GetNode("neck_interface"))


def disable_children_node(main_node):
    for node in hl.gVal.scene.GetNodeChildren(main_node):
        if node.HasObject():
            node.Disable()
        disable_children_node(node)


def enable_children_node(main_node):
    for node in hl.gVal.scene.GetNodeChildren(main_node):
        if node.HasObject():
            node.Enable()
        enable_children_node(node)


def main(argv):
    # ----------- VARIABLES ----------

    # Get args to set VR True or False (True by default, set False if not wanted or if something wrong)
    want_activate_VR = True  # Activate VR by default, allow to control the ik targets with the VR controllers

    # If app started with no vr, deactivate VR
    for i in range(len(argv)):
        cmd = argv[i]
        if cmd == "no_vr":
            want_activate_VR = False

    # The left and right correspondance between VR controller and reachy's hand
    left_controller_name = ""
    right_controller_name = ""
    flag_set_left_controller = True
    flag_set_default_left_controller = True
    flag_set_right_controller = True
    flag_set_default_right_controller = True

    controllers = {}  # Keep track of the controllers

    flag_reachy_head_hidden = False

    # used to calibrate camera or head on reachy's head
    calibration_timer = 2  # Time (in s) the user has to press start button
    calibration_timer_cptr = 0

    # Without VR, the right hand can be controlled with the keyboard by activing this flag. This allow to move the target, the wrist and the pinch.
    # Target's position of both hands can also be controlled with ui slidders.
    keyboard_override = True

    # ----------- PREPARE SCENE ----------
    hl.Init(1920, 1080, want_activate_VR) # if VR not detected but wanted, hl.gVal.activate VR is turned to False, and all the app works without VR

    hl.LoadSceneFromAssets("reachy.scn", hl.gVal.scene, hl.gVal.res, hl.GetForwardPipelineInfo())

    hl.AddFpsCamera(2, 1.5, 0, pi / 8, - pi / 2)  # Set the current camera as a fps camera
    hl.gVal.scene.SetCurrentCamera(hl.gVal.camera)

    calibration_gauge_node = hl.gVal.scene.GetNode("calibration_gauge_container")
    calibration_gauge_max_scale = calibration_gauge_node.GetTransform().GetScale().x
    calibration_gauge_node.GetTransform().SetScale(hl.Vec3(-1, 1, 1))

    controller_left_node = None
    controller_right_node = None
    controller_instances = []
    node_controller_type = 1 # 0 = "Oculus" 1 = "Vive"
    if (hl.gVal.activate_VR):
        controller_instances = create_vr_controller()
        controller_left_node = controller_instances[node_controller_type]["left"][0]
        controller_left_node.Enable()
        controller_right_node = controller_instances[node_controller_type]["right"][0]
        controller_right_node.Enable()

    # ------------------- REACHY -------------------
    reachy = Reachy_Robot() # Creates main structure with Reachy informations
    reachy_position = hl.Vec3(0, 1, 0)

    # Targets for Reachy's inverss kinematic hands
    vec_pos_target_right = hl.Vec3(0.062, 0.625, -0.131)
    vec_pos_target_left = hl.Vec3(0.062, 0.625, 0.131)

    # Link URDF joint names with the corresponding harfang nodes
    harfang_node = {}    
    for i in range (reachy.numJoints):
        jointInfo = physics.getJointInfo(reachy.reachyId, i)
        joint_name = jointInfo[1].decode()
        if joint_name in reachy_3D_link_to_join:
            node_name = reachy_3D_link_to_join[joint_name]
            node = hl.gVal.scene.GetNode(node_name)
            if node != hl.NullNode:
                harfang_node[joint_name] = node

    # Main HARFANG loop
    while not hl.UpdateDraw():
        physics.stepSimulation()

        controller_swap_side = False

        # gui_debug_model_rotation()

        # GUI panel to manually set targets for both Reachy's arms (without VR) or to invert hands controlled by the VR controllers (with VR).
        if hl.ImGuiBegin("Window"):

            if hl.gVal.activate_VR:
                change, node_controller_type = hl.ImGuiCombo("Controller type", node_controller_type, ["Oculus", "Vive"])
                if change:
                    controller_left_node.Disable()
                    controller_right_node.Disable()

                    controller_instance = controller_instances[node_controller_type]
                    controller_left_node = controller_instance["left"][0]
                    controller_left_node.Enable()
                    controller_right_node = controller_instance["right"][0]
                    controller_right_node.Enable()

                # Changes the hand controlled in case of only one controller used
                if hl.ImGuiButton("SetLeft"):
                    flag_set_left_controller = True
                if hl.ImGuiButton("SetRight"):
                    flag_set_right_controller = True

                # Inverts the hand associated to each controllers, in case of two controllers used
                if hl.ImGuiButton("Invert hand"):
                    controller_swap_side = True

            else:
                change, vec_pos_target_right = hl.ImGuiSliderVec3("Target pos right hand", vec_pos_target_right, -0.5, 1.0)

                change, vec_pos_target_left = hl.ImGuiSliderVec3("Target pos left hand", vec_pos_target_left, -0.5, 1.0)

        hl.ImGuiEnd()

        dist_to_reachy = 0
        flag_activate_calibration = False

        # Update of the Reachy arm control with keyboard without VR (only right arm)
        if not hl.gVal.activate_VR:
            if keyboard_override:
                vec_pos_target_right, \
                joint_node_offset["r_wrist_pitch"]["r"], \
                joint_node_offset["r_wrist_roll"]["r"], \
                joint_node_offset["r_gripper"]["r"] = update_keyboard_command_right_arm(vec_pos_target_right,
                                                             joint_node_offset["r_wrist_pitch"]["r"],
                                                             joint_node_offset["r_wrist_roll"]["r"],
                                                             joint_node_offset["r_gripper"]["r"])

            dist_to_reachy = hl.Dist(reachy_position, hl.GetT(hl.GetCameraMat4()))

        # Update of the Reachy arm controls with VR controllers
        if hl.gVal.activate_VR:
            # gui_debug_controller_input(controllers)
            connected_controller = update_connected_controller(controllers)
            for controller_name in connected_controller:
                controller = controllers[controller_name]

                if flag_set_left_controller:
                    if controller["input"].Down(hl.VRCB_Axis1) or flag_set_default_left_controller:
                        left_controller_name = controller_name
                        flag_set_left_controller = False
                        flag_set_default_left_controller = False

                elif flag_set_right_controller or flag_set_default_right_controller:
                    if controller["input"].Down(hl.VRCB_Axis1):
                        right_controller_name = controller_name
                        flag_set_right_controller = False
                        flag_set_default_right_controller = False

                if controller_swap_side:
                    old_right = right_controller_name
                    right_controller_name = left_controller_name
                    left_controller_name = old_right
                    controller_swap_side = False

                if left_controller_name == controller_name:
                    vec_pos_target_left = hl.GetT(controller["world"])

                    controller_left_node.GetTransform().SetWorld(controller["world"])

                    joint_node_gripper = joint_node_offset["l_gripper"]
                    joint_node_gripper["r"] = -0.5 + (controller["input"].Surface(1).x * abs(-0.5 - 0.290))

                    joint_node_forearm_yaw = joint_node_offset["l_forearm_yaw"]
                    z_rot = hl.GetR(controller["world"]).z
                    joint_node_forearm_yaw["r"] = z_rot

                    vec_joystick = controller["input"].Surface(0)
                    joint_node_offset["l_wrist_pitch"]["r"] = clamp(-vec_joystick.x, -0.5, 0.6)
                    joint_node_offset["l_wrist_roll"]["r"] = clamp(vec_joystick.y, -0.5, 0.5)

                elif right_controller_name == controller_name:
                    vec_pos_target_right = hl.GetT(controller["world"])

                    controller_right_node.GetTransform().SetWorld(controller["world"])

                    joint_node_gripper = joint_node_offset["r_gripper"]
                    joint_node_gripper["r"] = -0.290 + ((1 - controller["input"].Surface(1).x) * abs(-0.290 - 0.5))

                    joint_node_forearm_yaw = joint_node_offset["r_forearm_yaw"]
                    z_rot = hl.GetR(controller["world"]).z
                    joint_node_forearm_yaw["r"] = z_rot

                    vec_joystick = controller["input"].Surface(0)
                    joint_node_offset["r_wrist_pitch"]["r"] = clamp(vec_joystick.x, -0.5, 0.6)
                    joint_node_offset["r_wrist_roll"]["r"] = clamp(-vec_joystick.y, -0.5, 0.5)

            # Places the VR body on the Reachy
            if right_controller_name in controllers and left_controller_name in controllers and controllers[right_controller_name]["input"].Down(hl.VRCB_Axis1) and controllers[left_controller_name]["input"].Down(hl.VRCB_Axis1):
                flag_activate_calibration = True

            dist_to_reachy = hl.Dist(reachy_position, hl.GetT(hl.gVal.vr_state.head))

        # Places the VR body on the Reachy
        if hl.gVal.keyboard.Down(hl.K_Space):
            flag_activate_calibration = True

        if flag_activate_calibration:
            dt = hl.TickClock()
            dts = hl.time_to_sec_f(dt)
            f = calibration_timer_cptr / calibration_timer
            calibration_gauge_node.GetTransform().SetScale(hl.Vec3(f * calibration_gauge_max_scale, 1, 1))
            calibration_timer_cptr += dts
            if calibration_timer_cptr >= calibration_timer:
                if hl.gVal.activate_VR:
                    calibrate_VR_head()
                else:
                    calibrate_camera()
                calibration_timer_cptr = 0

        else:
            calibration_gauge_node.GetTransform().SetScale(hl.Vec3(0, 1, 1))
            calibration_timer_cptr = 0

        if dist_to_reachy < 0.25 and not flag_reachy_head_hidden:
            hide_reachy_head()
            flag_reachy_head_hidden = True

        elif dist_to_reachy > 0.30 and flag_reachy_head_hidden:
            show_reachy_head()
            flag_reachy_head_hidden = False


        # Updates the Reachy's joint rotation with information from the ik, taking into account the adjustments made to the model (joint_node_offset dict)
        joint_states = physics.getJointStates(reachy.reachyId, jointIndices=list(range(reachy.numJoints)))
            
        for j in range(reachy.numJoints):
            joint_info = physics.getJointInfo(reachy.reachyId, j)
            joint_name = joint_info[1].decode()
            if joint_name in harfang_node:
                rot = joint_states[j][0]
            
                if keyboard_override and joint_name in ["r_forearm_yaw", "r_gripper", "r_wrist_pitch", "r_wrist_roll", "l_forearm_yaw", "l_gripper", "l_wrist_pitch", "l_wrist_roll"]:
                    harfang_node[joint_name].GetTransform().SetRot(hl.Vec3(0, joint_node_offset[joint_name]["r"], 0))
                else:
                    if joint_node_offset[joint_name]["inverse_angle"]:
                        harfang_node[joint_name].GetTransform().SetRot(hl.Vec3(0, -rot + joint_node_offset[joint_name]["r"], 0))
                    else:
                        harfang_node[joint_name].GetTransform().SetRot(hl.Vec3(0, rot + joint_node_offset[joint_name]["r"], 0))
       
        # Compute IK according to the defined targets (using the GUI panel, the keyboard or the VR controllers)
        target_right = [vec_pos_target_right.x, vec_pos_target_right.z, vec_pos_target_right.y]
        target_left = [vec_pos_target_left.x, vec_pos_target_left.z, vec_pos_target_left.y]

        jointPoses = physics.calculateInverseKinematics2(reachy.reachyId, [reachy.right_arm_end_effector_index, reachy.left_arm_end_effector_index], [target_right, target_left])

        for i in range(reachy.numJoints):
            jointInfo = physics.getJointInfo(reachy.reachyId, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                # Resets the joints state (ignoring all dynamics, not recommended to use during simulation)
                physics.setJointMotorControl2(bodyIndex=reachy.reachyId, jointIndex=i, controlMode=physics.POSITION_CONTROL,
                                        targetPosition=jointPoses[qIndex-7])

        # Draw debug targets
        hl.DrawCrossV(vec_pos_target_right, hl.Color.Purple, size=0.05)
        hl.DrawCrossV(vec_pos_target_left, hl.Color.Green, size=0.05)

    hl.Uninit()
    return 0

if __name__ == "__main__":
   main(sys.argv[1:])
