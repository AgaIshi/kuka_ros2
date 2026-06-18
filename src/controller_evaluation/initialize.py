ROBOT_TYPE = "kuka"  # "franka" or "kuka" or "z1"

# For the KUKA LBR: in Gazebo the controller listens on the namespaced topic
# /lbr/cartesian_impedance_controller/target_frame, whereas hardware.launch.py
# remaps it to /lbr/target_frame. Set SIM accordingly.
SIM = True


def get_robot_params():
    topic_name = "/target_frame"
    if ROBOT_TYPE == "franka":
        base = "base"
        end_effector = "fr3_hand_tcp"
        prefix = "/cartesian_impedance_controller"
    elif ROBOT_TYPE == "kuka":
        base = "lbr_link_0"
        end_effector = "lbr_link_ee"
        if SIM:
            # namespaced controller topic used in Gazebo
            return "/lbr/cartesian_impedance_controller/target_frame", base, end_effector
        # hardware.launch.py remaps the target to /lbr/target_frame
        return "/lbr/target_frame", base, end_effector
    elif ROBOT_TYPE == "z1":
        base = "link00"
        end_effector = "link06"
        prefix = "/cartesian_impedance_controller"
    else:
        print("Robot type unknown")
        exit(1)
    return prefix + topic_name, base, end_effector
