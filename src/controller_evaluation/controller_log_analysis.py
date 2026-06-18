import h5py
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def load_mat_file(filename):
    data = {}
    with h5py.File(filename, 'r') as f:
        for key in f.keys():
            dataset = f[key]
            if isinstance(dataset, h5py.Dataset):
                print("Extracting", key, " data: ", f[key].shape)
                data[key] = np.array(dataset, dtype='float64')
            else:
                print(f"Skipping {key}, unexpected data format.")
    return data

def wrap_angle(angle):
    # wrap between -pi/2 and pi/2
    while np.any(angle > np.pi/2):
        angle[angle > np.pi/2] -= np.pi
    while np.any(angle < -np.pi/2):
        angle[angle < -np.pi/2] += np.pi
    return angle

    
def convert_orientation_to_euler(matrix_data):
    euler_angles = np.array([R.from_matrix(m.reshape(3, 3)).as_euler('xyz', degrees=False) for m in matrix_data])
    euler_angles[:, 0] = wrap_angle(euler_angles[:, 0])
    euler_angles[:, 1] = wrap_angle(euler_angles[:, 1])
    euler_angles[:, 2] = wrap_angle(euler_angles[:, 2])
    return euler_angles.T  # Transpose to separate roll, pitch, yaw

def set_ylim(ax, values1, values2):
    min_val = min(np.min(values1), np.min(values2))
    max_val = max(np.max(values1), np.max(values2))
    # min_val = values1[0] - 0.12
    # max_val = values1[0] + 0.12
    padding = (max_val - min_val) * 0.1  # Add 10% padding
    ax.set_ylim(min_val - padding, max_val + padding)

def plot_data(data):
    time = data['time_sec'].flatten() + data['time_nsec'].flatten() * 1e-9
    euler_target = convert_orientation_to_euler(data['cart_target_M'])
    euler_measured = convert_orientation_to_euler(data['cart_measured_M'])
    
    # Plot Cartesian XYZ and RPY angles in one window (two columns)
    fig, axs = plt.subplots(3, 2, figsize=(12, 12))
    
    
    axs[0, 0].plot(time, data['cart_target_x'], 'r--', label="Target X")
    axs[0, 0].plot(time, data['cart_measured_x'], 'r', label="Measured X")
    axs[0, 0].set_title("Cartesian X")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].legend()
    set_ylim(axs[0, 0], data['cart_target_x'], data['cart_measured_x'])

    axs[1, 0].plot(time, data['cart_target_y'], 'g--', label="Target Y")
    axs[1, 0].plot(time, data['cart_measured_y'], 'g', label="Measured Y")
    axs[1, 0].set_title("Cartesian Y")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].legend()
    set_ylim(axs[1, 0], data['cart_target_y'], data['cart_measured_y'])

    axs[2, 0].plot(time, data['cart_target_z'], 'b--', label="Target Z")
    axs[2, 0].plot(time, data['cart_measured_z'], 'b', label="Measured Z")
    axs[2, 0].set_title("Cartesian Z")
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].legend()
    set_ylim(axs[2, 0], data['cart_target_z'], data['cart_measured_z'])

    axs[0, 1].plot(time, euler_target[0], 'r--', label="Target Roll")
    axs[0, 1].plot(time, euler_measured[0], 'r', label="Measured Roll")
    axs[0, 1].set_title("Roll")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].legend()
    axs[0, 1].set_ylim(-np.pi, np.pi)

    axs[1, 1].plot(time, euler_target[1], 'g--', label="Target Pitch")
    axs[1, 1].plot(time, euler_measured[1], 'g', label="Measured Pitch")
    axs[1, 1].set_title("Pitch")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].legend()
    axs[1, 1].set_ylim(-np.pi, np.pi)

    axs[2, 1].plot(time, euler_target[2], 'b--', label="Target Yaw")
    axs[2, 1].plot(time, euler_measured[2], 'b', label="Measured Yaw")
    axs[2, 1].set_title("Yaw")
    axs[2, 1].set_xlabel("Time (s)")
    axs[2, 1].legend()
    axs[2, 1].set_ylim(-np.pi, np.pi)

    fig.tight_layout()
    
    # Plot joint position and measured in another window (two columns)
    fig2, axs2 = plt.subplots(4, 2, figsize=(12, 16))
    for i in range(7):
        row, col = divmod(i, 2)
        # axs2[row, col].plot(time, data['joint_target_IK_kdl'][:, i], 'b--', label=f"Joint Target (IK) KDL {i+1}")
        axs2[row, col].plot(time, data['joint_target_clik'][:, i], 'g--', label=f"Joint Target (IK) CLIK {i+1}")
        axs2[row, col].plot(time, data['joint_measured'][:, i], 'b', label=f"Joint Measured {i+1}")
        axs2[row, col].set_title(f"Joint {i+1} Position")
        axs2[row, col].set_xlabel("Time (s)")
        axs2[row, col].legend()
        # print("Mean abs error between IK and filtered: ", np.mean(np.abs(data['joint_target_IK'][:, i] - data['joint_target_filtered'][:, i])))

    fig2.tight_layout()

    # Plot commanded and measured torques for each joint in another window (two columns)
    fig3, axs3 = plt.subplots(4, 2, figsize=(12, 16))
    for i in range(7):
        row, col = divmod(i, 2)
        axs3[row, col].plot(time, data['commanded_torque'][:, i], 'y--', label=f"Commanded Torque {i+1}")
        axs3[row, col].plot(time, data['measured_torque'][:, i], 'y', label=f"Measured Torque {i+1}")
        axs3[row, col].set_title(f"Joint {i+1} Torques")
        axs3[row, col].set_xlabel("Time (s)")
        axs3[row, col].legend()

    fig3.tight_layout()

    # Plot Jacobian condition number over time 
    # catch key error
    if 'condition_number_jac' in data:
        fig4, ax4 = plt.subplots()
        ax4.plot(time, data['condition_number_jac'], 'm', label="Jacobian Condition Number")
        ax4.set_title("Jacobian Condition Number")
        ax4.set_xlabel("Time (s)")
        ax4.legend()

    if 'ns_task' in data:
        fig3, axs3 = plt.subplots(4, 2, figsize=(12, 16))
        it_range = np.arange(0, data['ns_task'].shape[0], 1)
        for i in range(7):
            row, col = divmod(i, 2)
            axs3[row, col].plot(it_range, data['ns_task'][:, i], 'y--', label=f"NS Task {i+1}")
            axs3[row, col].set_title(f"Joint {i+1} NS Task")
            axs3[row, col].set_xlabel("Iterations")
            axs3[row, col].legend()
        # set same scale for all axes
        for ax in axs3.flat:
            min_val = np.min(data['ns_task'])
            max_val = np.max(data['ns_task'])
            padding = (max_val - min_val) * 0.1
            ax.set_ylim(min_val - padding, max_val + padding)
        fig3.tight_layout()
    # Show all figures
    plt.show()


if __name__ == "__main__":
    filename = "cart_impedance.mat"  # Change this to your actual file name
    # filename = "/home/nardi/test_butter/alpha_080.mat"
    data = load_mat_file(filename)
    plot_data(data)

