#include <iostream>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

// ---------------------- KEYBOARD FUNCTION ---------------------------
int get_key() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return (ch != EOF) ? ch : -1;
}
// --------------------------------------------------------------------

class CartesianImpedanceTeleop : public rclcpp::Node {
public:
    CartesianImpedanceTeleop()
    : Node("teleop_stiffness")
    {
        this->declare_parameter<std::string>("controller_name",
                                             "cartesian_impedance_controller");
        controller_name_ = this->get_parameter("controller_name").as_string();

        cmd_topic_ = "/" + controller_name_ + "/stiffness_command";
        cur_topic_ = "/" + controller_name_ + "/current_stiffness";

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(cmd_topic_, 10);

        sub_stiff_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            cur_topic_, 10,
            std::bind(&CartesianImpedanceTeleop::cb_stiffness, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CartesianImpedanceTeleop::update_loop, this)
        );

        print_menu();
    }

private:
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_stiff_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string controller_name_;
    std::string cmd_topic_;
    std::string cur_topic_;

    // stiffness vector [fx fy fz tx ty tz]
    std::array<double, 6> K_ = {500, 500, 500, 50, 50, 50};

    // key mapping
    const double delta_T = 20.0;
    const double delta_R = 5.0;

    void cb_stiffness(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        K_[0] = msg->wrench.force.x;
        K_[1] = msg->wrench.force.y;
        K_[2] = msg->wrench.force.z;
        K_[3] = msg->wrench.torque.x;
        K_[4] = msg->wrench.torque.y;
        K_[5] = msg->wrench.torque.z;
        print_menu();
    }

    void update_loop() {
        int key = get_key();
        if (key < 0)
            return;

        char k = static_cast<char>(key);

        if (k == 'q') {
            RCLCPP_INFO(this->get_logger(), "Exiting teleop");
            rclcpp::shutdown();
            return;
        }

        bool changed = false;

        // TRANSLATION
        if (k == 'a') { K_[0] += delta_T; changed = true; }
        if (k == 's') { K_[0] -= delta_T; changed = true; }

        if (k == 'd') { K_[1] += delta_T; changed = true; }
        if (k == 'f') { K_[1] -= delta_T; changed = true; }

        if (k == 'w') { K_[2] += delta_T; changed = true; }
        if (k == 'e') { K_[2] -= delta_T; changed = true; }

        // ROTATION Z
        if (k == 'o') { K_[5] += delta_R; changed = true; }
        if (k == 'i') { K_[5] -= delta_R; changed = true; }

        if (changed) {
            publish_stiffness();
            print_menu();
        }
    }

    void publish_stiffness() {
        geometry_msgs::msg::WrenchStamped msg;
        msg.wrench.force.x  = K_[0];
        msg.wrench.force.y  = K_[1];
        msg.wrench.force.z  = K_[2];
        msg.wrench.torque.x = K_[3];
        msg.wrench.torque.y = K_[4];
        msg.wrench.torque.z = K_[5];

        pub_cmd_->publish(msg);
    }

    void print_menu() {
        std::cout << "\033[2J\033[1;1H";
        std::cout << "--------------------------------------\n";
        std::cout << "  Cartesian Impedance Teleop\n";
        std::cout << "--------------------------------------\n";
        std::cout << "Publishing to: " << cmd_topic_ << "\n\n";
        std::cout << "Current Stiffness Values:\n";
        std::cout << "  Tx Ty Tz : " << K_[0] << "  " << K_[1] << "  " << K_[2] << "\n";
        std::cout << "  Rx Ry Rz : " << K_[3] << "  " << K_[4] << "  " << K_[5] << "\n\n";

        std::cout << "Keys:\n";
        std::cout << "  a/s : +X / -X stiffness\n";
        std::cout << "  d/f : +Y / -Y stiffness\n";
        std::cout << "  w/e : +Z / -Z stiffness\n";
        std::cout << "  o/i : +Rz / -Rz stiffness\n";
        std::cout << "  q   : Quit\n";
        std::cout << "--------------------------------------\n";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianImpedanceTeleop>());
    return 0;
}
