#include "rclcpp/rclcpp.hpp"
#include "nereo_interfaces/msg/command_velocity.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include <cmath>
#include <array>
#include <vector>
#include <Eigen/Geometry>

// Define ControlSystem class for CS controller
class ControlSystem
{
    public:
        // Default constructor (needed for std::array)
        ControlSystem()
            : minForce(0.0f), maxForce(0.0f), maxErrorIntegral(0.0f), minErrorIntegral(0.0f), Ki(0.0f), ErrorIntegral(0.0f)
        {
            Kx[0] = 0.0f;
            Kx[1] = 0.0f;
        }

        //Constructor
        ControlSystem(float minForce, float maxForce, float Kx[2], float Ki);

        //Returns the output of the controller given a reference signal and the actual measured state of the system
        float calculateU(float reference, float y_measurement, float * x_measurement);

    public:

        float minForce;  //for control input saturation purposes
        float maxForce;  //for control input saturation purposes
        float maxErrorIntegral; //for integrator anti-windup
        float minErrorIntegral; //for integrator anti-windup

        // Variable for control laws's coefficients
        float Kx[2];
        float Ki;

        //Controller memory
        float ErrorIntegral;
};

// Define PID controller struct to match arm_pid_instance_f32
struct PidController {
    float A0;          // The derived gain, A0 = Kp + Ki + Kd
    float A1;          // The derived gain, A1 = -Kp - 2Kd
    float A2;          // The derived gain, A2 = Kd
    std::array<float, 3> state = {0.0f, 0.0f, 0.0f}; // state[0] = x[n-1], state[1] = x[n-2], state[2] = y[n-1]
    float Kp;          // The proportional gain
    float Ki;          // The integral gain
    float Kd;          // The derivative gain
    
    // Compute function that mimics arm_pid_f32
    float compute(float error) {
        // y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
        float out = (A0 * error) + (A1 * state[0]) + (A2 * state[1]) + state[2];
        
        // Update state
        state[1] = state[0];    // x[n-2] = x[n-1]
        state[0] = error;       // x[n-1] = x[n]
        state[2] = out;         // y[n-1] = y[n]
        
        return out;
    }
    
    // Update coefficients when PID parameters change
    void updateCoefficients() {
        A0 = Kp + Ki + Kd;
        A1 = -Kp - 2 * Kd;
        A2 = Kd;
    }
    
    void reset() {
        state = {0.0f, 0.0f, 0.0f};
    }
};

// Constants
constexpr float TOLERANCE = 0.05f;
constexpr int PID_NUMBER = 4;

class NereoControllerNode : public rclcpp::Node {
public:
    enum class ControlMode {
        DIRECT_PASSTHROUGH,
        PID_CONTROL,
        PID_ANTI_WINDUP,
        CS_CONTROLLER
    };

    NereoControllerNode() : Node("nereo_controller_node") {
        // Initialize parameters
        this->declare_parameter("control_mode", 0);
        this->declare_parameter("kp", std::vector<double>{0.1, 0.1, 0.1, 0.1});
        this->declare_parameter("ki", std::vector<double>{0.01, 0.01, 0.01, 0.01});
        this->declare_parameter("kd", std::vector<double>{0.05, 0.05, 0.05, 0.05});
        
        // CS Controller parameters
        this->declare_parameter("cs_kx0", std::vector<double>{198.0952, 468.0585});
        this->declare_parameter("cs_kx1", std::vector<double>{3.8191, 6.0003});
        this->declare_parameter("cs_kx2", std::vector<double>{5.2577, 11.8206});
        this->declare_parameter("cs_ki0", -359.2481);
        this->declare_parameter("cs_ki1", 0.0);
        this->declare_parameter("cs_ki2", -16.7389);
        this->declare_parameter("cs_heave_min", -60.0);
        this->declare_parameter("cs_heave_max", 80.0);
        this->declare_parameter("cs_angle_min", -30.0);
        this->declare_parameter("cs_angle_max", 30.0);
        
        // Initialize publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<nereo_interfaces::msg::CommandVelocity>(
            "/nereo_cmd_vel", 10);
            
        cmd_vel_sub_ = this->create_subscription<nereo_interfaces::msg::CommandVelocity>(
            "/nereo_cmd_vel_no_fb", 10,
            std::bind(&NereoControllerNode::cmdVelCallback, this, std::placeholders::_1));
            
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", 10,
            std::bind(&NereoControllerNode::imuCallback, this, std::placeholders::_1));
            
        pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "/barometer_pressure", 10,
            std::bind(&NereoControllerNode::pressureCallback, this, std::placeholders::_1));
            
        // Timer for parameter checking
        param_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NereoControllerNode::checkParameters, this));
            
        // Initialize PIDs
        initPids();
        
        // Initialize CS controller
        initControllers();
        
        // Initialize other variables
        first_update_ = true;
        last_cmd_vel_neq_0_ = {1, 1, 1, 1};
        
        RCLCPP_INFO(this->get_logger(), "Nereo controller node initialized");
    }

private:
    // Publishers and subscribers
    rclcpp::Publisher<nereo_interfaces::msg::CommandVelocity>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nereo_interfaces::msg::CommandVelocity>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
    rclcpp::TimerBase::SharedPtr param_timer_;
    
    // Control mode and PID controllers
    ControlMode control_mode_ = ControlMode::DIRECT_PASSTHROUGH;
    std::array<PidController, PID_NUMBER> pids_;
    
    // CS controller parameters
    std::array<ControlSystem, 3> controllers_;
    std::array<float, 2> Kx0_ = {198.0952f, 468.0585f};
    std::array<float, 2> Kx1_ = {3.8191f, 6.0003f};
    std::array<float, 2> Kx2_ = {5.2577f, 11.8206f};
    float Ki0_ = -359.2481f;
    float Ki1_ = 0.0f;
    float Ki2_ = -16.7389f;
    
    // Setpoints and state
    std::array<float, 4> setpoints_ = {0.0f};
    std::array<uint8_t, 4> last_cmd_vel_neq_0_ = {1};
    bool first_update_ = true;
    
    // Latest sensor data
    Eigen::Quaternionf current_orientation_ = Eigen::Quaternionf::Identity();
    float current_pressure_ = 0.0f;
    bool has_orientation_ = false;
    bool has_pressure_ = false;
    
    void checkParameters() {
        // Check if control mode parameter has changed
        int mode = this->get_parameter("control_mode").as_int();
        ControlMode new_mode = static_cast<ControlMode>(mode);
        
        if (new_mode != control_mode_) {
            control_mode_ = new_mode;
            RCLCPP_INFO(this->get_logger(), "Control mode changed to: %d", mode);
        }
        
        // Check if PID parameters have changed
        auto kp_values = this->get_parameter("kp").as_double_array();
        auto ki_values = this->get_parameter("ki").as_double_array();
        auto kd_values = this->get_parameter("kd").as_double_array();
        
        if (kp_values.size() == PID_NUMBER && ki_values.size() == PID_NUMBER && kd_values.size() == PID_NUMBER) {
            bool params_changed = false;
            
            for (size_t i = 0; i < PID_NUMBER; i++) {
                if (pids_[i].Kp != static_cast<float>(kp_values[i]) ||
                    pids_[i].Ki != static_cast<float>(ki_values[i]) ||
                    pids_[i].Kd != static_cast<float>(kd_values[i])) {
                    params_changed = true;
                    break;
                }
            }
            
            if (params_changed) {
                for (size_t i = 0; i < PID_NUMBER; i++) {
                    pids_[i].Kp = static_cast<float>(kp_values[i]);
                    pids_[i].Ki = static_cast<float>(ki_values[i]);
                    pids_[i].Kd = static_cast<float>(kd_values[i]);
                    pids_[i].updateCoefficients();
                    pids_[i].reset();
                }
                RCLCPP_INFO(this->get_logger(), "PID parameters updated");
            }
        }

        // Check if CS controller parameters have changed
        bool cs_params_changed = false;
        
        // Check KX0 parameters (heave)
        auto cs_kx0_values = this->get_parameter("cs_kx0").as_double_array();
        if (cs_kx0_values.size() == 2 && 
            (Kx0_[0] != static_cast<float>(cs_kx0_values[0]) || 
             Kx0_[1] != static_cast<float>(cs_kx0_values[1]))) {
            Kx0_[0] = static_cast<float>(cs_kx0_values[0]);
            Kx0_[1] = static_cast<float>(cs_kx0_values[1]);
            cs_params_changed = true;
        }
        
        // Check KX1 parameters (roll)
        auto cs_kx1_values = this->get_parameter("cs_kx1").as_double_array();
        if (cs_kx1_values.size() == 2 && 
            (Kx1_[0] != static_cast<float>(cs_kx1_values[0]) || 
             Kx1_[1] != static_cast<float>(cs_kx1_values[1]))) {
            Kx1_[0] = static_cast<float>(cs_kx1_values[0]);
            Kx1_[1] = static_cast<float>(cs_kx1_values[1]);
            cs_params_changed = true;
        }
        
        // Check KX2 parameters (pitch)
        auto cs_kx2_values = this->get_parameter("cs_kx2").as_double_array();
        if (cs_kx2_values.size() == 2 && 
            (Kx2_[0] != static_cast<float>(cs_kx2_values[0]) || 
             Kx2_[1] != static_cast<float>(cs_kx2_values[1]))) {
            Kx2_[0] = static_cast<float>(cs_kx2_values[0]);
            Kx2_[1] = static_cast<float>(cs_kx2_values[1]);
            cs_params_changed = true;
        }
        
        // Check Ki values
        double new_ki0 = this->get_parameter("cs_ki0").as_double();
        if (Ki0_ != static_cast<float>(new_ki0)) {
            Ki0_ = static_cast<float>(new_ki0);
            cs_params_changed = true;
        }
        
        double new_ki1 = this->get_parameter("cs_ki1").as_double();
        if (Ki1_ != static_cast<float>(new_ki1)) {
            Ki1_ = static_cast<float>(new_ki1);
            cs_params_changed = true;
        }
        
        double new_ki2 = this->get_parameter("cs_ki2").as_double();
        if (Ki2_ != static_cast<float>(new_ki2)) {
            Ki2_ = static_cast<float>(new_ki2);
            cs_params_changed = true;
        }
        
        // Check min/max force values
        double new_heave_min = this->get_parameter("cs_heave_min").as_double();
        double new_heave_max = this->get_parameter("cs_heave_max").as_double();
        double new_angle_min = this->get_parameter("cs_angle_min").as_double();
        double new_angle_max = this->get_parameter("cs_angle_max").as_double();
        
        // Update controllers if parameters have changed
        if (cs_params_changed) {
            controllers_[0] = ControlSystem(
                static_cast<float>(new_heave_min),
                static_cast<float>(new_heave_max),
                Kx0_.data(), Ki0_); // heave
                
            controllers_[1] = ControlSystem(
                static_cast<float>(new_angle_min),
                static_cast<float>(new_angle_max),
                Kx1_.data(), Ki1_); // roll
                
            controllers_[2] = ControlSystem(
                static_cast<float>(new_angle_min),
                static_cast<float>(new_angle_max),
                Kx2_.data(), Ki2_); // pitch
                
            RCLCPP_INFO(this->get_logger(), "CS Controller parameters updated");
        }
    }
    
    void initPids() {
        auto kp_values = this->get_parameter("kp").as_double_array();
        auto ki_values = this->get_parameter("ki").as_double_array();
        auto kd_values = this->get_parameter("kd").as_double_array();
        
        if (kp_values.size() == PID_NUMBER && ki_values.size() == PID_NUMBER && kd_values.size() == PID_NUMBER) {
            for (size_t i = 0; i < PID_NUMBER; i++) {
                pids_[i].Kp = static_cast<float>(kp_values[i]);
                pids_[i].Ki = static_cast<float>(ki_values[i]);
                pids_[i].Kd = static_cast<float>(kd_values[i]);
                pids_[i].updateCoefficients();
                pids_[i].reset();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid PID parameters, using defaults");
            for (size_t i = 0; i < PID_NUMBER; i++) {
                pids_[i].Kp = 0.1f;
                pids_[i].Ki = 0.01f;
                pids_[i].Kd = 0.05f;
                pids_[i].updateCoefficients();
                pids_[i].reset();
            }
        }
    }
    
    void initControllers() {
        // Get initial values from parameters
        auto cs_kx0_values = this->get_parameter("cs_kx0").as_double_array();
        auto cs_kx1_values = this->get_parameter("cs_kx1").as_double_array();
        auto cs_kx2_values = this->get_parameter("cs_kx2").as_double_array();
        
        if (cs_kx0_values.size() == 2) {
            Kx0_[0] = static_cast<float>(cs_kx0_values[0]);
            Kx0_[1] = static_cast<float>(cs_kx0_values[1]);
        }
        
        if (cs_kx1_values.size() == 2) {
            Kx1_[0] = static_cast<float>(cs_kx1_values[0]);
            Kx1_[1] = static_cast<float>(cs_kx1_values[1]);
        }
        
        if (cs_kx2_values.size() == 2) {
            Kx2_[0] = static_cast<float>(cs_kx2_values[0]);
            Kx2_[1] = static_cast<float>(cs_kx2_values[1]);
        }
        
        Ki0_ = static_cast<float>(this->get_parameter("cs_ki0").as_double());
        Ki1_ = static_cast<float>(this->get_parameter("cs_ki1").as_double());
        Ki2_ = static_cast<float>(this->get_parameter("cs_ki2").as_double());
        
        float heave_min = static_cast<float>(this->get_parameter("cs_heave_min").as_double());
        float heave_max = static_cast<float>(this->get_parameter("cs_heave_max").as_double());
        float angle_min = static_cast<float>(this->get_parameter("cs_angle_min").as_double());
        float angle_max = static_cast<float>(this->get_parameter("cs_angle_max").as_double());
        
        controllers_[0] = ControlSystem(heave_min, heave_max, Kx0_.data(), Ki0_); // heave
        controllers_[1] = ControlSystem(angle_min, angle_max, Kx1_.data(), Ki1_); // roll
        controllers_[2] = ControlSystem(angle_min, angle_max, Kx2_.data(), Ki2_); // pitch
        
        RCLCPP_INFO(this->get_logger(), "CS controllers initialized with parameters from ROS");
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    current_orientation_.w() = msg->orientation.w;
    current_orientation_.x() = msg->orientation.x;
    current_orientation_.y() = msg->orientation.y;
    current_orientation_.z() = msg->orientation.z;
    has_orientation_ = true;
    RCLCPP_DEBUG(this->get_logger(), "IMU data received: w=%.2f, x=%.2f, y=%.2f, z=%.2f",
        msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    }

    void pressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
        current_pressure_ = msg->fluid_pressure;
        has_pressure_ = true;
        RCLCPP_DEBUG(this->get_logger(), "Pressure data received: %.2f Pa", msg->fluid_pressure);
    }
    
    void cmdVelCallback(const nereo_interfaces::msg::CommandVelocity::SharedPtr msg) {
        if (!has_orientation_ || !has_pressure_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Missing sensor data (orientation: %s, pressure: %s)", 
                has_orientation_ ? "true" : "false", 
                has_pressure_ ? "true" : "false");
            
            // Pass through the command velocity without modification
            cmd_vel_pub_->publish(*msg);
            return;
        }
        
        // Convert command velocity to array
        std::array<float, 6> cmd_vel = {
            msg->cmd_vel[0],  // surge
            msg->cmd_vel[1],  // sway
            msg->cmd_vel[2],  // heave
            msg->cmd_vel[3],  // roll
            msg->cmd_vel[4],  // pitch
            msg->cmd_vel[5]   // yaw
        };
        
        // Apply feedback based on control mode
        std::array<float, 6> output_cmd_vel;
        
        switch (control_mode_) {
            case ControlMode::DIRECT_PASSTHROUGH:
                output_cmd_vel = cmd_vel;
                break;
                
            case ControlMode::PID_CONTROL:
                calculateFeedbackWithPid(cmd_vel, output_cmd_vel);
                break;
                
            case ControlMode::PID_ANTI_WINDUP:
                calculateFeedbackWithPidAntiWindup(cmd_vel, output_cmd_vel);
                break;
                
            case ControlMode::CS_CONTROLLER:
                calculateFeedbackWithCsController(cmd_vel, output_cmd_vel);
                break;
        }
        
        // Publish the output command velocity
        auto output_msg = nereo_interfaces::msg::CommandVelocity();
        output_msg.cmd_vel[0] = output_cmd_vel[0];  // surge
        output_msg.cmd_vel[1] = output_cmd_vel[1];  // sway
        output_msg.cmd_vel[2] = output_cmd_vel[2];  // heave
        output_msg.cmd_vel[3] = output_cmd_vel[3];  // roll
        output_msg.cmd_vel[4] = output_cmd_vel[4];  // pitch
        output_msg.cmd_vel[5] = output_cmd_vel[5];  // yaw
        
        cmd_vel_pub_->publish(output_msg);
    }
    
    void calculateRpyFromQuaternion(const Eigen::Quaternionf& quaternion, std::array<float, 3>& rpy) {
        // Extract Euler angles from quaternion (roll, pitch, yaw)
        Eigen::Vector3f euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
        rpy[0] = euler.x(); // roll
        rpy[1] = euler.y(); // pitch
        rpy[2] = euler.z(); // yaw
    }
    
    uint8_t updateSetpoints(const std::array<float, 6>& cmd_vel) {
        uint8_t count = 0;
        std::array<float, 3> rpy_rads;
        calculateRpyFromQuaternion(current_orientation_, rpy_rads);
        
        // Updates setpoints for angles
        for (uint8_t i = 0; i < 3; i++) {
            if (std::abs(cmd_vel[i+3]) < TOLERANCE) {
                if (last_cmd_vel_neq_0_[i+1]) {
                    setpoints_[i+1] = rpy_rads[i];
                    count++;
                }
                last_cmd_vel_neq_0_[i+1] = 0;
            } else {
                last_cmd_vel_neq_0_[i+1] = 1;
            }
        }
        
        // Updates depth setpoint
        Eigen::Vector3f z_out(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f z_out_RBF = current_orientation_.inverse() * z_out;
        
        bool x_condition = std::abs(z_out_RBF.x()) < TOLERANCE || std::abs(cmd_vel[0]) < TOLERANCE;
        bool y_condition = std::abs(z_out_RBF.y()) < TOLERANCE || std::abs(cmd_vel[1]) < TOLERANCE;
        bool z_condition = std::abs(z_out_RBF.z()) < TOLERANCE || std::abs(cmd_vel[2]) < TOLERANCE;
        
        if (x_condition && y_condition && z_condition) {
            if (last_cmd_vel_neq_0_[0]) {
                setpoints_[0] = current_pressure_;
                count++;
            }
        }

        if (count > 0 || first_update_) {
            // Log updated setpoint values
            RCLCPP_INFO(this->get_logger(), "Updated setpoints: Depth: %.2f, Roll: %.2f, Pitch: %.2f, Yaw: %.2f", 
                    setpoints_[0], setpoints_[1], setpoints_[2], setpoints_[3]);
        }

        // Check for Z component in EFBF
        Eigen::Vector3f cmd_vel_vec(cmd_vel[0], cmd_vel[1], cmd_vel[2]);
        Eigen::Vector3f cmd_vel_EFBF = current_orientation_ * cmd_vel_vec;
        
        if (std::abs(cmd_vel_EFBF.z()) < TOLERANCE) {
            last_cmd_vel_neq_0_[0] = 0;
        } else {
            last_cmd_vel_neq_0_[0] = 1;
        }
        
        if (first_update_) {
            setpoints_[0] = current_pressure_;
            setpoints_[1] = rpy_rads[0];
            setpoints_[2] = rpy_rads[1];
            setpoints_[3] = rpy_rads[2];
            first_update_ = false;
        }
        
        return count;
    }
    
    float clamp(float value, float max, float min) {
        if (value > max) return max;
        if (value < min) return min;
        return value;
    }
    
    void calculateFeedbackWithPid(const std::array<float, 6>& cmd_vel, std::array<float, 6>& output_cmd_vel) {
        // Calculate current values (z, roll, pitch, yaw)
        std::array<float, 4> current_values;
        std::array<float, 3> rpy_rads;
        calculateRpyFromQuaternion(current_orientation_, rpy_rads);
        
        current_values[0] = current_pressure_;
        current_values[1] = rpy_rads[0];  // roll
        current_values[2] = rpy_rads[1];  // pitch
        current_values[3] = rpy_rads[2];  // yaw
        
        updateSetpoints(cmd_vel);
        
        // Copy input to output
        output_cmd_vel = cmd_vel;
        
        // Calculate PID outputs
        float roll_pid_feedback = pids_[1].compute(setpoints_[1] - current_values[1]);
        float pitch_pid_feedback = pids_[2].compute(setpoints_[2] - current_values[2]);
        float yaw_pid_feedback = pids_[3].compute(setpoints_[3] - current_values[3]);
        
        // Depth control
        float z_pid_output = pids_[0].compute(setpoints_[0] - current_values[0]);
        
        // Convert z_pid_output to body frame
        Eigen::Vector3f z_out(0.0f, 0.0f, z_pid_output);
        Eigen::Vector3f z_out_RBF = current_orientation_.inverse() * z_out;
        
        // Apply feedback on x, y, z axes if conditions are met
        bool x_condition = std::abs(z_out_RBF.x()) < TOLERANCE || std::abs(output_cmd_vel[0]) < TOLERANCE;
        bool y_condition = std::abs(z_out_RBF.y()) < TOLERANCE || std::abs(output_cmd_vel[1]) < TOLERANCE;
        bool z_condition = std::abs(z_out_RBF.z()) < TOLERANCE || std::abs(output_cmd_vel[2]) < TOLERANCE;
        
        if (x_condition && y_condition && z_condition) {
            output_cmd_vel[0] += z_out_RBF.x();
            output_cmd_vel[1] += z_out_RBF.y();
            output_cmd_vel[2] += z_out_RBF.z();
        }
        
        // Apply roll, pitch, and yaw feedback
        if (std::abs(roll_pid_feedback) < TOLERANCE || std::abs(output_cmd_vel[3]) < TOLERANCE) {
            output_cmd_vel[3] += roll_pid_feedback;
        }

        if (std::abs(pitch_pid_feedback) < TOLERANCE || std::abs(output_cmd_vel[4]) < TOLERANCE) {
            output_cmd_vel[4] += pitch_pid_feedback;
        }

        
        if (std::abs(yaw_pid_feedback) < TOLERANCE || std::abs(output_cmd_vel[5]) < TOLERANCE) {
            output_cmd_vel[5] += yaw_pid_feedback;
        }
    }
    
    void calculateFeedbackWithPidAntiWindup(const std::array<float, 6>& cmd_vel, std::array<float, 6>& output_cmd_vel) {
        static const std::array<float, 4> anti_windup_gains = {1.0f, 1.0f, 1.0f, 1.0f};
        
        // Calculate current values (z, roll, pitch, yaw)
        std::array<float, 4> current_values;
        std::array<float, 3> rpy_rads;
        calculateRpyFromQuaternion(current_orientation_, rpy_rads);
        
        current_values[0] = current_pressure_;
        current_values[1] = rpy_rads[0];  // roll
        current_values[2] = rpy_rads[1];  // pitch
        current_values[3] = rpy_rads[2];  // yaw
        
        updateSetpoints(cmd_vel);
        
        // Copy input to output
        output_cmd_vel = cmd_vel;
        
        // Calculate PID outputs with anti-windup
        float roll_pid_feedback = pids_[1].compute(setpoints_[1] - current_values[1]);
        pids_[1].state[0] += (clamp(roll_pid_feedback, 1.0f, -1.0f) - roll_pid_feedback) * anti_windup_gains[1];
        
        float pitch_pid_feedback = pids_[2].compute(setpoints_[2] - current_values[2]);
        pids_[2].state[0] += (clamp(pitch_pid_feedback, 1.0f, -1.0f) - pitch_pid_feedback) * anti_windup_gains[2];
        
        float yaw_pid_feedback = pids_[3].compute(setpoints_[3] - current_values[3]);
        pids_[3].state[0] += (clamp(yaw_pid_feedback, 1.0f, -1.0f) - yaw_pid_feedback) * anti_windup_gains[3];
        
        // Depth control with anti-windup
        float z_pid_output = pids_[0].compute(setpoints_[0] - current_values[0]);
        pids_[0].state[0] += (clamp(z_pid_output, 1.0f, -1.0f) - z_pid_output) * anti_windup_gains[0];
        
        // Convert z_pid_output to body frame
        Eigen::Vector3f z_out(0.0f, 0.0f, z_pid_output);
        Eigen::Vector3f z_out_RBF = current_orientation_.inverse() * z_out;
        
        // Apply feedback on x, y, z axes if conditions are met
        bool x_condition = std::abs(z_out_RBF.x()) < TOLERANCE || std::abs(output_cmd_vel[0]) < TOLERANCE;
        bool y_condition = std::abs(z_out_RBF.y()) < TOLERANCE || std::abs(output_cmd_vel[1]) < TOLERANCE;
        bool z_condition = std::abs(z_out_RBF.z()) < TOLERANCE || std::abs(output_cmd_vel[2]) < TOLERANCE;
        
        if (x_condition && y_condition && z_condition) {
            output_cmd_vel[0] += z_out_RBF.x();
            output_cmd_vel[1] += z_out_RBF.y();
            output_cmd_vel[2] += z_out_RBF.z();
        }
        
        // Apply roll, pitch, and yaw feedback
        if (std::abs(roll_pid_feedback) < TOLERANCE || std::abs(output_cmd_vel[3]) < TOLERANCE) {
            output_cmd_vel[3] += roll_pid_feedback;
        }
        if (std::abs(pitch_pid_feedback) < TOLERANCE || std::abs(output_cmd_vel[4]) < TOLERANCE) {
            output_cmd_vel[4] += pitch_pid_feedback;
        }
        if (std::abs(yaw_pid_feedback) < TOLERANCE || std::abs(output_cmd_vel[5]) < TOLERANCE) {
            output_cmd_vel[5] += yaw_pid_feedback;
        }
    }
    
    void calculateFeedbackWithCsController(const std::array<float, 6>& cmd_vel, std::array<float, 6>& output_cmd_vel) {
        // Calculate current values (z, roll, pitch, yaw)
        std::array<float, 4> current_values;
        std::array<float, 3> rpy_rads;
        calculateRpyFromQuaternion(current_orientation_, rpy_rads);
        
        current_values[0] = current_pressure_;
        current_values[1] = rpy_rads[0];  // roll
        current_values[2] = rpy_rads[1];  // pitch
        current_values[3] = rpy_rads[2];  // yaw
        
        updateSetpoints(cmd_vel);
        
        // Copy input to output
        output_cmd_vel = cmd_vel;
        
        // Heave control (depth) using CS controller
        float heave_measurements[2] = {current_pressure_, 0.0f};  // Only using position for now
        float heave_correction = controllers_[0].calculateU(setpoints_[0], current_pressure_, heave_measurements);
        
        // Roll control using CS controller
        float roll_measurements[2] = {rpy_rads[0], 0.0f};  // Only using position for now
        float roll_correction = controllers_[1].calculateU(setpoints_[1], rpy_rads[0], roll_measurements);
        
        // Pitch control using CS controller
        float pitch_measurements[2] = {rpy_rads[1], 0.0f};  // Only using position for now
        float pitch_correction = controllers_[2].calculateU(setpoints_[2], rpy_rads[1], pitch_measurements);
        
        // Yaw control still using PID as in the original
        //
        float yaw_pid_feedback = pids_[3].compute(setpoints_[3] - current_values[3]);
        
        // Convert heave_correction to body frame
        Eigen::Vector3f z_out(0.0f, 0.0f, heave_correction);
        Eigen::Vector3f z_out_RBF = current_orientation_.inverse() * z_out;
        
        // Apply feedback on x, y, z axes if conditions are met
        bool x_condition = std::abs(z_out_RBF.x()) < TOLERANCE || std::abs(output_cmd_vel[0]) < TOLERANCE;
        bool y_condition = std::abs(z_out_RBF.y()) < TOLERANCE || std::abs(output_cmd_vel[1]) < TOLERANCE;
        bool z_condition = std::abs(z_out_RBF.z()) < TOLERANCE || std::abs(output_cmd_vel[2]) < TOLERANCE;
        
        if (x_condition && y_condition && z_condition) {
            output_cmd_vel[0] += z_out_RBF.x();
            output_cmd_vel[1] += z_out_RBF.y();
            output_cmd_vel[2] += z_out_RBF.z();
        }
        
        // Apply roll, pitch, and yaw feedback
        if (std::abs(roll_correction) < TOLERANCE || std::abs(output_cmd_vel[3]) < TOLERANCE) {
            output_cmd_vel[3] += roll_correction;
        }
        
        if (std::abs(pitch_correction) < TOLERANCE || std::abs(output_cmd_vel[4]) < TOLERANCE) {
            output_cmd_vel[4] += pitch_correction;
        }
        
        if (std::abs(yaw_pid_feedback) < TOLERANCE || std::abs(output_cmd_vel[5]) < TOLERANCE) {
            output_cmd_vel[5] += yaw_pid_feedback;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "CS Controller outputs: Heave: %.2f, Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
                    heave_correction, roll_correction, pitch_correction, yaw_pid_feedback);
    }
};

// You need to implement the ControlSystem constructor and calculateU method
ControlSystem::ControlSystem(float minForce, float maxForce, float Kx[2], float Ki) {
    this->minForce = minForce;
    this->maxForce = maxForce;
    this->Kx[0] = Kx[0];
    this->Kx[1] = Kx[1];
    this->Ki = Ki;
    this->ErrorIntegral = 0.0f;
    this->maxErrorIntegral = 100.0f;  // Set reasonable limits for integral windup
    this->minErrorIntegral = -100.0f;
}

float ControlSystem::calculateU(float reference, float y_measurement, float* x_measurement) {
    // Calculate error
    float error = reference - y_measurement;
    
    // Update integral term with anti-windup
    ErrorIntegral += error;
    if (ErrorIntegral > maxErrorIntegral) ErrorIntegral = maxErrorIntegral;
    if (ErrorIntegral < minErrorIntegral) ErrorIntegral = minErrorIntegral;
    
    // Calculate control output: -Kx*x + Ki*integral
    float u = Ki * ErrorIntegral;
    for (int i = 0; i < 2; i++) {
        u -= Kx[i] * x_measurement[i];
    }
    
    // Apply saturation
    if (u > maxForce) u = maxForce;
    if (u < minForce) u = minForce;
    
    return u;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NereoControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}