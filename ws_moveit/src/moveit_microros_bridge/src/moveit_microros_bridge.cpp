#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <map>
#include <vector>
#include <cmath>

class MoveItToMicroROSBridge : public rclcpp::Node
{
public:
    MoveItToMicroROSBridge() : Node("moveit_microros_bridge")
    {
        // Configuración de parámetros
        this->declare_parameter("joint_mapping", std::vector<std::string>{"rev_1", "rev_2", "rev_3"});
        this->declare_parameter("min_angle_change", 1.0); // REDUCIR a 1 grado
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("use_relative_angles", true);
        
        // Obtener parámetros
        joint_mapping_ = this->get_parameter("joint_mapping").as_string_array();
        min_angle_change_ = this->get_parameter("min_angle_change").as_double();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        use_relative_angles_ = this->get_parameter("use_relative_angles").as_bool();
        
        // Inicializar ángulos
        last_received_angles_.resize(joint_mapping_.size(), 0.0);
        accumulated_changes_.resize(joint_mapping_.size(), 0.0);
        last_sent_angles_.resize(joint_mapping_.size(), 0.0);
        first_message_ = true;
        
        // Publicador para micro-ROS
        motor_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/motor_angles", 10);
            
        // Suscriptor a JointState de MoveIt
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MoveItToMicroROSBridge::jointStateCallback, this, std::placeholders::_1));
            
        // Timer para publicación periódica
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&MoveItToMicroROSBridge::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "MoveIt to micro-ROS Bridge iniciado");
        RCLCPP_INFO(this->get_logger(), "Modo: %s", use_relative_angles_ ? "RELATIVO" : "ABSOLUTO");
        RCLCPP_INFO(this->get_logger(), "Cambio mínimo: %.1f°", min_angle_change_);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (first_message_) {
            // Primer mensaje - inicializar con los valores actuales
            for (size_t i = 0; i < joint_mapping_.size(); ++i) {
                const std::string& target_joint = joint_mapping_[i];
                auto it = std::find(msg->name.begin(), msg->name.end(), target_joint);
                if (it != msg->name.end()) {
                    int index = std::distance(msg->name.begin(), it);
                    double angle_rad = msg->position[index];
                    double angle_deg = angle_rad * 180.0 / M_PI;
                    last_received_angles_[i] = angle_deg;
                    last_sent_angles_[i] = angle_deg;
                }
            }
            first_message_ = false;
            RCLCPP_INFO(this->get_logger(), "Primer mensaje recibido - inicializando ángulos");
            return;
        }

        bool significant_change = false;
        
        for (size_t i = 0; i < joint_mapping_.size(); ++i) {
            const std::string& target_joint = joint_mapping_[i];
            
            auto it = std::find(msg->name.begin(), msg->name.end(), target_joint);
            if (it != msg->name.end()) {
                int index = std::distance(msg->name.begin(), it);
                double new_angle_rad = msg->position[index];
                double new_angle_deg = new_angle_rad * 180.0 / M_PI;
                
                // Calcular cambio desde el último ángulo recibido
                double delta = new_angle_deg - last_received_angles_[i];
                
                // Acumular cambios pequeños
                accumulated_changes_[i] += delta;
                
                // Verificar si el cambio acumulado es significativo
                if (std::abs(accumulated_changes_[i]) >= min_angle_change_) {
                    significant_change = true;
                    
                    RCLCPP_DEBUG(this->get_logger(), "Joint %s: Δ acumulado = %.2f°", 
                                target_joint.c_str(), accumulated_changes_[i]);
                }
                
                last_received_angles_[i] = new_angle_deg;
            }
        }
        
        if (significant_change) {
            new_data_available_ = true;
        }
    }
    
    void timerCallback()
    {
        if (!new_data_available_) {
            return;
        }
        
        auto motor_msg = std_msgs::msg::Int32MultiArray();
        motor_msg.data.resize(3);
        bool has_valid_data = false;
        
        for (size_t i = 0; i < 3; ++i) {
            if (std::abs(accumulated_changes_[i]) >= min_angle_change_) {
                if (use_relative_angles_) {
                    // Enviar cambio relativo acumulado
                    motor_msg.data[i] = static_cast<int>(std::round(accumulated_changes_[i]));
                } else {
                    // Enviar ángulo absoluto actual
                    motor_msg.data[i] = static_cast<int>(std::round(last_received_angles_[i]));
                }
                has_valid_data = true;
                
                // Resetear acumulador para este joint
                accumulated_changes_[i] = 0.0;
            } else {
                motor_msg.data[i] = 0; // No mover este joint
            }
        }
        
        if (has_valid_data) {
            motor_pub_->publish(motor_msg);
            
            if (use_relative_angles_) {
                RCLCPP_INFO(this->get_logger(), "Publicado RELATIVO: [%+d, %+d, %+d]°", 
                           motor_msg.data[0], motor_msg.data[1], motor_msg.data[2]);
            } else {
                RCLCPP_INFO(this->get_logger(), "Publicado ABSOLUTO: [%d, %d, %d]°", 
                           motor_msg.data[0], motor_msg.data[1], motor_msg.data[2]);
            }
        }
        
        new_data_available_ = false;
    }
    
    // Variables miembro
    std::vector<std::string> joint_mapping_;
    std::vector<double> last_received_angles_;  // Últimos ángulos recibidos de MoveIt
    std::vector<double> accumulated_changes_;   // Cambios acumulados
    std::vector<double> last_sent_angles_;      // Últimos ángulos enviados
    double min_angle_change_;
    bool use_relative_angles_;
    bool first_message_ = true;
    bool new_data_available_ = false;
    
    // ROS 2
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItToMicroROSBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}