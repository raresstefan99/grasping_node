/** @file */
#ifndef GRASPING_UTILS_H
#define GRASPING_UTILS_H

// IMPORT LIBRARIES
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <deque>

#include "manipulators/ManipulatorMenu.h"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "geometry_msgs/msg/pose2_d.hpp"

using RobotiQGripperControl = manipulator_interfaces::srv::RobotiQGripperControl;

enum class GraspState {
  Idle,
  Approach,
  Grasp,
  Move,
  Place,
  Home
};

enum class SearchState {
  Idle,
  InitialPose,
  ScanEnvironment,
  ApproachTarget,
  End
};

using namespace std::chrono_literals;

using std::placeholders::_1;


class GraspNode : public rclcpp::Node {

public:
    // =======================================================================
    // ============================= COSTRUTTORE =============================
    // =======================================================================
    GraspNode();

    // ========================================================================
    // ========================== FUNZIONI PUBBLICHE ==========================
    // ========================================================================
    void init();
    void create_search_timer();
    void stop_search_timer();
    void create_grasp_timer();
    void stop_grasp_timer();
    void startRoutine();
    void stopRoutine();
    void setMenuMode(bool flag);
    void setSimMode(bool flag);
    void busy_keypoint_callback(bool enable);

    // ========================== MACCHINE A STATI ==========================
    void searchAndApproch();
    void grasping();
    void next_step_search();
    void next_step_grasp();

    // ========================== LOGICA DI CONTROLLO MANIPOLATORE ==========================
    void execute_place();
    void grasp_pose();
    void execute_home();
    void initial_scan_pose();
    void execute_grasp();
    void goDownUntilForce(double target_force = 6.0, double delta_z = 0.005, bool linear = true);
    void alignWithKeypoint(const geometry_msgs::msg::Pose& pose_keypoint, double tolerance = 0.01);

    // ========================== GESTIONE DATI ==========================
    bool process_received_keypoint();
    void fill_keypoint_buffer(const geometry_msgs::msg::Point &msg);
    bool are_keypoints_similar();
    bool filter_keypoint(const geometry_msgs::msg::Point &new_kp,
                         std::vector<geometry_msgs::msg::Point> &buffer,
                         double angle_threshold, double distance_threshold);
    double normalizza_angolo (double a);
    bool xy_is_similar(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, double tolerance = 0.5);
    geometry_msgs::msg::Point nearest_keypoint();
    bool cluster_keypoint(const geometry_msgs::msg::Point &new_kp,const double angle_threshold = 5, const double distance_threshold = 0.3, const double z_threshold = 0.15);
    bool is_duplicate(const geometry_msgs::msg::Point &new_point, double tolerance = 0.3);


    // ========================== MARKER / DEBUG ==========================
    void publish_marker_at_keypoint(const geometry_msgs::msg::Point &point, std::string name, int index, bool add);
    void delete_all_markers();

    // ========================== GRIPPER ==========================
    bool open(int32_t speed = 100, int32_t force = 100);
    bool close(int32_t speed = 100, int32_t force = 100);

    // ========================== LOGICA DI CONTROLLO BASE MOBILE ==========================
    void move_base_towards(const geometry_msgs::msg::Point &new_point, double step = 0.5, double min_distance = 1.0);
    void rotate_and_scan(double target_yaw = 360);
    bool rotate_until_target (double angle_rad, double angular_speed = 0.3);
    bool rotate_base(double deg);
    bool goToXY(double target_x, double target_y);


    // ========================== VARIABILI FLAG ==========================
    std::atomic<bool> keypoint_arrived{false};  // Flag per verificare se il keypoint è stato ricevuto
    std::atomic<bool> busy{false};  // Flag per indicare se il robot è occupato in una routine
    std::atomic<bool> grasping_{false}; // Flag per indicare se il robot è in modalità presa
    std::atomic<bool> searching_{false}; // Flag per indicare se il robot è in modalità ricerca

    SearchState Sstate = SearchState::Idle;  // Stato corrente della routine di ricerca
    GraspState Gstate = GraspState::Idle;  // Stato corrente della routine di presa

    // ========================== VARIABILI PER LA GESTIONE KEYPOINT ==========================
    geometry_msgs::msg::Pose target_pose;  // Posa iniziale del robot
    std::deque<geometry_msgs::msg::Point> keypoint_buffer_;
    const std::size_t required_similar_keypoints_ = 3; 
    const double position_tolerance_ = 0.2; // 20 cm di tolleranza
    geometry_msgs::msg::Point latest_keypoint_;
    geometry_msgs::msg::Point latest_keypoint_world_;
    geometry_msgs::msg::Point latest_keypoint_relative_;

    // ========================== VARIABILI PER LA GESTIONE DEI SACCHI ==========================
    std::vector<geometry_msgs::msg::Point> bag_buffer_; // Buffer per i sacchi già visti
    std::vector<std::vector<geometry_msgs::msg::Point>> clusters_; // Cluster di sacchi
    int nearest_index = -1;
    double min_distance = 0.0;

    // ========================== VARIABILI PER LA GESTIONE SENSORE DI FORZA ==========================
    double force_z_;
    double height;

    // ========================== PUNTATORE AL MANIPOLATORE ==========================
    std::shared_ptr<ManipulatorMenu> menu_; 
    

private:
    // ======================================================================
    // ========================== FUNZIONI PRIVATE ==========================
    // ======================================================================

    // ========================== CALLBACKS ==========================
    void keypoint_callback(const geometry_msgs::msg::Point & msg);
    void jointState_callback(const sensor_msgs::msg::JointState & msg);
    void basePoseCallback(const geometry_msgs::msg::Pose2D & msg);
    void forceCallback(const geometry_msgs::msg::WrenchStamped & msg);

    // ========================== CONTROLLO JOINTS ==========================
    bool moveJointsAndWait(const std::vector<double> &joint_target_deg, double tolerance_deg, double timeout_sec = 5.0);
    bool moveJointAndWait(int num, double joint_rot, double tolerance_deg, double timeout_sec = 5.0);


    // ========================== LOGICA DI CONTROLLO GRIPPER ==========================
    void publish_grasp_command(const std::string & command);

    bool waitForService(const std::chrono::milliseconds & timeout = 1000ms);
    bool command(int32_t position, int32_t speed, int32_t force);


    // =======================================================================
    // ========================== VARIABILI PRIVATE ==========================
    // =======================================================================

    // ========================== CALLBACK GROUPS ==========================
    rclcpp::CallbackGroup::SharedPtr callback_group_keypoint_;
    rclcpp::CallbackGroup::SharedPtr callback_group_joint_;
    rclcpp::CallbackGroup::SharedPtr callback_group_base_;
    rclcpp::CallbackGroup::SharedPtr callback_group_force_;

    // ========================== VARIABILI TIMER ==========================
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr search_timer_;
    rclcpp::TimerBase::SharedPtr grasp_timer_;
    
    // ========================== SUBCRIBERS ==========================
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr keypoint_sub_; // Iscrizione al topic dei keypoint
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr base_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    
    // ========================== PUBLISHERS ==========================
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_grasp_control; // Publisher per i comandi di presa e rilascio
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // Publisher per i marker in RViz
    public:rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Service client per il gripper Robotiq
    rclcpp::Client<RobotiQGripperControl>::SharedPtr client_;
    
    // ========================== VARIABILI MACCHINA A STATI DI RICERCA ==========================
    int consecutive_mismatches_ = 0;
    const int max_mismatches_ = 3; 
    bool alligned_with_target_ = false;
    bool kp_control = false;
    double step_start_distance_ = -1.0;

    // ========================== VARIABILI MACCHINA A STATI DI PRESA ==========================
    bool busy_grasp = false;

    
    // ========================== PARAMETRI YAML ==========================
    bool menu_mode_;
    bool sim_mode_;
    std::string base_frame_, camera_frame_;
    double orientation_tolerance_;
    std::vector<double> home_pose_;
    std::vector<double> scan_pose_;
    std::vector<double> grasping_pose_; 
    bool neobotix_mpo_500_;
    geometry_msgs::msg::Pose box_down_pose_;
    std::vector<double> box_down_pose_vector_;
    std::vector<double> box_down_size_;
    geometry_msgs::msg::Pose box_up_pose_;
    std::vector<double> box_up_pose_vector_;
    std::vector<double> box_up_size_;

    

    // ========================== VARIABILI PER LA GESTIONE DEI JOINT ==========================
    sensor_msgs::msg::JointState current_joint_pose_; // Variabile per memorizzare la posa corrente del manipolatore
    

    // ========================== VARIABILI ODOMETRIA ==========================
    double base_x_ = 0.0;
    double base_y_ = 0.0;
    double base_theta_ = 0.0;

    // Pose della base in mondo (Coppelia)
    double xb;
    double yb;
    double th;
    double zb = 0.3;   // altezza base in coppelia

    // Punto relativo nel frame base_link senza rotazione
    double xr;
    double yr;
    double zr;

    double c = std::cos(th);
    double s = std::sin(th);

    // Trasformazione: base mobile + rotazione
    double x_r;
    double y_r;
    double z_r;

    // Trasformazione: base mobile + rotazione + traslazione
    double x_w;
    double y_w;
    double z_w;
    
};


/// Struttura per ogni voce del menu
struct MenuChoice {
    std::string description;
    std::function<void()> callback;
};

struct MenuSection {
    std::string name;
    size_t start_index;
    size_t end_index;
};

// ==========================================================
// ========================== MENU ==========================
// ==========================================================
class AutoGraspMenu {
public:
    AutoGraspMenu(const std::string &title);

    // Aggiungi una scelta al menu
    void addChoice(const std::string &desc, std::function<void()> callback);

    // Aggiungi una sezione per raggruppare le scelte
    void addSection(const std::string &section_name, size_t start_index, size_t end_index);

    // Per avviare lo spinner del menu
    void spinnerMenu();

    int last_; // tiene traccia dell'ultima voce inserita

private:
    std::string title_;
    std::vector<MenuChoice> choices_;
    std::vector<MenuSection> sections_;
    std::vector<std::string> names_;
};

#endif /* GRASPING_UTILS_H */