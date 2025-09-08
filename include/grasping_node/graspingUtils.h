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

#include "visualization_msgs/msg/marker.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "geometry_msgs/msg/pose2_d.hpp"


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
  End
};

using namespace std::chrono_literals;

using std::placeholders::_1;


class GraspNode : public rclcpp::Node {

public:
    GraspNode();

    // Funzioni pubbliche
    void init();
    void create_search_timer();
    void stop_search_timer();
    void create_grasp_timer();
    void stop_grasp_timer();
    void startRoutine();
    

private:
    // ========================== CALLBACKS ==========================
    void keypoint_callback(const geometry_msgs::msg::Point & msg);
    void jointState_callback(const sensor_msgs::msg::JointState & msg);
    void basePoseCallback(const geometry_msgs::msg::Pose2D & msg);

    // ========================== LOGICA DI CONTROLLO MANIPOLATORE ==========================
    void rotate_base(double deg);
    void execute_place();
    void execute_home();
    void initial_scan_pose();
    bool moveJointsAndWait(const std::vector<double> &joint_target_deg, double tolerance_deg, double timeout_sec = 5.0);
    bool moveJointAndWait(int num, double joint_rot, double tolerance_deg, double timeout_sec = 5.0);

    // ========================== LOGICA DI CONTROLLO BASE MOBILE ==========================
    void move_base_towards(const geometry_msgs::msg::Point &new_point, double step = 0.5, double min_distance = 1.0);
    void rotate_and_scan();
    bool rotate_until_target (double angle_rad, double angular_speed = 0.3);

    // ========================== LOGICA DI CONTROLLO GRIPPER ==========================
    void publish_grasp_command(const std::string & command);
    void execute_grasp();

    // ========================== GESTIONE DATI ==========================
    bool process_received_keypoint(const geometry_msgs::msg::Point& msg);
    void fill_keypoint_buffer(const geometry_msgs::msg::Point &msg);
    bool are_keypoints_similar();
    bool filter_keypoint(const geometry_msgs::msg::Point &new_kp,
                         std::vector<geometry_msgs::msg::Point> &buffer,
                         double angle_threshold, double distance_threshold);
    double normalizza_angolo (double a);
    bool xy_is_similar(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, double tolerance = 0.5);
    geometry_msgs::msg::Point nearest_keypoint();

    // ========================== MARKER / DEBUG ==========================
    void publish_marker_at_keypoint(const geometry_msgs::msg::Point &point, std::string name, int index, bool add);
    void delete_all_markers();

    // ========================== MACCHINE A STATI ==========================
    void searchAndApproch();
    void grasping();

    // ========================== VARIABILI MEMBRO ==========================
    std::shared_ptr<ManipulatorMenu> menu_; // Puntatore al menu del manipolatore
    rclcpp::CallbackGroup::SharedPtr callback_group_; //  Gruppo di callback per gestire più callback
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr keypoint_sub_; // Iscrizione al topic dei keypoint
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_grasp_control; // Publisher per i comandi di presa e rilascio
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // Publisher per i marker in RViz
    GraspState Gstate = GraspState::Idle;  // Stato corrente della routine di presa
    std::atomic<bool> keypoint_arrived{false};  // Flag per verificare se il keypoint è stato ricevuto
    geometry_msgs::msg::Pose target_pose;  // Posa iniziale del robot
    std::deque<geometry_msgs::msg::Point> keypoint_buffer_;
    const std::size_t required_similar_keypoints_ = 3; // std::size_t perché confrontato con il buffer che è di tipo std::size_t
    const double position_tolerance_ = 0.2; // 20 cm di tolleranza
    std::atomic<bool> busy{false};
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    bool sim_mode_;
    std::string base_frame_, camera_frame_;
    double orientation_tolerance_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    //variabili per rotazione base mobile
    const int MAX_ROTATION_STEPS = 24;              // 360° / 45°
    const double ROTATION_ANGLE_RAD = (2 * M_PI) / MAX_ROTATION_STEPS; // 45 gradi in radianti
    const double ROTATION_ANGLE_DEG = 360 / MAX_ROTATION_STEPS; // 45 gradi            // rad/s

    std::vector<geometry_msgs::msg::Point> bag_buffer_; // Buffer per i sacchi già visti

    std::vector<double> scan_pose_ = {0., -80., 160., -80., 90., 180.}; // Posa di scansione del robot 
    // -> morbida (0., -60., 120., -60., 90., 180.)
    // -> media (0., -80., 140., -60., 90., 180.)
    // -> spigolosa (0., -80., 160., -80., 90., 180.)

    std::atomic<bool> grasping_{false};
    std::atomic<bool> searching_{false};

    SearchState Sstate = SearchState::Idle;  // Stato corrente della routine di presa
    rclcpp::TimerBase::SharedPtr search_timer_;
    rclcpp::TimerBase::SharedPtr grasp_timer_;

    geometry_msgs::msg::Point latest_keypoint_;
    geometry_msgs::msg::Point latest_keypoint_world_;
    geometry_msgs::msg::Point latest_keypoint_relative_;

    int nearest_index = -1;
    double min_distance = 0.0;
    double angle_deg = 0.0;
    int rotation_index = 1;

    //Subscriber al topic joint_states
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState current_joint_pose_; // Variabile per memorizzare la posa corrente del manipolatore

    //Subscriber a odometria da coppelia
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr base_pose_sub_;
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
    
    rclcpp::CallbackGroup::SharedPtr callback_group_base_;

    int consecutive_mismatches_ = 0;
    const int max_mismatches_ = 3; 
    bool alligned_with_target_ = false;
    bool kp_control = false;
    double step_start_distance_ = -1.0;

    bool busy_grasp = false;
    
};

#endif /* GRASPING_UTILS_H */