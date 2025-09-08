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
  ApproachTarget,
  End
};




using namespace std::chrono_literals;

using std::placeholders::_1;


class GraspNode : public rclcpp::Node {
public:

  // ========================================================= COSTRUTTORE =========================================================
  GraspNode() : Node("grasp_bag_node") {
    // Per evitare conflitti di accesso alle risorse condivise - Race Conditions
    // Callback separati per evitare che la ricezione dei keypoint interferisca con l'input utente
    // Creazione dei gruppi
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // MutuallyExclusive per evitare conflitti tra callback
    
    //Opzioni per la subscription
    rclcpp::SubscriptionOptions options_keypoint;
    options_keypoint.callback_group = callback_group_;

    // Iscrizione al topic dei keypoint
    // Utilizza il callback group per evitare conflitti di accesso alle risorse condivise
    keypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/keypoint_data",
      rclcpp::QoS(1),
      std::bind(&GraspNode::keypoint_callback, this, _1),
      options_keypoint);

    // Publisher del comando di presa e rilascio
    pub_grasp_control = this->create_publisher<std_msgs::msg::String>("/grasp_control", 1);

    // Publisher per il marker in RViz
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("keypoint_marker", 1);
    
    // Publisher per i comandi di movimento del robot
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // Gruppo separato per odometria/joint states
    callback_group_base_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_base;
    options_base.callback_group = callback_group_base_;

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(1),
      std::bind(&GraspNode::jointState_callback, this, _1),
      options_base);
    
    base_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
    "/base_pose2d", rclcpp::QoS(1),
    std::bind(&GraspNode::basePoseCallback, this, _1),
    options_base);
  


    // === PARAMETRI YAML ===
    this->declare_parameter<bool>("sim_mode", false); // false di default
    this->get_parameter("sim_mode", sim_mode_);

    this->declare_parameter<std::string>("base_frame", "base_link");
    this->get_parameter("base_frame", base_frame_);

    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->get_parameter("camera_frame", camera_frame_);

    this->declare_parameter<double>("orientation_tolerance", 0.01);
    this->get_parameter("orientation_tolerance", orientation_tolerance_);
    
    // Timer inizializzazione start routine
    init_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        this->startRoutine();
        this->init_timer_->cancel();
      }
    );
  }

  // ========================================================= INIZIALIZZA MANIPOLATORE =========================================================
  void init() {
    ManipulatorMenuParams params;
  
    params.manipulator_name = "manipulator";
    params.planning_group = "ur_manipulator";
    params.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    params.base_link_name = "base_link";
    //params.gripper = "tcp_gripper";
    params.gripper = "robotiq_85";
    params.gripper_group = "robotiq_85_gripper";
    
    // Ricava il path del pacchetto manipulators
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("manipulators");
    params.known_poses_path = pkg_share_dir + "/config/known_poses.yaml";   
    
    menu_ = std::make_shared<ManipulatorMenu>(params, shared_from_this(), false);
  }

  // ========================================================= FUNZIONI PUBBLICHE =========================================================

  // Funzione per gestire lo stato della routine
  void set_state(GraspState new_state){ 
    Gstate = new_state; 
  }

  // Funzione per inizializzare la posa del robot
  // Questa funzione imposta la posa iniziale del robot utilizzando una posa nota
  void home_robot_pose() {
    if (!menu_) return;
    auto joints = menu_->getKnownPose("home_gripper_down"); // joints è di tipo std::vector<double>
    if (joints.empty()) return;
    //menu_->publishJointGoal(joints);
    menu_->cartesianPlanExecuteAndWait({menu_->pose_from_vector(joints)}, {}, "", 5);
    rclcpp::sleep_for(std::chrono::milliseconds(5000));

  }

private:
  // ========================================================= VARIABILI PRIVATE  =========================================================

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




  // ========================================================= FUNZIONI PRIVATE  =========================================================

  // Funzione per pubblicare il comando di presa o rilascio
  // Questa funzione invia un messaggio al topic /grasp_control con il comando specificato
  // Il comando può essere "grasp" per prendere l'oggetto o "release" per rilasciarlo
  void publish_grasp_command(const std::string & command) {
    std_msgs::msg::String msg;
    msg.data = command;
    pub_grasp_control->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Comando '%s' pubblicato.", command.c_str());
  }

  // Funzione per pubblicare un marker in RViz al keypoint ricevuto
  // Questa funzione crea un marker di tipo SPHERE e lo pubblica sul topic /visualization_marker
  void publish_marker_at_keypoint(const geometry_msgs::msg::Point &point, std::string name, int index, bool add) {
    // ---- Sfera ----
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = "base_link";
    sphere_marker.header.stamp = this->now();
    sphere_marker.ns = "keypoint_marker";
    sphere_marker.id = index; 
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;

    if (add) {
      sphere_marker.action = visualization_msgs::msg::Marker::ADD;
      sphere_marker.pose.position = point;
      sphere_marker.pose.orientation.w = 1.0;
      sphere_marker.scale.x = 0.05;
      sphere_marker.scale.y = 0.05;
      sphere_marker.scale.z = 0.05;
      sphere_marker.color.r = 0.0f;
      sphere_marker.color.g = 1.0f;
      sphere_marker.color.b = 0.0f;
      sphere_marker.color.a = 1.0f;
      //sphere_marker.lifetime = rclcpp::Duration::from_seconds(10);
    } else {
      sphere_marker.action = visualization_msgs::msg::Marker::DELETE;
    }

    marker_pub_->publish(sphere_marker);

    // ---- Testo ----
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "base_link";
    text_marker.header.stamp = this->now();
    text_marker.ns = "keypoint_marker_text";
    text_marker.id = index + 1000; // id diverso dalla sfera per evitare conflitti
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

    if (add) {
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = point;
      text_marker.pose.position.z += 0.07; // testo sopra la sfera
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.05; // solo Z conta per il testo
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 1.0f;
      text_marker.color.a = 1.0f;
      text_marker.text = name + std::to_string(index);
      //text_marker.lifetime = rclcpp::Duration::from_seconds(10);
    } else {
      text_marker.action = visualization_msgs::msg::Marker::DELETE;
    }

    marker_pub_->publish(text_marker);
  }
  
  void delete_all_markers() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_pub_->publish(marker);
  }

  void execute_grasp(){
    rclcpp::sleep_for(std::chrono::milliseconds(5000)); // Attendi che il robot sia pronto
    publish_grasp_command("grasp");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    menu_->moveGripper(true);
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Attendi chiusura gripper
    menu_->move_along_z(0.2, true); // Alza l'oggetto di  20 cm
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
  }

  // Funzione per ruotare giunto 0 - non usata
  void rotate_base(double deg){
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    if (sim_mode_){
      menu_->oneJointMove(0, deg); // Ruota il braccio di 90 gradi
    }
    else{
      // TODO rotazione base mobile 
    }
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
  }

  void execute_place(){
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    menu_->move_along_z(-0.2, true);  // Abbassa l'oggetto di 20 cm
    rclcpp::sleep_for(std::chrono::milliseconds(5000));  // Attendi che l'oggetto venga posizionato
    publish_grasp_command("release");
    menu_->moveGripper(false);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));  // Attendi apertura gripper
    menu_->move_along_z(0.3, true);  // Alza il braccio di 30 cm dopo aver rilasciato l'oggetto
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
  }

  void execute_home(){
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    menu_->publishJointGoal(menu_->getKnownPose("home_gripper_down"));
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
  }

  void initial_scan_pose(){
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    moveJointsAndWait(scan_pose_, 1.0, 15.0);
    //menu_->cartesianPlanExecuteAndWait({menu_->pose_from_vector(scan_pose_)}, {}, "", 5);
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
  }

  // TODO funzione vecchia non in uso - decidere se tenerla o no
  bool depth_check(){
    if (target_pose.position.z < 0.2 || target_pose.position.z > 2.0) {
      RCLCPP_WARN(this->get_logger(), "Profondità anomala %.2f, punto scartato.", target_pose.position.z);
      RCLCPP_WARN(this->get_logger(), "Rigenerazione del keypoint...");
      keypoint_buffer_.clear();
      return false;
    }
    return true;
  }

  // Funzione per aggiornare il target in prossimità del sacco e pianificare/eseguire la posa dritta, inclinata di 45 gradi o dall'alto
  bool process_received_keypoint(const geometry_msgs::msg::Point& msg) {
    
    for(int i = 0; i < 3; i++) {
      target_pose.orientation = menu_->quaternion_from_euler(0 + 45 * i, 90, 180);
      manipulator_interfaces::msg::TrajectoryResult traj_result = menu_->planAndWait(target_pose);
      if (traj_result.success)
      {
          // target_pose definito -> prediligo presa orizzontale, poi obliqua e infine verticale
          RCLCPP_INFO(this->get_logger(), "Presa pianificata con successo con orientamento %d gradi.", 180 + 45 * i);
          keypoint_arrived = true;
          bool success = menu_->executeAndWait(traj_result.trajectory);
            if (success)
            {
                std::cout << "Trajectory executed successfully." << std::endl;
            }
            else
            {
                std::cout << "Trajectory execution failed." << std::endl;
            }
          return true;
      }
    }

    //TODO avere altri 2 orientation possibili dall'alto, frontale 180 0 90 e 90 0 90
    RCLCPP_INFO(this->get_logger(), "Keypoint ricevuto ma presa non pianificata: x=%.3f y=%.3f z=%.3f xR=%.3f Ry=%.3f Rz=%.3f", 
      target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
    // Visualizza il marker in RViz
    //publish_marker_at_keypoint(target_pose.position, 1, true);

    return false;
  }
  
  // Funzione che prepara un buffer di 5 keypoint
  void fill_keypoint_buffer(const geometry_msgs::msg::Point& msg){

    keypoint_buffer_.push_back(msg);
        if (keypoint_buffer_.size() > required_similar_keypoints_) {
            keypoint_buffer_.pop_front();
        }
  }
  
  // Controllo che nel keypoint_buffer ricevo tot keypoint simili per confermare keypoint
  bool are_keypoints_similar()
  {
    if (keypoint_buffer_.size() < required_similar_keypoints_){
      RCLCPP_WARN(this->get_logger(), "Numero di keypoint ricevuti insufficiente: %zu, atteso: %ld", keypoint_buffer_.size(), required_similar_keypoints_);
      return false;
    } 

    const auto& ref = keypoint_buffer_.front();
    for (const auto& kp : keypoint_buffer_)
    {
        double dx = kp.x - ref.x;
        double dy = kp.y - ref.y;
        double dz = kp.z - ref.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist > position_tolerance_){
          RCLCPP_WARN(this->get_logger(), "Keypoint non simili, distanza massima superata: %.3f > %.3f", dist, position_tolerance_);
          return false;
        }
    }
    return true;
  }
  
  // TODO da visionare non completa - usare funzione di ITALO
  void add_collision_box(const geometry_msgs::msg::Point &center, int index)
  {
    // Crea il messaggio per l'oggetto di collisione
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "bag_box_" + std::to_string(index);  // ID unico per ogni sacco

    // Definisci la forma: BOX
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.2, 0.2, 0.2};  // LxWxH in metri

    // Definisci la posa
    geometry_msgs::msg::Pose box_pose;
    box_pose.position = center;
    box_pose.orientation.w = 1.0;  // orientamento neutro
    menu_->addObj("bag_box_" + std::to_string(index), 1, {0.5, 0.5, 0.5}, box_pose, 0);
    /*
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Planning Scene Interface statico per persistenza
    static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Debug
    RCLCPP_INFO(this->get_logger(), "Adding box %s at (%.2f, %.2f, %.2f)",
                collision_object.id.c_str(),
                center.x, center.y, center.z);

    rclcpp::sleep_for(std::chrono::milliseconds(500)); // garantisce sync scena
    
    planning_scene_interface.addCollisionObjects({collision_object});
    */
    RCLCPP_INFO(this->get_logger(), "Box di collisione %s aggiunto.", collision_object.id.c_str());
  }

  void remove_collision_box(int index)
  {
    menu_->removeObj("bag_box_" + std::to_string(index+1));
    RCLCPP_INFO(this->get_logger(), "Box di collisione bag_box_%d rimosso.", index);
  }

  void clear_all_collision_boxes()
  {
    // TODO funziona?
    static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto existing = planning_scene_interface.getKnownObjectNames();
    planning_scene_interface.removeCollisionObjects(existing);
    RCLCPP_INFO(this->get_logger(), "Tutte le collision box rimosse.");
  }

  // Funzione per confrontare un keypoint nuovo con i keypoint nel bag_buffer per evitare più keypoint per lo stesso sacco
  // TODO attualmente non usata e sostituita con filter_keypoint
  bool is_duplicate(const geometry_msgs::msg::Point &new_point, double tolerance = 0.3)
  {
    RCLCPP_INFO(this->get_logger(), "Check duplicati: %lu keypoint nel buffer", bag_buffer_.size());
    for (const auto &p : bag_buffer_)
    {
      double dx = new_point.x - p.x;
      double dy = new_point.y - p.y;
      double dz = new_point.z - p.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < tolerance)
      {
        RCLCPP_INFO(this->get_logger(), "Keypoint duplicato trovato.");
        return true;
      }
    }
    return false;
  }

  // Funzione filtro per evitare keypoint duplicati per lo stesso sacco
  bool filter_keypoint(const geometry_msgs::msg::Point& new_kp, 
                     std::vector<geometry_msgs::msg::Point>& buffer,
                     double angle_threshold, double distance_threshold)
  {
    double angle_new = std::atan2(new_kp.y, new_kp.x);
    double dist_new  = std::sqrt(new_kp.x*new_kp.x + new_kp.y*new_kp.y + new_kp.z*new_kp.z);

    for (auto it = buffer.begin(); it != buffer.end(); ++it) {
        double angle_old = std::atan2(it->y, it->x);
        double dist_old  = std::sqrt(it->x*it->x + it->y*it->y + it->z*it->z);

        // Se hanno direzione simile
        if (std::fabs(angle_new - angle_old) < angle_threshold) {
            if (dist_new < dist_old - distance_threshold) {
                // il nuovo è davanti → sostituisco il vecchio
                *it = new_kp;
                return true;
            } else {
                // il nuovo è dietro o quasi uguale → scarto
                return false;
            }
        }
    }

    // nessun keypoint in direzione simile → lo aggiungo
    buffer.push_back(new_kp);
    return true;
  }

  // Funzione per portare il robot in una posizione di joint target e attendo fino a quando non viene raggiunto
  bool moveJointsAndWait(const std::vector<double> &joint_target_deg, double tolerance_deg, double timeout_sec = 5.0) 
  {
    if (joint_target_deg.size() != 6)
    {
      RCLCPP_ERROR(this->get_logger(), "Target joints vector must have 6 elements.");
      return false;
    }

    // Debug posizione attuale
    //for (unsigned long k = 0; k < 6; k++){std::cout << "Joint " << k << " : " << current_joint_pose_.position[k] * 180.0 / M_PI << std::endl;}

    // Debug target
    //for (unsigned long k = 0; k < 6; k++){std::cout << "Joint " << k << " : " << joint_target_deg[k] << std::endl;}

    if (!menu_) {
    RCLCPP_ERROR(this->get_logger(), "menu_ non inizializzato!");
    return false;
    }

    // Pubblica il target
    menu_->publishJointGoal(joint_target_deg, {}, true);

    // Attesa fino al raggiungimento del target
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    auto start_time = steady_clock.now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok())
    {
      // Timeout
      if ((steady_clock.now() - start_time).seconds() > timeout_sec)
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for all joints to reach target.");
        return false;
      }

      bool all_reached = true;

      for (unsigned long k = 0; k < 6; k++)
      {
        double current_deg = current_joint_pose_.position[k] * 180.0 / M_PI;
        double error = std::fabs(current_deg - joint_target_deg[k]);
        //debug
        //std::cout << "Error joint " << k << " = " << error << std::endl;

        if (error > tolerance_deg)
        {
          all_reached = false;
          break;
        }
      }

      if (all_reached)
      {
        RCLCPP_INFO(this->get_logger(), "All joints reached target within tolerance.");
        return true;
      }

      rate.sleep();
    }

    return false;
  }

  // Funzione per muovere un solo giunto e attendo fino a quando non è raggiunto
  bool moveJointAndWait(int num, double joint_rot, double tolerance_deg, double timeout_sec = 5.0)
  {
    if(num < 0 || num >= 6) {
      RCLCPP_ERROR(this->get_logger(), "Invalid joint number %d", num);
      return false;
    }
    // Crea il target a partire dalla posa attuale
    std::vector<double> joint_target_deg;
    for (unsigned long k = 0; k < 6; k++)
    {
      joint_target_deg.push_back(current_joint_pose_.position[k] * 180.0 / M_PI); // rad → gradi
      //debug
      //std::cout << "Joint " << k << " : " <<current_joint_pose_.position[k] * 180 / M_PI << std::endl;
    }

    // Modifica solo il giunto richiesto
    joint_target_deg[num] = joint_rot;
    for (unsigned long k = 0; k < 6; k++)
    {
      //debug
      //std::cout << "Joint " << k << " : " << joint_target_deg[k] << std::endl;
    }

    if (!menu_) {
    RCLCPP_ERROR(this->get_logger(), "menu_ non inizializzato!");
    return false;
    }

    // Pubblica il target e avvia il movimento
    menu_->publishJointGoal(joint_target_deg, {}, true); // true = esegui subito

    // Attesa fino al raggiungimento del target
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    auto start_time = steady_clock.now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok())
    {
      // Timeout
      if ((steady_clock.now() - start_time).seconds() > timeout_sec)
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for joint %d to reach target.", num);
        return false;
      }

      // Differenza attuale vs target (in gradi)
      double current_deg = current_joint_pose_.position[num+6] * 180.0 / M_PI;
      double error = std::fabs(current_deg - joint_target_deg[num]);
      //debug
      //std::cout << "Error joint = " << error << std::endl;

      if (error <= tolerance_deg)
      {
        RCLCPP_INFO(this->get_logger(), "Joint %d reached target (error: %.3f deg).", num, error);
        return true;
      }

      rate.sleep();
    }

    return false;
  }

  // Funzione di normalizzazione angolo [-pi, pi]
  double normalizza_angolo (double a){
    return std::atan2(std::sin(a), std::cos(a));
  }

  // Funzione per approccio della base mobile in direzione del sacco più vicino 
  void move_base_towards(const geometry_msgs::msg::Point &new_point, double step = 0.5, double min_distance = 1.0) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    // Differenza posizione target - base
    double dx = new_point.x - base_x_;
    double dy = new_point.y - base_y_;
    double distance = std::sqrt(dx * dx + dy * dy);
    // inizializza step se non è già stato fatto
    if (step_start_distance_ < 0.0) {
      step_start_distance_ = distance;
    }

    // Angolo desiderato verso il target
    double target_yaw = std::atan2(dy, dx); 

    // Errore di orientamento
    double base_heading = normalizza_angolo(base_theta_ + M_PI); // M_PI perché il robot guarda all'indietro -x
    double yaw_error = normalizza_angolo(target_yaw - base_heading); 
    if (std::fabs(yaw_error) < 0.01) yaw_error = 0.0; // evita piccole oscillazioni

    // Parametri velocità
    double linear_speed  = 1.5;   // m/s
    double angular_speed = 0.5;   // rad/s
    double angular_kp = 1.2;       // guadagno proporzionale

    // --- Caso 1: Sono arrivato abbastanza vicino e allineato ---
    if (distance <= min_distance && std::fabs(yaw_error) <= 0.05) {
      RCLCPP_INFO(this->get_logger(), "Vicino al target (d=%.3f m). Prendo il sacco.", distance);
      alligned_with_target_ = true;
      grasping_ = true;
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
      return;
    }

    // --- Caso 2: Devo correggere orientamento ---
    if (std::fabs(yaw_error) > 0.05) {
      alligned_with_target_ = false;
      twist.angular.z = angular_kp * yaw_error;
      if (twist.angular.z > angular_speed) twist.angular.z = angular_speed;
      if (twist.angular.z < -angular_speed) twist.angular.z = -angular_speed;
      twist.linear.x  = 0.0;
      RCLCPP_INFO(this->get_logger(), "Correzione orientamento: errore yaw=%.2f rad", yaw_error);
    } 
    // --- Caso 3: Allineato → Avanza per un passo ---
    else {
      alligned_with_target_ = true;
      busy = false;
      twist.angular.z = angular_kp * yaw_error; // piccola correzione in marcia
      // quanto ho percorso dall'inizio dello step
      double travelled = step_start_distance_ - distance;
      // Avanza al massimo fino a min_distance
      double forward = std::min(step, distance - min_distance);
      twist.linear.x = - std::min(linear_speed * (forward + 0.1), linear_speed); // il meno perché il robot guarda all'indietro -x :: +0.1 per evitare di fermarsi siccome forward tende a zero
      if (travelled >= step) {
        twist.linear.x = 0.0; // ferma se troppo vicino
        twist.angular.z = 0.0;
        kp_control = true;
        step_start_distance_ = -1.0; // reset step
        RCLCPP_INFO(this->get_logger(), "Step completato: %.2f m (dist=%.2f m, travelled=%.2f m, yaw=%.2f°)", 
                  step, distance, travelled, yaw_error*180.0/M_PI);
      }
      RCLCPP_INFO(this->get_logger(), "Avanzo di uno step: %.2f m (dist=%.2f m, travelled=%.2f m, yaw=%.2f°)", 
                  step, distance, travelled, yaw_error*180.0/M_PI);
    }

    // Pubblica il comando di movimento
    cmd_vel_pub_->publish(twist);
  }

  // Funzione per controllare se un nuovo keypoint è simile al precedente - utile per capire se ci si sta avvicinando verso un keypoint valido
  bool xy_is_similar(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, double tolerance = 0.5) 
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    //double dz = p1.z - p2.z;

    double distance = std::sqrt(dx*dx + dy*dy); // manca dz
    RCLCPP_INFO(this->get_logger(), "Controllo keypoint simili, distanza = %.3f tolleranza = %.3f", distance, tolerance);

    return (distance <= tolerance);
  }

  // Funzione per girare su se stesso e scanarizzare l'ambiente in cerca di oggetti
  void rotate_and_scan()
  {
    RCLCPP_INFO(this->get_logger(), "Inizio scansione...");
    if(!sim_mode_) {

      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.0;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = 0.0;
      busy = false;
      // Velocità angolare costante (rad/s)
      double angular_speed = 0.08;  // 0.08
      twist.angular.z = angular_speed;

      // Salvo l'orientamento iniziale
      double start_theta = base_theta_;
      double accumulated_rotation = 0.0;

      rclcpp::Rate rate(10);  // 10 Hz

      // Pubblica il comando per ruotare
      cmd_vel_pub_->publish(twist);

      while (rclcpp::ok()) {

        // Calcolo rotazione accumulata
        double delta_yaw = normalizza_angolo(base_theta_ - start_theta);
        accumulated_rotation = accumulated_rotation + std::fabs(delta_yaw);
        start_theta = base_theta_;  // aggiorna riferimento
        //RCLCPP_INFO(this->get_logger(), "accumulated_rotation = %.1f°", accumulated_rotation * 180.0 / M_PI);
        // Se ho completato ~360° (2π)
        if (accumulated_rotation >= 2 * M_PI) {
          RCLCPP_INFO(this->get_logger(), "Rotazione completa di 360°.");
          break;
        }

        rate.sleep();
      }

      // Ferma la base dopo la rotazione
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
      cmd_vel_pub_->publish(twist);
      cmd_vel_pub_->publish(twist);
      RCLCPP_INFO(this->get_logger(), "Rotazione terminata, robot fermo.");
      busy = true;
    }
    else if(sim_mode_){
      // TODO non corretto, da togliere???
      // Rotazione del giunto 0
      RCLCPP_INFO(this->get_logger(), "Simulazione");
      angle_deg = 0.0;
      busy = false;
      // ruota giunto 0 del manipolatore
      rotation_index = 0; // reset at start
      for (; rotation_index < MAX_ROTATION_STEPS; rotation_index++) {
        RCLCPP_INFO(this->get_logger(), "[Scan] Rotazione step %d/%d", rotation_index+1, MAX_ROTATION_STEPS);
        angle_deg = rotation_index * ROTATION_ANGLE_DEG;
        //menu_->oneJointMove(0, angle_deg); // Ruota il braccio di 15 gradi
        if (rotation_index <= MAX_ROTATION_STEPS/2){
          RCLCPP_INFO(this->get_logger(), "ruoto");
          bool success = moveJointAndWait(0, angle_deg, 2.0, 10.0);
          if (success) {
            busy = false;
            rclcpp::sleep_for(std::chrono::milliseconds(3000)); // Attendi che il robot visualizzi il sacco
            busy = true;
          }
        }
        else if(rotation_index > MAX_ROTATION_STEPS/2){
          bool success = moveJointAndWait(0, 180-angle_deg, 2.0, 10.0); // Ruota il braccio di 15 gradi
          if (success) {
            busy = false;
            rclcpp::sleep_for(std::chrono::milliseconds(3000)); // Attendi che il robot visualizzi il sacco
            busy = true;
          }
        }
        // Simula la scansione
        if (keypoint_arrived && !is_duplicate(latest_keypoint_relative_, 0.5))
        {
          RCLCPP_INFO(this->get_logger(), "Nuovo sacco rilevato in posizione x=%.2f y=%.2f z=%.2f",
                  latest_keypoint_world_.x, latest_keypoint_world_.y, latest_keypoint_world_.z);
          bag_buffer_.push_back(latest_keypoint_world_);
          publish_marker_at_keypoint(latest_keypoint_relative_, "bag", bag_buffer_.size(), true);
          keypoint_arrived = false; // Reset flag dopo aver processato il keypoint
          //break;
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Sacco già visto o non trovato, continuo a ruotare");
          keypoint_arrived = false; // Reset flag dopo aver processato il keypoint
        }
      }
      // Debug stampa bag_buffer_
      RCLCPP_INFO(this->get_logger(), "Buffer sacchi trovati:");
      int keypoint_index = 1;
      for (const auto &bag : bag_buffer_) {
        RCLCPP_INFO(this->get_logger(), "Sacco[%.d] in posizione x=%.2f y=%.2f z=%.2f",
                    keypoint_index, bag.x, bag.y, bag.z);
        publish_marker_at_keypoint(bag, "bag", keypoint_index, true); // Pubblica marker per ogni sacco trovato
        keypoint_index = keypoint_index + 1;
      }
      RCLCPP_INFO(this->get_logger(), "Numero sacchi trovati = %.d", keypoint_index-1);
    }
  }

  // Funzione per trovare il sacco più vicino
  geometry_msgs::msg::Point nearest_keypoint() {

    geometry_msgs::msg::Point empty_point;
    empty_point.x = 0.0;
    empty_point.y = 0.0;
    empty_point.z = 0.0;

    if (bag_buffer_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Nessun sacco rilevato.");
      return empty_point;
    }

    // Calcolo del più vicino
    min_distance = std::numeric_limits<double>::max();
    nearest_index = -1;

    for (size_t i = 0; i < bag_buffer_.size(); ++i) {
      double dx = bag_buffer_[i].x;
      double dy = bag_buffer_[i].y;
      double dz = bag_buffer_[i].z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (dist < min_distance) {
        min_distance = dist;
        nearest_index = static_cast<int>(i);
      }
    }

    if (nearest_index != -1) {
      RCLCPP_INFO(this->get_logger(),
        "Sacco più vicino: numero %d a distanza %.2f m (x=%.2f, y=%.2f, z=%.2f)",
        nearest_index + 1,
        min_distance,
        bag_buffer_[nearest_index].x,
        bag_buffer_[nearest_index].y,
        bag_buffer_[nearest_index].z
      );

      return bag_buffer_[nearest_index];
    }
    return empty_point;
  }

  // Timer per richiamare periodicamente la funzione di ricerca del sacco
  void create_search_timer() {
    if (search_timer_) search_timer_->cancel();
    search_timer_ = this->create_wall_timer(500ms, std::bind(&GraspNode::searchAndApproch, this));
  }

  void stop_search_timer() {
    if (search_timer_) {
      search_timer_->cancel();
      search_timer_.reset();
    }
  }

  // Timer per richiamare periodicamente la funzione di presa del sacco
  void create_grasp_timer() {
    if (grasp_timer_) grasp_timer_->cancel();
    grasp_timer_ = this->create_wall_timer(100ms, std::bind(&GraspNode::grasping, this));
  }

  void stop_grasp_timer() {
    if (grasp_timer_) {
      grasp_timer_->cancel();
      grasp_timer_.reset();
    }
  }

  // Funzione per ricerca keypoint in prossimità del sacco se questo non è ben centrato
  bool rotate_until_target (double angle_rad, double angular_speed = 0.3) {

    busy = false;
    keypoint_arrived = false;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;

    // Fase 1: ruota a sinistra di "angle_rad"

    double start_theta = base_theta_;

    while (std::fabs(base_theta_ - start_theta) < angle_rad) {
      if (keypoint_arrived) { 
        busy = true;
        RCLCPP_INFO(this->get_logger(), "Keypoint trovato durante rotazione a sinistra.");
        twist.angular.z = 0;
        cmd_vel_pub_->publish(twist); // stop
        return true;
      }
      twist.angular.z = + angular_speed;
      cmd_vel_pub_->publish(twist);
    }

    // Rotazione a destra di 2 angle_rad
    while (fabs(base_theta_ - start_theta) < 2 * angle_rad) {
      if (keypoint_arrived) {
        busy = true;
        RCLCPP_INFO(this->get_logger(), "Keypoint trovato durante rotazione a destra.");
        twist.angular.z = 0;
        cmd_vel_pub_->publish(twist); // stop
        return true;
      }
      twist.angular.z = - angular_speed;
      cmd_vel_pub_->publish(twist);
    }

    // Torna al centro
    while (fabs(base_theta_ - start_theta) > 0.01) {
      if (keypoint_arrived) {
        busy = true;
        RCLCPP_INFO(this->get_logger(), "Keypoint trovato durante rotazione a destra.");
        return true;
      }
      twist.angular.z = angular_speed;
      cmd_vel_pub_->publish(twist);
    }

    RCLCPP_WARN(this->get_logger(), "Keypoint non trovato dopo la scansione.");
    return false;
  }

  // Macchina a stati ricerca del sacco e avvicinamento
  void searchAndApproch() {
    switch (Sstate) {
      case SearchState::Idle:{
        RCLCPP_INFO(this->get_logger(), "Robot in Idle.");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        break;
      }
      case SearchState::InitialPose:{
        RCLCPP_INFO(this->get_logger(), "Robot in Initial Pose.");
        initial_scan_pose();
        Sstate = SearchState::ScanEnvironment;
        break;
      } 
      case SearchState::ScanEnvironment:{
        bag_buffer_.clear(); // Pulisci il buffer dei sacchi trovati
        rotate_and_scan();
        if (bag_buffer_.empty()) {
          RCLCPP_WARN(this->get_logger(), "Nessun sacco rilevato, riprovo a scansionare.");
          Sstate = SearchState::InitialPose;  // ripeti la scansione
        } else {
          delete_all_markers();
          target_pose.position = nearest_keypoint();
          busy = true;
          publish_marker_at_keypoint(target_pose.position, "nearest_bag_", nearest_index+1, true); // Pubblica marker per il sacco più vicino
          //add_collision_box(target_pose.position, nearest_index+1); // Aggiungi box di collisione per il sacco più vicino
          RCLCPP_INFO(this->get_logger(), "Sacco trovato in posizione x=%.2f y=%.2f z=%.2f",
                      target_pose.position.x, target_pose.position.y, target_pose.position.z);
          bag_buffer_.clear(); // Pulisci il buffer dei sacchi trovati
          Sstate = SearchState::ApproachTarget;
        }
        break;
      }
      case SearchState::ApproachTarget: {
        RCLCPP_DEBUG(this->get_logger(), "Avvicinamento al sacco...");
        // --- Fase 1: movimento a step ---
        if (!kp_control) {
          // Fai un piccolo passo verso il target
          move_base_towards(target_pose.position, 0.3, 1.0);  // esempio: 0.3 m per step
        } 
        // --- Fase 2: fermo → controllo consistenza del target ---
        else if (kp_control && alligned_with_target_) { 
          if (xy_is_similar(latest_keypoint_world_, target_pose.position, 0.3)) {
            // target confermato
            busy = true;
            consecutive_mismatches_ = 0;
            target_pose.position = latest_keypoint_world_; // aggiorna posizione
            delete_all_markers();
            publish_marker_at_keypoint(latest_keypoint_relative_, "nuovo_nearest_bag_", nearest_index+1, true);
            bag_buffer_.clear(); // Pulisci il buffer dei sacchi trovati
            kp_control = false;
            //keypoint_arrived = false;
            RCLCPP_INFO(this->get_logger(), "Target globale consistente: aggiornato a (%.2f, %.2f, %.2f)",
                        target_pose.position.x, target_pose.position.y, target_pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Target relativo consistente: aggiornato a (%.2f, %.2f, %.2f)",
                        latest_keypoint_relative_.x, latest_keypoint_relative_.y, latest_keypoint_relative_.z);
          } else {
            // target incoerente
            if (!keypoint_arrived && !rotate_until_target(0.2, 0.08)) {
              consecutive_mismatches_++;
              rclcpp::sleep_for(std::chrono::milliseconds(2000));
              RCLCPP_WARN(this->get_logger(), "Target incoerente (%d/%d)", consecutive_mismatches_, max_mismatches_);
              if (consecutive_mismatches_ >= max_mismatches_) {
                RCLCPP_ERROR(this->get_logger(),"Troppi mismatch: annullo target.");    
                kp_control = false;
                step_start_distance_ = -1.0;
                bag_buffer_.clear(); // Pulisci il buffer dei sacchi trovati
                Sstate = SearchState::ScanEnvironment;
                break;
              }
            }
          }
        }

        // Se il sacco è abbastanza vicino → passa a End
        if (grasping_) {
          RCLCPP_INFO(this->get_logger(), "Sacco in prossimità.");

          searching_ = false;
          busy = true;
          step_start_distance_ = -1.0;
          stop_search_timer(); 
          Gstate = GraspState::Approach;
          create_grasp_timer();
          bag_buffer_.clear(); // Pulisci il buffer dei sacchi trovati
          delete_all_markers();
          Sstate = SearchState::End;
          break;
        }
        Sstate = SearchState::ApproachTarget; // continua ad avvicinarsi

        break;
        
        case SearchState::End:{
          RCLCPP_INFO(this->get_logger(), "Fine routine di ricerca.");
          Sstate = SearchState::Idle;
          break;
        }
      }
    }
  }

  // Macchina a stati per presa e spostamento del sacco
  void grasping() {
    switch (Gstate) {
      // Stato di approccio all'oggetto
      case GraspState::Approach:{
        if (busy_grasp) break; // evita chiamate multiple
        
        if (!keypoint_arrived){
          rotate_until_target(0.35, 0.08);
          Gstate = GraspState::Approach;
          break;
        }
        
        busy_grasp = true;
       
        if(!process_received_keypoint(latest_keypoint_)){
          Gstate = GraspState::Approach;
          break;
        }

        busy_grasp= false;

        Gstate = GraspState::Grasp;
        break;
      }

      // Stato di presa dell'oggetto
      case GraspState::Grasp:{
        execute_grasp();
        Gstate = GraspState::Move;
        break;
      }

      // Stato di spostamento dell'oggetto
      case GraspState::Move:{
        rotate_base(90);
        Gstate = GraspState::Place;
        break;
      }

      // Stato di posizionamento dell'oggetto
      case GraspState::Place:{
        execute_place();
        Gstate = GraspState::Home;
        break;
      }
      
      // Stato di ritorno alla posizione Home
      case GraspState::Home:{
        execute_home();
        Gstate = GraspState::Idle;
        break;
      }

      // Stato di inattività
      case GraspState::Idle:{
        RCLCPP_INFO(this->get_logger(), "Robot in Idle.");
        rclcpp::sleep_for(std::chrono::milliseconds(5000));
        busy = false;
        keypoint_buffer_.clear();
        Gstate = GraspState::Approach;
        break;
      }

      default:{
        break;
      }
    }
  }


  void startRoutine() {
    busy = true;
    delete_all_markers();
    searching_ = false;
    grasping_ = false;
    
    if (!searching_) {
      Sstate = SearchState::InitialPose;
      searching_ = true;
      create_search_timer();
    }
    
  }

  void keypoint_callback(const geometry_msgs::msg::Point & msg) {

    RCLCPP_DEBUG(this->get_logger(), "Keypoint callback triggered.");
    if (busy) {
      RCLCPP_DEBUG(this->get_logger(), "Robot busy, keypoint ignored.");
      return; // Se il robot è occupato, ignora il nuovo keypoint
    } else {
      if (searching_ && !keypoint_arrived) {
        
        RCLCPP_DEBUG(this->get_logger(), "Robot in ricerca, keypoint accettato.");
        fill_keypoint_buffer(msg);
        
        if(are_keypoints_similar()){
          RCLCPP_DEBUG(this->get_logger(), "%ld keypoint di fila simili.", required_similar_keypoints_);

          keypoint_arrived = true;
          // Pose della base in mondo (Coppelia)
          xb = base_x_;
          yb = base_y_;
          th = base_theta_;
          zb = 0.0;   // 0.0 perché in rviz la base mobile non c'è
          
          // Punto relativo nel frame base_link senza rotazione
          xr = msg.x;
          yr = msg.y;
          zr = msg.z;

          c = std::cos(th);
          s = std::sin(th);

          // Trasformazione solo rotazione: posizione relativa alla base mobile 
          x_r = c*xr - s*yr;
          y_r = s*xr + c*yr;
          z_r = zr;
          geometry_msgs::msg::Point p_r;
          p_r.x = x_r;
          p_r.y = y_r;
          p_r.z = z_r;
          latest_keypoint_relative_ = p_r; // aggiorna latest_keypoint_world 
          // Trasformazione: mondo = T_world_base * p_rel
          x_w = xb + x_r; 
          y_w = yb + y_r;
          z_w = zb + z_r;
          geometry_msgs::msg::Point p_w;
          p_w.x = x_w;
          p_w.y = y_w;
          p_w.z = z_w;
          latest_keypoint_world_ = p_w; // aggiorna latest_keypoint_world 
          if (keypoint_arrived && filter_keypoint(latest_keypoint_relative_, bag_buffer_, 0.2, 0.2)){
            RCLCPP_INFO(this->get_logger(), "Keypoint_relative salvato: x=%.3f y=%.3f z=%.3f", latest_keypoint_relative_.x, latest_keypoint_relative_.y, latest_keypoint_relative_.z);
            publish_marker_at_keypoint(latest_keypoint_relative_, "keypoint_relativo_", bag_buffer_.size(), true);
            keypoint_arrived = false;
          } else {
            RCLCPP_DEBUG(this->get_logger(), "Keypoint duplicato, marker non pubblicato.");
            keypoint_arrived = false;
          }
        } 
      } 
      else if (grasping_ && !keypoint_arrived) {
        
        fill_keypoint_buffer(msg);
        if (are_keypoints_similar()){
          RCLCPP_DEBUG(this->get_logger(), "%ld keypoint di fila simili.", required_similar_keypoints_);
          latest_keypoint_ = msg;
          target_pose.position = latest_keypoint_;
          bag_buffer_.push_back(latest_keypoint_);
          publish_marker_at_keypoint(latest_keypoint_, "bag_keypoint_", bag_buffer_.size(), true);
          keypoint_arrived = true;
        }
      }
    }
  }
  
  void jointState_callback(const sensor_msgs::msg::JointState & msg) {
    // Aggiorna la posa corrente del manipolatore
    current_joint_pose_ = msg;
  }
  
  void basePoseCallback(const geometry_msgs::msg::Pose2D & msg)
  {
    // Debug stampa della posa base mobile
    //RCLCPP_INFO(this->get_logger(), "Base Pose: x=%.2f y=%.2f theta=%.2f", msg->x, msg->y, msg->theta);

    base_x_ = msg.x;
    base_y_ = msg.y;
    base_theta_ = normalizza_angolo(msg.theta);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspNode>();
  node->init();

  // Executor multithread per gestire più callback in parallelo
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

