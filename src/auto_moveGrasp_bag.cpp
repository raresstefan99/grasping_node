#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "manipulators/ManipulatorMenu.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <thread>
#include <atomic>
#include "ament_index_cpp/get_package_share_directory.hpp"
//#include "moveit_msgs/msg/collision_object.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include "shape_msgs/msg/solid_primitive.hpp"

using std::placeholders::_1;

enum class GraspState {
  Idle,
  Approach,
  Grasp,
  Move,
  Place,
  Home
};

class GraspNode : public rclcpp::Node {
public:

  // ========================================================= COSTRUTTORE =========================================================
  GraspNode() : Node("grasp_bag_node") {
    // Per evitare conflitti di accesso alle risorse condivise - Race Conditions
    // Callback separati per evitare che la ricezione dei keypoint interferisca con l'input utente
    // Creazione dei gruppi
    keypoint_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // MutuallyExclusive per evitare conflitti tra callback
    
    //Opzioni per la subscription
    rclcpp::SubscriptionOptions options;
    options.callback_group = keypoint_callback_group_;

    // Iscrizione al topic dei keypoint
    // Utilizza il callback group per evitare conflitti di accesso alle risorse condivise
    keypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/keypoint_data",
      rclcpp::QoS(1),
      std::bind(&GraspNode::keypoint_callback, this, _1),
      options);

    // Publisher del comando di presa e rilascio
    pub_grasp_control = this->create_publisher<std_msgs::msg::String>("/grasp_control", 1);

    // Publisher per il marker in RViz
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("keypoint_marker", 1);
    
    // Timer inizializzazione per la home
    // ATTENZIONE - COMMENTARE LE SEGUENTI RIGHE SE NON SI E' IN AMBIENTE DI SIMULAZIONE
    init_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            this->initial_robot_pose();
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
    params.gripper = "robotiq_85";
    
    // Ricava il path del pacchetto manipulators
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("manipulators");
    params.known_poses_path = pkg_share_dir + "/config/known_poses.yaml";   

    menu_ = std::make_shared<ManipulatorMenu>(params, shared_from_this(), false);
  }

  // ========================================================= FUNZIONI PUBBLICHE =========================================================

  // Funzione per gestire lo stato della routine
  void set_state(GraspState new_state){ 
    state = new_state; 
  }

  // Funzione per inizializzare la posa del robot
  // Questa funzione imposta la posa iniziale del robot utilizzando una posa nota
  void initial_robot_pose() {
    if (!menu_) return;
    auto joints = menu_->getKnownPose("home_gripper_down"); // joints è di tipo std::vector<double>
    if (joints.empty()) return;
    menu_->publishJointGoal(joints);
  }

private:
  // ========================================================= VARIABILI PRIVATE  =========================================================

  std::shared_ptr<ManipulatorMenu> menu_; // Puntatore al menu del manipolatore
  rclcpp::CallbackGroup::SharedPtr keypoint_callback_group_; //  Gruppo di callback per gestire keypoint callback
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr keypoint_sub_; // Iscrizione al topic dei keypoint
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_grasp_control; // Publisher per i comandi di presa e rilascio
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // Publisher per i marker in RViz
  GraspState state = GraspState::Approach;  // Stato corrente della routine di presa
  bool keypoint_arrived = false;  // Flag per verificare se il keypoint è stato ricevuto
  geometry_msgs::msg::Pose target_pose;  // Posa iniziale del robot
  std::vector<geometry_msgs::msg::Point> keypoint_buffer_;
  const std::size_t required_similar_keypoints_ = 5; // std::size_t perché confrontato con il buffer che è di tipo std::size_t
  const double position_tolerance_ = 0.01; // 1 cm di tolleranza
  // TODO orientation_tolerance 0.01 radianti
  bool busy = false;
  rclcpp::TimerBase::SharedPtr init_timer_;

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
  void publish_marker_at_keypoint(const geometry_msgs::msg::Point &point) {
    //TODO aggiungere variabile true false per add o delete
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    //TODO 3 sacchi? trova quello più vicino
    // TODO macchina a stati per avvicinarmi alla distanza giusta e iterazione modi di presa
    marker.ns = "keypoint_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration::from_seconds(10);  // 10 secondi
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

  void execute_move(){
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    menu_->oneJointMove(0, 90); // Ruota il braccio di 90 gradi
    //TODO gira il robot base mobile e non giunto 0 - info da mettere in file yaml e distinguere sim true o sim false
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

  void depth_check(){
    if (target_pose.position.z < 0.2 || target_pose.position.z > 2.0) {
      RCLCPP_WARN(this->get_logger(), "Profondità anomala, punto scartato.");
      RCLCPP_WARN(this->get_logger(), "Rigenerazione del keypoint...");
      keypoint_buffer_.clear();
    }
  }

  void process_received_keypoint(const geometry_msgs::msg::Point& msg) {
    //--------------------------------------------NOTA BENE-----------------------------------------------
    // msg riceve x y z corretti per CoppeliaSim (prendendo il manipolatore disteso come riferimento, x punta verso destra, y punta in avanti, z punta verso l'alto)
    // In Rviz però gli assi x e y non corrispondono a quelli di CoppeliaSim, ma avviene una rotazione di -90 gradi rispetto a z
    // Perciò:
    // x rviz = -y coppeliasim
    // y rviz = x coppeliasim
    //----------------------------------------------------------------------------------------------------
    //target_pose_ globale mettre _
    target_pose.position.x = -msg.y;
    target_pose.position.y = msg.x;
    target_pose.position.z = msg.z;
    target_pose.orientation = menu_->quaternion_from_euler(180, 0.0, 0.0);
    //TODO avere altri 2 orientation possibili dall'alto, frontale 180 0 90 e 90 0 90
    RCLCPP_INFO(this->get_logger(), "Keypoint ricevuto: x=%.3f y=%.3f z=%.3f", msg.x, msg.y, msg.z);
    // Visualizza il marker in RViz
    publish_marker_at_keypoint(target_pose.position);
    keypoint_arrived = true;
  }
  
  bool keypoint_control(const geometry_msgs::msg::Point& msg){

    keypoint_buffer_.push_back(msg);
    if (keypoint_buffer_.size() == required_similar_keypoints_) {
      return true; 
    }
    else if (keypoint_buffer_.size() > required_similar_keypoints_)
    {
      RCLCPP_WARN(this->get_logger(), "Buffer di keypoint pieno, rimuovo il più vecchio.");
      keypoint_buffer_.erase(keypoint_buffer_.begin());
      return true; 
    }

    return false;
  }

  bool are_keypoints_similar()
    {
        if (keypoint_buffer_.size() < required_similar_keypoints_){
          RCLCPP_WARN(this->get_logger(), "Numero di keypoint ricevuti insufficiente: %zu, atteso: %ld", keypoint_buffer_.size(), required_similar_keypoints_);
          return false;
        }

        const auto& ref = keypoint_buffer_.front();
        for (const auto& kp : keypoint_buffer_)
        {
          //TODO interessante per capire se la bag è sempre la stessa durante le misurazioni mentre mi muovo verso la bag
          //se la differenza di posizione cambia di tanto significa che quello di prima era un falso positivo
            if (std::fabs(kp.x - ref.x) > position_tolerance_ ||
                std::fabs(kp.y - ref.y) > position_tolerance_ ||
                std::fabs(kp.z - ref.z) > position_tolerance_)
            {
              RCLCPP_WARN(this->get_logger(), "Keypoint fuori tolleranza.");
              return false;
            }
        }
        return true;
    }

  void add_collision_box(const geometry_msgs::msg::Point &center)
{
  // Crea il messaggio per l'oggetto di collisione
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "bag_box";

  // Definisci la forma: BOX
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  std::vector<double> box_size = {0.2, 0.2, 0.2};  // LxWxH in metri
  if (box_size.size() != 3) {
    RCLCPP_WARN(this->get_logger(), "Dimensioni non valide per il box");
    return;
  }
  primitive.dimensions.assign(box_size.begin(), box_size.end());

  // Definisci la posa
  geometry_msgs::msg::Pose box_pose;
  box_pose.position = center;
  box_pose.orientation.w = 1.0;  // orientamento neutro

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Crea il publisher temporaneo
  static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects({collision_object});

  RCLCPP_INFO(this->get_logger(), "Box di collisione aggiunto alla scena.");
}

// ========================================================= KEYPOINT CALLBACK  =========================================================
  // Callback per gestire i keypoint ricevuti
  void keypoint_callback(const geometry_msgs::msg::Point & msg) {
    if(!busy){
      if (!keypoint_control(msg)){ 
        return;
      }
    }

    if (are_keypoints_similar()){
      busy = true;
      switch (state) {
        // Stato di approccio all'oggetto
        case GraspState::Approach:{
          if(!keypoint_arrived){
            process_received_keypoint(msg);
          }

          // Controllo profondità - se troppo vicino o troppo lontano, scarta il keypoint
          depth_check();    

          // Procedi con la pianificazione
          menu_->cartesianPlanExecuteAndWait({target_pose}, {}, 5, 5);
    
          state = GraspState::Grasp;
        
          break;
        }

        // Stato di presa dell'oggetto
        case GraspState::Grasp:{
          execute_grasp();
          state = GraspState::Move;
          break;
        }

        // Stato di spostamento dell'oggetto
        case GraspState::Move:{
          execute_move();
          state = GraspState::Place;
          break;
        }

        // Stato di posizionamento dell'oggetto
        case GraspState::Place:{
          execute_place();
          state = GraspState::Home;
          break;
        }
      
        // Stato di ritorno alla posizione Home
        case GraspState::Home:{
          execute_home();
          state = GraspState::Idle;
          break;
        }

        // Stato di inattività
        case GraspState::Idle:{
          RCLCPP_INFO(this->get_logger(), "Robot in Idle.");
          rclcpp::sleep_for(std::chrono::milliseconds(5000));
          busy = false;
          keypoint_buffer_.clear();
          state = GraspState::Approach;
          break;
        }

        default:{
          break;
        }
      }
    }
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

