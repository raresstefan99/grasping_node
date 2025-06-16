#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "manipulators/ManipulatorMenu.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <thread>
#include <atomic>

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
  GraspNode() : Node("grasp_bag_node") {
    // Per evitare conflitti di accesso alle risorse condivise - Race Conditions
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Gruppo di callback mutualmente esclusivo - risponde solo a una callback alla volta 
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    // Iscrizione al topic dei keypoint
    // Utilizza il callback group per evitare conflitti di accesso alle risorse condivise
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/keypoint_data",
      rclcpp::QoS(10),
      std::bind(&GraspNode::keypoint_callback, this, _1),
      options);
    
    // Publisher del comando di presa e rilascio
    pub_grasp_control = this->create_publisher<std_msgs::msg::String>("/grasp_control", 10);
    // Publisher per il marker in RViz
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("keypoint_marker", 10);
  }

  // Inizzializza il menu del manipolatore
  void init() {
    ManipulatorMenuParams params;
    params.manipulator_name = "manipulator";
    params.planning_group = "ur_manipulator";
    params.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    params.base_link_name = "base_link_inertia";
    params.known_poses_path = "/home/stefan/bag_catcher_ros2_ws/src/manipulators/manipulators/config/known_poses.yaml";
    params.gripper = "robotiq_85";

    menu_ = std::make_shared<ManipulatorMenu>(params, shared_from_this(), false);
  }

  // Funzione per gestire flag di avanzamento della routine
  void set_flag(int value){
    flag.store(value); 
  }
  // Funzione per attivare la callback del keypoint
  void enable_callback(bool enable){ 
    callback_enabled_ = enable; 
  }
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

  // Funzione setter per il flag di approvazione del keypoint
  // 'c' = continua, 'r' = rigenera, ' ' = nessuna decisione
  void setDecisionFlag(char flag){
    decision_flag_ = flag;
  }

  // Funzione getter per il flag di approvazione del keypoint
  // Questa funzione può essere chiamata per verificare lo stato della decisione dell'utente
  // Momentaneamente non viene utilizzata
  char getDecisionFlag() const {
    return decision_flag_;
  }


private:
  std::shared_ptr<ManipulatorMenu> menu_; // Puntatore al menu del manipolatore
  rclcpp::CallbackGroup::SharedPtr callback_group_; //  Gruppo di callback per gestire le risorse condivise
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_; // Iscrizione al topic dei keypoint
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_grasp_control; // Publisher per i comandi di presa e rilascio
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // Publisher per i marker in RViz
  GraspState state = GraspState::Approach;  // Stato corrente della routine di presa
  std::atomic<int> flag;  // Flag per gestire l'avanzamento della routine
  bool callback_enabled_ = false;  // Flag per abilitare/disabilitare la callback del keypoint
  bool keypoint_arrived = false;  // Flag per verificare se il keypoint è stato ricevuto
  geometry_msgs::msg::Pose initial_pose;  // Posa iniziale del robot
  std::atomic<char> decision_flag_{' '};  // ' ' = nessuna decisione - flag di approvazione del keypoint: 'c' = continua, 'r' = rigenera

  // Funzione per attendere l'input dell'utente
  void wait_for_user(const std::string& step_name) {
    RCLCPP_INFO(this->get_logger(), "Attesa comando utente per lo step: %s", step_name.c_str());
    while (rclcpp::ok() && flag.load() != 1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    flag.store(0);
  }

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
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";  // Adatta al tuo sistema di riferimento
    marker.header.stamp = this->now();
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

  // Funzione per chiedere all'utente se vuole continuare o rigenerare il keypoint
  // Questa funzione stampa un messaggio e attende l'input dell'utente
  // 'c' per continuare o 'r' per rigenerare il keypoint
  char ask_user_to_continue() {
    char response;
    std::cout << "Punto di presa ricevuto. Vuoi continuare? (c = continua, r = rigenera): ";
    std::cin >> response;
    return response;
  } 

  // Callback per gestire i keypoint ricevuti
  void keypoint_callback(const geometry_msgs::msg::Point & msg) {
    if (!callback_enabled_){ 
      return;
    }

    switch (state) {
      // Stato di approccio all'oggetto
      case GraspState::Approach:{
        if(!keypoint_arrived){
          //--------------------------------------------NOTA BENE-----------------------------------------------
        // msg riceve x y z corretti per CoppeliaSim (prendendo il manipolatore disteso come riferimento, x punta verso destra, y punta in avanti, z punta verso l'alto)
        // In Rviz però gli assi x e y non corrispondono a quelli di CoppeliaSim, ma avviene una rotazione di -90 gradi rispetto a z
        // Perciò:
        // x rviz = -y coppeliasim
        // y rviz = x coppeliasim
        //----------------------------------------------------------------------------------------------------
          initial_pose.position.x = -msg.y;
          initial_pose.position.y = msg.x;
          initial_pose.position.z = msg.z;
          initial_pose.orientation = menu_->quaternion_from_euler(180, 0.0, 0.0);
          RCLCPP_INFO(this->get_logger(), "Keypoint ricevuto: x=%.3f y=%.3f z=%.3f", msg.x, msg.y, msg.z);
          // Visualizza il marker in RViz
          publish_marker_at_keypoint(initial_pose.position);
          keypoint_arrived = true;
        }

        // Controllo profondità - se troppo vicino o troppo lontano, scarta il keypoint
        if (initial_pose.position.z < 0.1 || initial_pose.position.z > 2.0) {
          RCLCPP_WARN(this->get_logger(), "Profondità anomala, punto scartato.");
          RCLCPP_WARN(this->get_logger(), "Rigenerazione del keypoint...");
          decision_flag_ = 'r'; // Reset della decisione
        }

        // Attendi conferma dell'utente
        if (decision_flag_ != 'c' && decision_flag_ != 'r') {
          RCLCPP_INFO(this->get_logger(), "In attesa di input utente ('c' per continuare, 'r' per rigenerare)...");
          return; // Esce e rientrerà alla prossima callback
        }

        // Se l'utente decide di rigenerare il keypoint
        if (decision_flag_ == 'r') {
          RCLCPP_INFO(this->get_logger(), "Rigenerazione del keypoint...");
          keypoint_arrived = false; // Reset per rigenerare il keypoint
          decision_flag_ = ' '; // Reset della decisione
          return; // Esce e rientrerà alla prossima callback
        }
        
        decision_flag_ = ' ';  // Resetto la decisione per evitare conflitti successivi

        RCLCPP_INFO(this->get_logger(), "Approccio all’oggetto...");

        // Attendi l'input dell'utente per procedere con la pianificazione
        wait_for_user("Pianificazione e approccio all'oggetto");
        menu_->cartesianPlanExecuteAndWait({initial_pose}, {}, "", 5);
        state = GraspState::Grasp;
        break;
      }

      // Stato di presa dell'oggetto
      case GraspState::Grasp:{
        wait_for_user("Prendi oggetto"); 
        publish_grasp_command("grasp");
        menu_->moveGripper(true);
        rclcpp::sleep_for(std::chrono::milliseconds(500)); // Attendi chiusura gripper
        menu_->move_along_z(0.2, true); // Alza l'oggetto di  20 cm
        state = GraspState::Move;
        break;
      }

      // Stato di spostamento dell'oggetto
      case GraspState::Move:{
        wait_for_user("Sposta oggetto");
        menu_->oneJointMove(0, 90); // Ruota il braccio di 90 gradi
        state = GraspState::Place;
        break;
      }

      // Stato di posizionamento dell'oggetto
      case GraspState::Place:{
        wait_for_user("Appoggia oggettto");
        menu_->move_along_z(-0.2, true);  // Abbassa l'oggetto di 20 cm
        rclcpp::sleep_for(std::chrono::milliseconds(4000));  // Attendi che l'oggetto venga posizionato
        publish_grasp_command("release");
        menu_->moveGripper(false);
        rclcpp::sleep_for(std::chrono::milliseconds(500));  // Attendi apertura gripper
        menu_->move_along_z(0.3, true);  // Alza il braccio di 30 cm dopo aver rilasciato l'oggetto
        state = GraspState::Home;
        break;
      }
      
      // Stato di ritorno alla posizione Home
      case GraspState::Home:{
        wait_for_user("Ritorno a Home");
        menu_->publishJointGoal(menu_->getKnownPose("home_gripper_down"));
        state = GraspState::Idle;
        break;
      }

      // Stato di inattività
      case GraspState::Idle:{
        RCLCPP_INFO(this->get_logger(), "Robot in Idle.");
        break;
      }

      default:{
        break;
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

  // Avvia il menu del manipolatore in un thread separato
  std::thread input_thread([&]() {
    char input;
    while (rclcpp::ok()) {
      std::cout << "\nPremi 'h'=home, 's'=start callback, '1'=next step, 'q'=quit: ";
      std::cin >> input;
      if (input == 'h') {
        node->initial_robot_pose(); // Inizializza la posa del robot nella posizione Home
      } else if (input == 's') {
        node->enable_callback(true);  // Abilita la callback per ricevere i keypoint
      } else if (input == '1') {
        node->set_flag(1);   // Imposta il flag per avanzare alla prossima fase della routine
      } else if (input == 'q') {
          rclcpp::shutdown(); // Chiudi il nodo e termina l'esecuzione
          break;
      } else if (input == 'c') {
          node->setDecisionFlag('c');  // ✅ Setta la decisione per la callback
      } else if (input == 'r') {
          node->setDecisionFlag('r');  // ❌ Rifiuta la decisione per la callback
      } else {
          std::cout << "Input non riconosciuto, riprova." << std::endl;
      }
    }
  });

  executor.spin();
  input_thread.join();

  rclcpp::shutdown();
  return 0;
}

