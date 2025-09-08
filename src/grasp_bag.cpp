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
    timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
    
    //Opzioni per la subscription
    rclcpp::SubscriptionOptions options;
    options.callback_group = keypoint_callback_group_;

    // Iscrizione al topic dei keypoint
    // Utilizza il callback group per evitare conflitti di accesso alle risorse condivise
    keypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/keypoint_data",
      rclcpp::QoS(10),
      std::bind(&GraspNode::keypoint_callback, this, _1),
      options);
    
    // Timer per input utente
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GraspNode::input_menu, this),
      timer_callback_group_);

    // Publisher del comando di presa e rilascio
    pub_grasp_control = this->create_publisher<std_msgs::msg::String>("/grasp_control", 10);
    // Publisher per il marker in RViz
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("keypoint_marker", 10);
  }

  // ========================================================= INIZIALIZZA MANIPOLATORE =========================================================
  void init() {
    ManipulatorMenuParams params;
    params.manipulator_name = "manipulator";
    params.planning_group = "ur_manipulator";
    params.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    params.base_link_name = "base_link_inertia";
    params.gripper = "robotiq_85";
    
    // Ricava il path del pacchetto manipulators
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("manipulators");
    params.known_poses_path = pkg_share_dir + "/config/known_poses.yaml";   

    menu_ = std::make_shared<ManipulatorMenu>(params, shared_from_this(), false);
  }

  // ========================================================= FUNZIONI PUBBLICHE DI GESTIONE DELLA ROUTINE =========================================================
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
  char getDecisionFlag() const {
    return decision_flag_;
  }

private:
  // ========================================================= VARIABILI PRIVATE  =========================================================

  std::shared_ptr<ManipulatorMenu> menu_; // Puntatore al menu del manipolatore
  rclcpp::CallbackGroup::SharedPtr keypoint_callback_group_; //  Gruppo di callback per gestire keypoint callback
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_; //  Gruppo di callback per gestire il menu
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr keypoint_sub_; // Iscrizione al topic dei keypoint
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_grasp_control; // Publisher per i comandi di presa e rilascio
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; // Publisher per i marker in RViz
  GraspState state = GraspState::Approach;  // Stato corrente della routine di presa
  std::atomic<int> flag;  // Flag per gestire l'avanzamento della routine
  bool callback_enabled_ = false;  // Flag per abilitare/disabilitare la callback del keypoint
  bool keypoint_arrived = false;  // Flag per verificare se il keypoint è stato ricevuto
  geometry_msgs::msg::Pose target_pose;  // Posa iniziale del robot
  std::atomic<char> decision_flag_{' '};  // ' ' = nessuna decisione - flag di approvazione del keypoint: 'c' = continua, 'r' = rigenera
  bool waiting_for_decision_ = false; // Flag per verificare se si sta aspettando la decisione sull'approvazione del keypoint
  rclcpp::TimerBase::SharedPtr timer_;

  // ========================================================= FUNZIONI PRIVATE  =========================================================
  // Funzione menu per gestire l'input dell'utente
  void input_menu()
  {

    std::cout << "\nPremi 'h'=home, 's'=start callback 'e'=exit callback, '1'=next step, 'q'=quit: ";
    char input;
    std::cin >> input;
    

    switch (input) {
      case 'h':
        // Inizializza la posa del robot nella posizione Home
        RCLCPP_INFO(this->get_logger(), "Moving to home position...");
        initial_robot_pose();
        break;
      case 's':
        // Abilita la callback per ricevere i keypoint
        RCLCPP_INFO(this->get_logger(), "Start callback.");
        enable_callback(true);
        break;
      case 'e':
        // Disattiva la callback per non ricevere più keypoint
        RCLCPP_INFO(this->get_logger(), "Exit callback.");
        enable_callback(false);
        keypoint_arrived = false; // Reset del flag per il keypoint
        waiting_for_decision_ = false; // Reset del flag di attesa decisione
        break;
      case '1':
        // Imposta il flag per avanzare alla prossima fase della routine
        set_flag(1);
        break;
      case 'q':
        // Chiudi il nodo e termina l'esecuzione
        RCLCPP_INFO(this->get_logger(), "Quit command received. Shutting down.");
        rclcpp::shutdown();
        break;
      case 'c':
        // Setta la decisione per la callback - c = continua
        setDecisionFlag('c');
        break;
      case 'r':
        // Setta la decisione per la callback - r = rigenera
        setDecisionFlag('r');
        break;
      default:
        std::cout << "Input non riconosciuto, riprova." << std::endl;
        break;
  }
}
  
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
    marker.header.frame_id = "base_link";
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

  void execute_grasp(){
    wait_for_user("Prendi oggetto"); 
    publish_grasp_command("grasp");
    menu_->moveGripper(true);
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Attendi chiusura gripper
    menu_->move_along_z(0.2, true); // Alza l'oggetto di  20 cm
  }

  void execute_move(){
    wait_for_user("Sposta oggetto");
    menu_->oneJointMove(0, 90); // Ruota il braccio di 90 gradi
  }

  void execute_place(){
    wait_for_user("Appoggia oggettto");
    menu_->move_along_z(-0.2, true);  // Abbassa l'oggetto di 20 cm
    rclcpp::sleep_for(std::chrono::milliseconds(4000));  // Attendi che l'oggetto venga posizionato
    publish_grasp_command("release");
    menu_->moveGripper(false);
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Attendi apertura gripper
    menu_->move_along_z(0.3, true);  // Alza il braccio di 30 cm dopo aver rilasciato l'oggetto
  }

  void execute_home(){
    wait_for_user("Ritorno a Home");
    menu_->publishJointGoal(menu_->getKnownPose("home_gripper_down"));
  }

  void depth_check(){
    if (target_pose.position.z < 0.1 || target_pose.position.z > 2.0) {
      RCLCPP_WARN(this->get_logger(), "Profondità anomala, punto scartato.");
      RCLCPP_WARN(this->get_logger(), "Rigenerazione del keypoint...");
      setDecisionFlag('r'); // Rigenera keypoint
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
    target_pose.position.x = -msg.y;
    target_pose.position.y = msg.x;
    target_pose.position.z = msg.z;
    target_pose.orientation = menu_->quaternion_from_euler(180, 0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Keypoint ricevuto: x=%.3f y=%.3f z=%.3f", msg.x, msg.y, msg.z);
    // Visualizza il marker in RViz
    publish_marker_at_keypoint(target_pose.position);
    keypoint_arrived = true;
  }

  bool wait_decision(){
    if (getDecisionFlag() != 'c' && getDecisionFlag() != 'r') {
      if (!waiting_for_decision_) {
        RCLCPP_INFO(this->get_logger(), "In attesa di input utente ('c' per continuare, 'r' per rigenerare)...");
         waiting_for_decision_ = true;
      }
      return true; // Indica che si sta aspettando una decisione
    }
    return false; // Indica che non si sta aspettando una decisione
  }

  bool keypoint_regeneration(){
    if (getDecisionFlag() == 'r') {
      RCLCPP_INFO(this->get_logger(), "Rigenerazione del keypoint...");
      keypoint_arrived = false; // Reset per rigenerare il keypoint
      setDecisionFlag(' '); // Reset della decisione
      waiting_for_decision_ = false; // Reset flag di attesa decisione
      return true; // Indica che il keypoint è stato rigenerato
    }
    return false; // Indica che non è stata richiesta la rigenerazione del keypoint
  }
  
// ========================================================= KEYPOINT CALLBACK  =========================================================
  // Callback per gestire i keypoint ricevuti
  void keypoint_callback(const geometry_msgs::msg::Point & msg) {
    if (!callback_enabled_){ 
      return;
    }

    switch (state) {
      // Stato di approccio all'oggetto
      case GraspState::Approach:{
        if(!keypoint_arrived){
          process_received_keypoint(msg);
        }

        // Controllo profondità - se troppo vicino o troppo lontano, scarta il keypoint
        depth_check();

        // Attendi conferma dell'utente
        if (wait_decision()) {
          return;
        }

        // Se l'utente decide di rigenerare il keypoint
        if (keypoint_regeneration()) {
          return;
        }
        
        decision_flag_ = ' ';  // Reset decisione per evitare conflitti successivi
        waiting_for_decision_ = false; // Reset flag di attesa decisione

        // Attendi l'input dell'utente per procedere con la pianificazione
        wait_for_user("Pianificazione e approccio all'oggetto");
        menu_->cartesianPlanExecuteAndWait({target_pose}, {}, "", 5);
    
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

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

