#include "grasping_node/graspingUtils.h"

// ========================== COSTRUTTORE ==========================
GraspNode::GraspNode() : Node("grasp_bag_node") {

    // ------------------------------------------------- PARAMETRI YAML -------------------------------------------------
    this->declare_parameter<bool>("menu_mode", false); 
    this->get_parameter("menu_mode", menu_mode_);

    this->declare_parameter<std::string>("base_frame", "base_link");
    this->get_parameter("base_frame", base_frame_);

    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->get_parameter("camera_frame", camera_frame_);

    this->declare_parameter<double>("orientation_tolerance", 0.01);
    this->get_parameter("orientation_tolerance", orientation_tolerance_);

    this->declare_parameter<std::vector<double>>("home_pose", {0., -90., 90., -90., -90., 180.});
    this->get_parameter("home_pose", home_pose_);

    this->declare_parameter<std::vector<double>>("scan_pose", {0., -80., 160., -80., 90., 180.});
    this->get_parameter("scan_pose", scan_pose_);

    this->declare_parameter<std::vector<double>>("grasping_pose", {0., -60., 40., 90., 90., 180.});
    this->get_parameter("grasping_pose", grasping_pose_);

    this->declare_parameter<bool>("neobotix_mpo_500", false);
    this->get_parameter("neobotix_mpo_500", neobotix_mpo_500_);

    this->declare_parameter<std::vector<double>>("box_down_pose", {-0.25, 0.0, -0.25, 0.0, 0.0, 0.0, 1.0});
    this->get_parameter("box_down_pose", box_down_pose_vector_);
    if (box_down_pose_vector_.size() == 7) {
        box_down_pose_.position.x = box_down_pose_vector_[0];
        box_down_pose_.position.y = box_down_pose_vector_[1];
        box_down_pose_.position.z = box_down_pose_vector_[2];
        box_down_pose_.orientation.x = box_down_pose_vector_[3];
        box_down_pose_.orientation.y = box_down_pose_vector_[4];
        box_down_pose_.orientation.z = box_down_pose_vector_[5];
        box_down_pose_.orientation.w = box_down_pose_vector_[6];
    } else {
        RCLCPP_WARN(this->get_logger(), "Parameter 'box_down' does not have 7 elements, using default values.");
        box_down_pose_.position.x = -0.25;
        box_down_pose_.position.y = 0.0;
        box_down_pose_.position.z = -0.25;
        box_down_pose_.orientation.x = 0.0;
        box_down_pose_.orientation.y = 0.0;
        box_down_pose_.orientation.z = 0.0;
        box_down_pose_.orientation.w = 1.0;
    }

    this->declare_parameter<std::vector<double>>("box_down_size", {1.0, 0.7, 0.5});
    this->get_parameter("box_down_size", box_down_size_);

    this->declare_parameter<std::vector<double>>("box_up_pose", {-0.5, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0});
    this->get_parameter("box_up_pose", box_up_pose_vector_);
    if (box_up_pose_vector_.size() == 7) {
        box_up_pose_.position.x = box_up_pose_vector_[0];
        box_up_pose_.position.y = box_up_pose_vector_[1];
        box_up_pose_.position.z = box_up_pose_vector_[2];
        box_up_pose_.orientation.x = box_up_pose_vector_[3];
        box_up_pose_.orientation.y = box_up_pose_vector_[4];
        box_up_pose_.orientation.z = box_up_pose_vector_[5];
        box_up_pose_.orientation.w = box_up_pose_vector_[6];
    } else {
        RCLCPP_WARN(this->get_logger(), "Parameter 'box_down' does not have 7 elements, using default values.");
        box_up_pose_.position.x = -0.5;
        box_up_pose_.position.y = 0.0;
        box_up_pose_.position.z = 0.2;
        box_up_pose_.orientation.x = 0.0;
        box_up_pose_.orientation.y = 0.0;
        box_up_pose_.orientation.z = 0.0;
        box_up_pose_.orientation.w = 1.0;
    }

    this->declare_parameter<std::vector<double>>("box_up_size", {0.5, 0.7, 0.4});
    this->get_parameter("box_ip_size", box_up_size_);


    // ------------------------------------------------- CALLBACK GROUPS -------------------------------------------------

    // Per evitare conflitti di accesso alle risorse condivise - Race Conditions
    // Callback group separati e iscrizione ai topic

    // KEYPOINT
    callback_group_keypoint_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // MutuallyExclusive = una callback alla volta
    
    rclcpp::SubscriptionOptions options_keypoint;
    options_keypoint.callback_group = callback_group_keypoint_;

    keypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/keypoint_data",
      rclcpp::QoS(1),
      std::bind(&GraspNode::keypoint_callback, this, _1),
      options_keypoint);

    // ODOMETRIA BASE MOBILE
    callback_group_base_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_base;
    options_base.callback_group = callback_group_base_;

    base_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
    "/base_pose2d", rclcpp::QoS(1),
    std::bind(&GraspNode::basePoseCallback, this, _1),
    options_base);

    // JOINT STATES
    callback_group_joint_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_joint;
    options_joint.callback_group = callback_group_joint_;

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(1),
      std::bind(&GraspNode::jointState_callback, this, _1),
      options_base);
    

    // ------------------------------------------------- PUBLISHERS -------------------------------------------------

    // Publisher del comando di presa e rilascio utile in simulazione
    pub_grasp_control = this->create_publisher<std_msgs::msg::String>("/grasp_control", 1);

    // Publisher per il marker in RViz
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("keypoint_marker", 1);
    
    // Publisher per i comandi di movimento della base mobile
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);


}


// ========================== INIZIALIZZATORE ==========================
void GraspNode::init() {
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
    
    if(neobotix_mpo_500_){
      
      menu_->addObj("box_down", 1, box_down_size_, box_down_pose_, 0);

      menu_->addObj("box_up", 1, box_up_size_, box_up_pose_, 0);
    }

    client_ = this->create_client<RobotiQGripperControl>("/ur_rtde/robotiq_gripper/command");

    // ------------------------------------------------- TIMER START ROUTINE -------------------------------------------------
    
    if(!menu_mode_){
        // Timer inizializzazione start routine
        init_timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          [this]() {
              this->startRoutine();
              this->init_timer_->cancel();
          }
        );
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Menu mode attivo, avvio menu...");
    }
}

// ========================== TIMERS E UTILITY ==========================
// Timer per richiamare periodicamente la funzione di ricerca del sacco
void GraspNode::create_search_timer() {
    if (search_timer_) search_timer_->cancel();
    search_timer_ = this->create_wall_timer(500ms, std::bind(&GraspNode::searchAndApproch, this));
  }

void GraspNode::stop_search_timer() {
    if (search_timer_) {
      search_timer_->cancel();
      search_timer_.reset();
    }
  }

  // Timer per richiamare periodicamente la funzione di presa del sacco
void GraspNode::create_grasp_timer() {
    if (grasp_timer_) grasp_timer_->cancel();
    grasp_timer_ = this->create_wall_timer(100ms, std::bind(&GraspNode::grasping, this));
  }

void GraspNode::stop_grasp_timer() {
    if (grasp_timer_) {
      grasp_timer_->cancel();
      grasp_timer_.reset();
    }
  }

void GraspNode::setMenuMode(bool flag) 
{ 
  menu_mode_ = flag; 
}


// ========================== ROUTINE ==========================
void GraspNode::startRoutine() {
    busy = true;
    delete_all_markers();
    searching_ = false;
    grasping_ = false;
    
    Sstate = SearchState::InitialPose;
    create_search_timer();
    
  }

void GraspNode::stopRoutine() {

    stop_search_timer();
    stop_grasp_timer();
    // stop twist
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);
    // stop manipulator
    if(menu_){
      menu_->stopTrajectory();
    }
}


// ========================== LOGICA DI CONTROLLO MANIPOLATORE ==========================
void GraspNode::execute_place(){
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    menu_->move_along_z(-0.3, true);  // Abbassa l'oggetto di 20 cm
    rclcpp::sleep_for(std::chrono::milliseconds(5000));  // Attendi che l'oggetto venga posizionato
    publish_grasp_command("release");
    // menu_->moveGripper(false);
    open();
    rclcpp::sleep_for(std::chrono::milliseconds(1000));  // Attendi apertura gripper
    //menu_->move_along_z(0.2, true);  // Alza il braccio di 30 cm dopo aver rilasciato l'oggetto
    moveJointsAndWait(grasping_pose_, 1.0, 15.0);
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
}

void GraspNode::execute_home(){
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    menu_->publishJointGoal(home_pose_);
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
}

void GraspNode::initial_scan_pose(){
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    moveJointsAndWait(scan_pose_, 1.0, 15.0);
    //menu_->cartesianPlanExecuteAndWait({menu_->pose_from_vector(scan_pose_)}, {}, "", 5);
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
}

void GraspNode::execute_grasp()
{
    rclcpp::sleep_for(std::chrono::milliseconds(5000)); // Attendi che il robot sia pronto
    publish_grasp_command("grasp");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    // menu_->moveGripper(true);
    close();
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Attendi chiusura gripper
    //menu_->move_along_z(0.3, true); // Alza l'oggetto di  30 cm
    moveJointsAndWait(grasping_pose_, 1.0, 15.0);
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
}


// ========================== IMPLEMENTAZIONE CALLBACKS ==========================
void GraspNode::keypoint_callback(const geometry_msgs::msg::Point & msg) {

    RCLCPP_DEBUG(this->get_logger(), "Keypoint callback triggered.");
    if (busy) {
      RCLCPP_DEBUG(this->get_logger(), "Robot busy, keypoint ignored.");
      return; // Se il robot è occupato, ignora il nuovo keypoint
    } else {
      if (searching_ && !keypoint_arrived) {
        
        RCLCPP_DEBUG(this->get_logger(), "Robot in ricerca, keypoint accettato.");
        //fill_keypoint_buffer(msg);
        
        //if(are_keypoints_similar()){
          RCLCPP_DEBUG(this->get_logger(), "%ld keypoint di fila simili.", required_similar_keypoints_);

          //keypoint_arrived = true;
          // Odometria della base in mondo (Coppelia)
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
          latest_keypoint_relative_ = p_r; // aggiorna latest_keypoint_relative_ 
          // Trasformazione: mondo = T_world_base * p_rel
          x_w = xb + x_r; 
          y_w = yb + y_r;
          z_w = zb + z_r;
          geometry_msgs::msg::Point p_w;
          p_w.x = x_w;
          p_w.y = y_w;
          p_w.z = z_w;
          latest_keypoint_world_ = p_w; // aggiorna latest_keypoint_world_

          if(cluster_keypoint(latest_keypoint_relative_, 5.0, 0.3, 0.2)){
            const auto &avg = keypoint_buffer_.back();

            if (keypoint_arrived && filter_keypoint(avg, bag_buffer_, 0.2, 0.2)){
              RCLCPP_INFO(this->get_logger(), "Keypoint_relative salvato: x=%.3f y=%.3f z=%.3f", keypoint_buffer_.back().x, keypoint_buffer_.back().y, keypoint_buffer_.back().z);
              publish_marker_at_keypoint(avg, "keypoint_relativo_", bag_buffer_.size(), true);
              keypoint_arrived = false;
            } else {
              RCLCPP_DEBUG(this->get_logger(), "Keypoint duplicato, marker non pubblicato.");
              keypoint_arrived = false;
            }
          } 
      } 
      else if (grasping_ && !keypoint_arrived) {

        fill_keypoint_buffer(msg);
        
        if(are_keypoints_similar()){
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

// Funzione per attivare la callback del keypoint
void GraspNode::busy_keypoint_callback(bool enable){ 
  busy = enable; 
}

void GraspNode::jointState_callback(const sensor_msgs::msg::JointState & msg) {
    // Aggiorna la posa corrente del manipolatore
    current_joint_pose_ = msg;
}
  
void GraspNode::basePoseCallback(const geometry_msgs::msg::Pose2D & msg)
{
    // Debug stampa della posa base mobile
    //RCLCPP_INFO(this->get_logger(), "Base Pose: x=%.2f y=%.2f theta=%.2f", msg->x, msg->y, msg->theta);

    base_x_ = msg.x;
    base_y_ = msg.y;
    base_theta_ = normalizza_angolo(msg.theta);
}


// ========================== CONTROLLO JOINTS ==========================
bool GraspNode::moveJointsAndWait(const std::vector<double> &joint_target_deg, double tolerance_deg, double timeout_sec) 
{
    if (joint_target_deg.size() < 6)
    {
      RCLCPP_ERROR(this->get_logger(), "Target joints vector must have 6 elements.");
      return false;
    } 
    else if (current_joint_pose_.position.empty()) 
    {
      RCLCPP_ERROR(this->get_logger(), "current_joint_pose_ è vuoto!");
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
bool GraspNode::moveJointAndWait(int num, double joint_rot, double tolerance_deg, double timeout_sec)
{
    if(num < 0 || num >= 6) {
      RCLCPP_ERROR(this->get_logger(), "Invalid joint number %d", num);
      return false;
    }
    if (current_joint_pose_.position.empty()) 
    {
      RCLCPP_ERROR(this->get_logger(), "current_joint_pose_ è vuoto!");
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

// ========================== LOGICA DI CONTROLLO BASE MOBILE ==========================
// Funzione per approccio della base mobile in direzione del sacco più vicino 
void GraspNode::move_base_towards(const geometry_msgs::msg::Point &new_point, double step, double min_distance) 
{
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
    double base_heading = normalizza_angolo(base_theta_);
    double yaw_error = normalizza_angolo(target_yaw - base_heading); 
    if (std::fabs(yaw_error) < 0.01) yaw_error = 0.0; // evita piccole oscillazioni

    // Parametri velocità
    double linear_speed  = 1.5;   // m/s
    double angular_speed = 0.5;   // rad/s
    double angular_kp = 0.8;       // guadagno proporzionale

    // --- Caso 1: Sono arrivato abbastanza vicino e allineato ---
    if (distance <= min_distance && std::fabs(yaw_error) <= 0.02) {
      RCLCPP_INFO(this->get_logger(), "Vicino al target (d=%.3f m). Prendo il sacco.", distance);
      alligned_with_target_ = true;
      grasping_ = true;
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
      return;
    }

    // --- Caso 2: Devo correggere orientamento ---
    if (std::fabs(yaw_error) > 0.02) {
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
      twist.linear.x = std::min(linear_speed * (forward + 0.1), linear_speed); //+0.1 per evitare di fermarsi siccome forward tende a zero
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

// Funzione per girare su se stesso e scanarizzare l'ambiente in cerca di oggetti
void GraspNode::rotate_and_scan()
{
  RCLCPP_INFO(this->get_logger(), "Inizio scansione...");

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

// Funzione per ricerca keypoint in prossimità del sacco se questo non è ben centrato
bool GraspNode::rotate_until_target (double angle_rad, double angular_speed) 
{

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

// Funzione per ruotare la base mobile di un angolo specificato in gradi
bool GraspNode::rotate_base(double deg)
{
    // Ruota base mobile fino a raggiungere l'angolo desiderato
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    double angular_speed = 0.5;   // rad/s
    double angular_kp = 1.2;       // guadagno proporzionale
    double target_angle = normalizza_angolo(base_theta_ - deg * M_PI / 180.0); // Converti gradi in radianti e normalizza
    double yaw_error = normalizza_angolo(target_angle - base_theta_);
    twist.angular.z = angular_kp * yaw_error;
    if (twist.angular.z > angular_speed) twist.angular.z = angular_speed;
    if (twist.angular.z < -angular_speed) twist.angular.z = -angular_speed;
    cmd_vel_pub_->publish(twist);

    if (std::abs(normalizza_angolo(base_theta_ - target_angle)) < 0.05) { // Tolleranza di 0.05 rad
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
      return true;
    }


    return false;
}

bool GraspNode::goToXY(double target_x, double target_y)
{
    rclcpp::Rate rate(10);  // 10 Hz

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    // Differenza posizione target - base
    double dx = target_x - base_x_;
    double dy = target_y - base_y_;
    double distance = std::sqrt(dx * dx + dy * dy);
    // inizializza step se non è già stato fatto

    // Angolo desiderato verso il target
    double target_yaw = std::atan2(dy, dx); 

    // Errore di orientamento
    double base_heading = normalizza_angolo(base_theta_);
    double yaw_error = normalizza_angolo(target_yaw - base_heading); 
    if (std::fabs(yaw_error) < 0.01) yaw_error = 0.0; // evita piccole oscillazioni

    // Parametri velocità
    double linear_speed  = 3.0;   // m/s
    double angular_speed = 0.5;   // rad/s
    double angular_kp = 0.5;       // guadagno proporzionale

    // --- Caso 1: Sono arrivato abbastanza vicino e allineato ---
    if (distance <= 0.10) {
      RCLCPP_INFO(this->get_logger(), "Vicino al target (d=%.3f m).", distance);
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
      return true;
    }
    // --- Caso 2: Devo correggere orientamento ---
     if (std::fabs(yaw_error) > 0.08){
      twist.linear.x = 0;
      twist.angular.z = angular_kp * yaw_error;
      if (twist.angular.z > angular_speed) twist.angular.z = angular_speed;
      if (twist.angular.z < -angular_speed) twist.angular.z = -angular_speed;
      twist.linear.x  = 0.0;
      RCLCPP_INFO(this->get_logger(), "Correzione orientamento: errore yaw=%.2f rad", yaw_error);
    }
    // --- Caso 3: Mi avvicino al target ---
    else if (distance > 0.10) {
      twist.linear.x = linear_speed;
      twist.angular.z = angular_kp * yaw_error; // piccola correzione in marcia
       RCLCPP_INFO(this->get_logger(), "Mi avvicino: distanza=%.2f m", distance);
    } 

    // Pubblica il comando di movimento
    cmd_vel_pub_->publish(twist);
    return false;
}

// ========================== LOGICA DI CONTROLLO GRIPPER ==========================

// Funzione per pubblicare il comando di presa o rilascio
// Questa funzione invia un messaggio al topic /grasp_control con il comando specificato
// Il comando può essere "grasp" per prendere l'oggetto o "release" per rilasciarlo
void GraspNode::publish_grasp_command(const std::string & command)
{
    std_msgs::msg::String msg;
    msg.data = command;
    pub_grasp_control->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Comando '%s' pubblicato.", command.c_str());
}
  
// ========================== GESTIONE DATI ==========================

// Funzione per aggiornare il target in prossimità del sacco e pianificare/eseguire la posa dritta, inclinata di 45 gradi o dall'alto
bool GraspNode::process_received_keypoint() 
{
    // Log info about received keypoint
    RCLCPP_INFO(this->get_logger(), "Keypoint ricevuto: x=%.3f y=%.3f z=%.3f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    for(int i = 0; i < 3; i++) {
      target_pose.orientation = menu_->quaternion_from_euler(-90.0 - 45 * i, 0.0, -90.0);
      auto traj_result = menu_->cartesianPlanAndWait({target_pose}, {}, "", 5);
        if(traj_result.success) {
            // Esegui solo se la pianificazione è reale
            menu_->executeAndWait(traj_result.trajectory, 5);
            return true; // successo, esci dal for
            RCLCPP_INFO(get_logger(), "Orientamento %f gradi raggiunto.", -90.0 - 45 * i);
        } else if(traj_result.error_code == manipulator_interfaces::msg::TrajectoryResult::SAME_POSITION) {
            RCLCPP_INFO(get_logger(), "Orientamento %d già raggiunto, passo al successivo", i);
            continue; // prova il prossimo orientamento
        }
        // fallimento reale → prova il prossimo
    return false; // nessuno dei tre orientamenti ha funzionato
    }

    RCLCPP_INFO(this->get_logger(), "Keypoint ricevuto ma presa non pianificata: x=%.3f y=%.3f z=%.3f xR=%.3f Ry=%.3f Rz=%.3f", 
      target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);

    return false;
}

// Funzione che prepara un buffer di 5 keypoint
void GraspNode::fill_keypoint_buffer(const geometry_msgs::msg::Point& msg)
{
    keypoint_buffer_.push_back(msg);
        if (keypoint_buffer_.size() > required_similar_keypoints_) {
            keypoint_buffer_.pop_front();
        }
}

  // Controllo che nel keypoint_buffer ricevo tot keypoint simili per confermare keypoint
  bool GraspNode::are_keypoints_similar()
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

// Funzione filtro per evitare keypoint duplicati per lo stesso sacco
bool GraspNode::filter_keypoint(const geometry_msgs::msg::Point& new_kp, 
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

// Funzione di normalizzazione angolo [-pi, pi]
double GraspNode::normalizza_angolo (double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

// Funzione per controllare se un nuovo keypoint è simile al precedente - utile per capire se ci si sta avvicinando verso un keypoint valido
bool GraspNode::xy_is_similar(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, double tolerance) 
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    //double dz = p1.z - p2.z;

    double distance = std::sqrt(dx*dx + dy*dy); // manca dz
    RCLCPP_INFO(this->get_logger(), "Controllo keypoint simili, distanza = %.3f tolleranza = %.3f", distance, tolerance);

    return (distance <= tolerance);
}

// Funzione per trovare il sacco più vicino
geometry_msgs::msg::Point GraspNode::nearest_keypoint() 
{

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

bool GraspNode::cluster_keypoint(const geometry_msgs::msg::Point &new_kp,const double angle_threshold, const double distance_threshold, const double z_threshold) 
{
    // cluster come variabile membro privata, non static
    const double angle_thr_rad = angle_threshold * M_PI / 180;   // da gradi a radianti

    double angle_new = std::atan2(new_kp.y, new_kp.x);
    double dist_new  = std::sqrt(new_kp.x*new_kp.x + new_kp.y*new_kp.y);

    for (auto it = clusters_.begin(); it != clusters_.end(); ++it) {
        auto &cluster = *it;
        const auto &ref = cluster.front();

        double angle_old = std::atan2(ref.y, ref.x);
        double dist_old  = std::sqrt(ref.x*ref.x + ref.y*ref.y);

        if (std::fabs(angle_new - angle_old) < angle_thr_rad &&
            std::fabs(dist_new - dist_old) < distance_threshold &&
            std::fabs(new_kp.z - ref.z) < z_threshold) {
            
            cluster.push_back(new_kp);

            if (cluster.size() >= required_similar_keypoints_) {
                geometry_msgs::msg::Point avg;
                avg.x = avg.y = avg.z = 0.0;
                for (const auto& p : cluster) {
                    avg.x += p.x;
                    avg.y += p.y;
                    avg.z += p.z;
                }
                avg.x /= cluster.size();
                avg.y /= cluster.size();
                avg.z /= cluster.size();

                keypoint_buffer_.push_back(avg);
                keypoint_arrived = true;
                
                clusters_.erase(it);
                return true;
            }
            return false;
        }
    }

    clusters_.push_back({new_kp});
    return false;
}


// ========================== MARKER / DEBUG ==========================

// Funzione per pubblicare un marker in RViz al keypoint ricevuto
// Questa funzione crea un marker di tipo SPHERE e lo pubblica sul topic /visualization_marker
void GraspNode::publish_marker_at_keypoint(const geometry_msgs::msg::Point &point, std::string name, int index, bool add) 
{
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

void GraspNode::delete_all_markers() 
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_pub_->publish(marker);
}

// ========================== MACCHINE A STATI ==========================

// Macchina a stati ricerca del sacco e avvicinamento
void GraspNode::searchAndApproch() 
{
    switch (Sstate) {
      case SearchState::Idle:{
        RCLCPP_INFO(this->get_logger(), "Robot in Idle.");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        break;
      }
      case SearchState::InitialPose:{
        RCLCPP_INFO(this->get_logger(), "Robot in Initial Pose.");
        delete_all_markers();
        initial_scan_pose();
        searching_ = true;
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
          searching_ = false;
          Sstate = SearchState::ApproachTarget;
        }
        break;
      }
      case SearchState::ApproachTarget: {
        RCLCPP_DEBUG(this->get_logger(), "Avvicinamento al sacco...");
        searching_ = true;
        // --- Fase 1: movimento a step ---
        if (!kp_control) {
          // Fai un piccolo passo verso il target
          move_base_towards(target_pose.position, 0.3, 0.7);  // esempio: 0.3 m per step
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
          busy = false;
          keypoint_arrived = false;
          step_start_distance_ = -1.0;
          bag_buffer_.clear(); // Pulisci il buffer dei sacchi trovati
          keypoint_buffer_.clear();
          delete_all_markers();
          stop_search_timer(); 
          rclcpp::sleep_for(std::chrono::milliseconds(3000));
          Gstate = GraspState::Approach;
          create_grasp_timer();
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
void GraspNode::grasping() {
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
       
        if(!process_received_keypoint()){
          rclcpp::sleep_for(std::chrono::milliseconds(3000));
          Gstate = GraspState::Approach;
          break;
        }
        keypoint_arrived = false;

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
        delete_all_markers();
        if(goToXY(0.0, 3.0)){ // Spostati in un punto B predefinito
          Gstate = GraspState::Place;
          break;
        }
        
        break;
      }

      // Stato di posizionamento dell'oggetto
      case GraspState::Place:{
        execute_place();
        initial_scan_pose();
        Gstate = GraspState::Home;
        break;
      }
      
      // Stato di ritorno alla posizione Home
      case GraspState::Home:{
        if(goToXY(0.0, 0.0)){ // Torna alla posizione iniziale
          Gstate = GraspState::Idle;
          break; 
          //if(!rotate_base(0.0)) break; // Allinea la base all'angolo 0
        }

        break;
      }

      // Stato di inattività
      case GraspState::Idle:{
        RCLCPP_INFO(this->get_logger(), "Robot in Idle.");
        rclcpp::sleep_for(std::chrono::milliseconds(2000));
        keypoint_buffer_.clear();
        bag_buffer_.clear();
        stop_grasp_timer();
        searching_ = true;
        grasping_ = false;
        Sstate = SearchState::InitialPose;
        create_search_timer();
        break;
      }

      default:{
        break;
      }
    }
}

void GraspNode::next_step_search() {
    switch(Sstate) {
        case SearchState::Idle: Sstate = SearchState::InitialPose; break;
        case SearchState::InitialPose: Sstate = SearchState::ScanEnvironment; break;
        case SearchState::ScanEnvironment: Sstate = SearchState::ApproachTarget; break;
        case SearchState::ApproachTarget: Sstate = SearchState::End; break;
        case SearchState::End: Sstate = SearchState::Idle; break;
    }
    // Chiama subito la funzione per eseguire lo stato aggiornato
    searchAndApproch();
}

void GraspNode::next_step_grasp() {
    switch(Gstate) {
        case GraspState::Idle: Gstate = GraspState::Approach; break;
        case GraspState::Approach: Gstate = GraspState::Grasp; break;
        case GraspState::Grasp: Gstate = GraspState::Move; break;
        case GraspState::Move: Gstate = GraspState::Place; break;
        case GraspState::Place: Gstate = GraspState::Home; break;
        case GraspState::Home: Gstate = GraspState::Idle; break;
    }
    grasping();
}

// Funzione per eseguire la presa
bool GraspNode::waitForService(const std::chrono::milliseconds & timeout)
    {
        while (!client_->wait_for_service(timeout))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }
        return true;
    }

bool GraspNode::command(int32_t position, int32_t speed, int32_t force)
{
    auto req = std::make_shared<RobotiQGripperControl::Request>();
    req->position = position;  // 0=closed, 100=open
    req->speed    = speed;     // 0..100
    req->force    = force;     // 0..100

    auto future = client_->async_send_request(req);

    // Block until we have a response
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        return false;
    }

    auto resp = future.get();
    if (!resp)
    {
        RCLCPP_ERROR(this->get_logger(), "Empty response.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "success=%s, status=%d",
                resp->success ? "true" : "false", resp->status);
    return resp->success;
}

bool GraspNode::open(int32_t speed, int32_t force)
{
    return command(100, speed, force);
}

bool GraspNode::close(int32_t speed, int32_t force)
  {
      return command(0, speed, force);
  }

// ========================== COSTRUTTORE MENU E FUNZIONI MENU ==========================

AutoGraspMenu::AutoGraspMenu(const std::string& title) : last_(-1), title_(title) {}

void AutoGraspMenu::addChoice(const std::string& description, std::function<void()> callback) {
    choices_.push_back({description, callback});
    last_ = choices_.size() - 1; // aggiorna last_ all’ultimo indice inserito
}

void AutoGraspMenu::addSection(const std::string &section_name, size_t start_index, size_t end_index) {
    sections_.push_back({section_name, start_index, end_index});
}

void AutoGraspMenu::spinnerMenu() {
    bool running = true;

    while (running) {
        std::cout << "\n===== " << title_ << " =====\n";

        // Stampa le sezioni e le scelte associate
        for (const auto &sec : sections_) {
            std::cout << "\n--- " << sec.name << " ---\n";
            for (size_t i = sec.start_index; i <= sec.end_index && i < choices_.size(); ++i) {
                std::cout << i + 1 << ") " << choices_[i].description << "\n";
            }
        }

        // Mostra eventuali scelte rimaste fuori dalle sezioni
        size_t last_sec_end = sections_.empty() ? 0 : sections_.back().end_index + 1;
        for (size_t i = last_sec_end; i < choices_.size(); ++i) {
            std::cout << i + 1 << ") " << choices_[i].description << "\n";
        }

        std::cout << "q) Quit\n";
        std::cout << "Scelta: ";

        char input;
        std::cin >> input;

        if (input == 'q') {
            running = false;
            std::cout << "Chiusura menu.\n";
        } else {
            int idx = input - '1';
            if (idx >= 0 && static_cast<size_t>(idx) < choices_.size()) {
                choices_[idx].callback();
            } else {
                std::cout << "Comando non valido, riprova.\n";
            }
        }
    }
}

