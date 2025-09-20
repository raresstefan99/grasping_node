#include "grasping_node/graspingUtils.h"

// Flag globali per la gestione
std::atomic<bool> auto_running_(false);
std::atomic<bool> auto_paused_(false);
std::thread auto_thread;
bool emergency_stop_ = false;

void stopBaseMotion(std::shared_ptr<GraspNode> node){
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  node->cmd_vel_pub_->publish(twist);
}

void stop_all_motions(std::shared_ptr<GraspNode> node)
{
    RCLCPP_WARN(node->get_logger(), "Stopping all motions...");

    // Stop callback keypoint/grasping
    node->grasping_ = false;
    node->searching_ = false;
    node->keypoint_arrived = false;
    node->busy_keypoint_callback(true);

    // Fermare il braccio
    if (node->menu_)
    {
        try
        {
            node->menu_->stopTrajectory(); 
            RCLCPP_INFO(node->get_logger(), "Braccio fermato.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Errore nel fermare il braccio: %s", e.what());
        }
    }

    // Fermare il gripper
    try
    {
        node->open(0, 0);  // posizione attuale, velocità 0 => ferma
        RCLCPP_INFO(node->get_logger(), "Gripper fermato.");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Errore nel fermare il gripper: %s", e.what());
    }

    // Fermare la base mobile
    try
    {
        // Assumendo che tu abbia move_base_towards o simili:
        stopBaseMotion(node); 
        RCLCPP_INFO(node->get_logger(), "Base mobile fermata.");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Errore nel fermare la base: %s", e.what());
    }

    RCLCPP_WARN(node->get_logger(), "Tutte le motion fermate.");
}

void run_auto_routine(std::shared_ptr<GraspNode> node)
{
  while (auto_running_) {
    while (auto_paused_ && auto_running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 1: Going Scan position...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->initial_scan_pose();
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 2: Rotate & Search bag...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->delete_all_markers();
    node->busy_keypoint_callback(false);
    node->keypoint_arrived = false;
    node->searching_ = true;
    node->rotate_and_scan(90);

    if (node->bag_buffer_.empty()) {
      RCLCPP_WARN(node->get_logger(), "Nessun sacco rilevato!");
      auto_running_ = false;
      break;
    }
    node->target_pose.position = node->nearest_keypoint();
    node->publish_marker_at_keypoint(node->target_pose.position, "nearest_bag_", node->nearest_index+1, true);
    node->bag_buffer_.clear();
    node->searching_ = false;

    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 3: Moving base towards bag...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    while (auto_running_ && node->grasping_ == false) {
      node->move_base_towards(node->target_pose.position, 0.3, 0.6);
      if (auto_paused_) {
        RCLCPP_WARN(node->get_logger(), "Routine in pausa...");
        while (auto_paused_ && auto_running_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(node->get_logger(), "Routine ripresa.");
      }
    }
    if (!auto_running_) break;


    RCLCPP_INFO(node->get_logger(), "Step 4: Grasp position...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->grasp_pose();
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 5: Detect keypoint...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->delete_all_markers();
    node->busy_keypoint_callback(false);
    node->grasping_ = true;
    node->keypoint_arrived = false;
    // wait
    while (node->keypoint_arrived == false){
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO(node->get_logger(), "wait keypoint...");
    }
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 6: Align with keypoint...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->alignWithKeypoint(node->target_pose, 0.01);
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 7: Going down until contact...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->goDownUntilForce(0.0001, 0.1, true);
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 8: Execute grasp...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->execute_grasp();
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 9: Go to xy...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    while(node->goToXY(0, 1) == false){
          if(emergency_stop_) break;
      }
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 10: Execute place...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    node->execute_place();
    if (!auto_running_) break;

    RCLCPP_INFO(node->get_logger(), "Step 11: Return to Initial pose...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    while(node->goToXY(0, 0) == false){
          if(emergency_stop_) break;
      }

    RCLCPP_INFO(node->get_logger(), "Routine automatica completata.");
    auto_running_ = false;  // resetta flag per evitare loop infiniti
  }
}

// Funzione menu per gestire l'input dell'utente
void input_menu(std::shared_ptr<GraspNode> node, std::atomic<bool>& running)
{
  std::cout << "\nPremi:";
  std::cout << "\n'h'=home position, 'j'=scan position, 'k'=grasp position";
  std::cout << "\n'o'=start grasp callback, 'p'=exit grasp callback";
  std::cout << "\n'v'=rotate/search bag, 'b'=go towards bag, 'n'=go to xy, 'm'=initial pose";
  std::cout << "\n'0'=process keypoint, '1'=align with keypoint, '2'=go down, '3'=execute grasp, '4'=execute place";
  std::cout << "\n'5'=open gripper, '6'=close gripper";
  std::cout << "\n'w'=step forward TCP, 's'=step back TCP, 'a'=step left TCP, 'd'=step right TCP, 'r'=step up TCP, 'f'=step down TCP";
  std::cout << "\n'z'=EMERGENCY STOP for auto routine, 'x'=Start auto routine";
  std::cout << "\n'q'=quit\n";

  char input;
  std::cin >> input;

  switch (input) {
    case 'h':
      RCLCPP_INFO(node->get_logger(), "Moving to home position...");
      node->execute_home();
      break;
    case 'j':
      RCLCPP_INFO(node->get_logger(), "Moving to scan position...");
      node->initial_scan_pose();
      break;
    case 'k':
      RCLCPP_INFO(node->get_logger(), "Moving to grasp position...");
      node->grasp_pose();
      break;
    case 'o':
      RCLCPP_INFO(node->get_logger(), "Start callback grasping.");
      node->delete_all_markers();
      node->busy_keypoint_callback(false);
      node->grasping_ = true;
      node->keypoint_arrived = false;
      break;
    case 'p':
      RCLCPP_INFO(node->get_logger(), "Exit callback grasping.");
      node->busy_keypoint_callback(true);
      node->keypoint_arrived = false;
      node->grasping_ = false;
      node->searching_ = false;
      node->delete_all_markers();
      node->keypoint_buffer_.clear();
      node->bag_buffer_.clear();
      break;
    case 'v':
      RCLCPP_INFO(node->get_logger(), "Rotate and Search bag.");
      node->delete_all_markers();
      node->busy_keypoint_callback(false);
      node->keypoint_arrived = false;
      node->grasping_ = false;
      node->searching_ = true;
      node->keypoint_buffer_.clear();
      node->bag_buffer_.clear();
      node->rotate_and_scan(90);

      if (node->bag_buffer_.empty()) {
        RCLCPP_WARN(node->get_logger(), "Nessun sacco rilevato.");
        break;
      } else {
        node->delete_all_markers();
        node->busy_keypoint_callback(true);
        node->target_pose.position = node->nearest_keypoint();
        node->publish_marker_at_keypoint(node->target_pose.position,
                                        "nearest_bag_", node->nearest_index+1, true);
        node->bag_buffer_.clear();
        node->searching_ = false;
      }
      break;
    case 'b':
      RCLCPP_INFO(node->get_logger(), "Go towards the bag.");
      node->busy_keypoint_callback(false);
      node->keypoint_arrived = false;
      node->grasping_ = false;
      node->searching_ = true;
      while(node->grasping_ == false){
        if(emergency_stop_) break;  // stop immediato
        node->move_base_towards(node->target_pose.position, 0.3, 0.7);
      }
      node->busy_keypoint_callback(true);
      node->keypoint_buffer_.clear();
      node->bag_buffer_.clear();
      break;
    case 'n':
      RCLCPP_INFO(node->get_logger(), "Go to xy (0,1).");
      while(node->goToXY(0, 1) == false){
          if(emergency_stop_) break;
      }
      break;
    case 'm':
      RCLCPP_INFO(node->get_logger(), "Go to initial pose (0,0).");
      while(node->goToXY(0, 0) == false){
          if(emergency_stop_) break;
      }
      break;
    case '0': node->process_received_keypoint(); break;
    case '1': node->alignWithKeypoint(node->target_pose, 0.05); break;
    case '2': node->goDownUntilForce(0.0001, 0.1, true); break;
    case '3': node->execute_grasp(); break;
    case '4': node->execute_place(); break;
    case '5': node->open(); break;
    case '6': node->close(); break;

    // STEP-BY-STEP TCP
    case 'r': node->menu_->move_along_z(0.01, true); break;  // up
    case 'f': node->menu_->move_along_z(-0.01, true); break; // down
    case 'a': node->menu_->move_along_y(0.01, true); break;  // left
    case 'd': node->menu_->move_along_y(-0.01, true); break; // right
    case 'w': node->menu_->move_along_x(0.01, true); break; // forward
    case 's': node->menu_->move_along_x(-0.01, true); break; // back

    // EMERGENCY STOP
    case 'z':
      // termina la routine automatica e ferma tutto
      auto_running_.store(false);
      auto_paused_.store(false);
      emergency_stop_ = true;
      stop_all_motions(node);
      RCLCPP_WARN(node->get_logger(), "EMERGENCY STOP attivato! Routine terminata.");
      break;

    // START routine (clear emergency)
    case 'x':
      if (!auto_running_) {
        emergency_stop_ = false;
        auto_running_.store(true);
        auto_paused_.store(false);
        auto_thread = std::thread(run_auto_routine, node);
        auto_thread.detach();
      }
      break;

    case 'q': // Uscita
      auto_running_ = false;
      auto_paused_ = false;
      stop_all_motions(node);
      RCLCPP_INFO(node->get_logger(), "Chiusura menu.");
      running = false;
      rclcpp::shutdown();
      return;

    default:
      RCLCPP_WARN(node->get_logger(), "Comando non riconosciuto!");
      break;
  }
}

  
  
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspNode>();
  node->setMenuMode(true);
  node->setSimMode(true);
  node->init();

  // Executor multithread
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Thread executor (gestisce ROS2)
  std::thread exec_thread([&executor]() {
    executor.spin();
  });

  std::atomic<bool> running(true);

  // Thread menu (gestisce input utente)
  std::thread menu_thread([node, &running]() {
    while (rclcpp::ok() && running) {
      input_menu(node, running);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  exec_thread.join();
  menu_thread.join();

  rclcpp::shutdown();
  return 0;
}