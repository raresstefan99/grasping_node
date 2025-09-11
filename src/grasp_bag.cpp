#include "grasping_node/graspingUtils.h"

  // Funzione menu per gestire l'input dell'utente
  void input_menu(std::shared_ptr<GraspNode> node)
{
  std::cout << "\nPremi 'h'=home position, '0'=scan position 's'=start callback, 'e'=exit callback, '1'=process keypoint, '2'=execute grasp, '3'=execute place, 'q'=quit: ";
  char input;
  std::cin >> input;

  switch (input) {
    case 'h':
      RCLCPP_INFO(node->get_logger(), "Moving to home position...");
      node->execute_home();
      break;
    case '0':
      RCLCPP_INFO(node->get_logger(), "Moving to scan position...");
      node->initial_scan_pose();
      break;
    case 's':
      RCLCPP_INFO(node->get_logger(), "Start callback.");
      node->busy_keypoint_callback(false);
      node->grasping_ = true;
      break;
    case 'e':
      RCLCPP_INFO(node->get_logger(), "Exit callback.");
      node->busy_keypoint_callback(true);
      node->keypoint_arrived = false;
      node->grasping_ = false;
      node->delete_all_markers();
      node->keypoint_buffer_.clear();
      node->bag_buffer_.clear();
      break;
    case '1':
      node->process_received_keypoint();
      break;
    case '2':
      node->execute_grasp();
      break;
    case '3':
      node->execute_place();
      break;
    case 'q':
      RCLCPP_INFO(node->get_logger(), "Quit command received. Shutting down.");
      rclcpp::shutdown();
      break;
    default:
      std::cout << "Input non riconosciuto, riprova." << std::endl;
      break;
  }
}
  
  
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspNode>();
  node->setMenuMode(true);
  node->init();

  // Executor multithread
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Thread executor (gestisce ROS2)
  std::thread exec_thread([&executor]() {
    executor.spin();
  });

  // Thread menu (gestisce input utente)
  std::thread menu_thread([node]() {
    while (rclcpp::ok()) {
      input_menu(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  exec_thread.join();
  menu_thread.join();

  rclcpp::shutdown();
  return 0;
}