#include "grasping_node/graspingUtils.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GraspNode>();
    node->setMenuMode(true);  // Forza modalità menu
    node->init();  // inizializza menu_ e subscription

    // Start executor in background
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread exec_thread([&executor]() {
        executor.spin();
    });

    // Crea menu
    auto menu = std::make_shared<AutoGraspMenu>("Auto Grasp Menu");


    int section_start = 0;

    // --------------------
    // Sezione: Posizioni Predefinite
    // --------------------
    menu->addChoice("Vai in Home position", [node]() { node->execute_home(); });
    menu->addChoice("Vai in Scan position", [node]() { node->initial_scan_pose(); });
    menu->addSection("Posizioni Home", static_cast<size_t>(section_start), static_cast<size_t>(menu->last_));
    section_start = menu->last_ + 1;

    // --------------------
    // Sezione: Macchina a stati 
    // --------------------
    menu->addChoice("Avvia macchina a stati", [node]() { 
      node->searching_ = true;
      node->startRoutine();
    });
    menu->addChoice("Ferma macchina a stati", [node]() { 
      node->stopRoutine();
    });
    menu->addChoice("Avanza di 1 step", [node]() { 
      node->next_step_search();
    });
    menu->addSection("Macchina a Stati", static_cast<size_t>(section_start), static_cast<size_t>(menu->last_));
    section_start = menu->last_ + 1;

    // --------------------
    // Sezione: Uscita
    // --------------------
    menu->addSection("Uscita", static_cast<size_t>(section_start), static_cast<size_t>(menu->last_));

    // Avvia menu
    menu->spinnerMenu();


    // shutdown
    rclcpp::shutdown();
    exec_thread.join();

    return 0;
}
