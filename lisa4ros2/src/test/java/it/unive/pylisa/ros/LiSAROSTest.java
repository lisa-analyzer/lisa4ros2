package it.unive.pylisa.ros;

import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.ROSApplication;
import it.unive.ros.application.RosApplicationBuilder;
import org.junit.Test;

public class LiSAROSTest {

    public void printInfo(ROSApplication r, String projName) {
        long loc = r.getLinesOfCode();
        int nodes = r.getRosNetwork().getNetworkEntityContainers().size();
        String topics = r.getRosNetwork().getTopics().size()  +" (" + r.getRosNetwork().getSystemTopics().size() + ")";
        String services = r.getRosNetwork().getServices().size() + " (" + r.getRosNetwork().getSystemServices().size() + ")";
        int actions = r.getRosNetwork().getActions().size();
        String publishers = r.getRosNetwork().getPublishers().size()  +" (" + r.getRosNetwork().getSystemPublishers().size() + ")";
        String subscriptions = r.getRosNetwork().getSubscribers().size()  +" (" + r.getRosNetwork().getSystemSubscribers().size() + ")";
        String serviceServers = r.getRosNetwork().getServiceServers().size()  +" (" + r.getRosNetwork().getSystemServiceServers().size() + ")";
        int serviceClients = r.getRosNetwork().getServiceClients().size();
        int actionServers = r.getRosNetwork().getActionServers().size();
        int actionClients = r.getRosNetwork().getActionClients().size();
        System.out.println("LOC: " + loc);
        System.out.println("N: " + nodes);
        System.out.println("T: " + topics);
        System.out.println("S: " + services);
        System.out.println("A: " + actions);
        System.out.println("PUB: " + publishers);
        System.out.println("SUB: " + subscriptions);
        System.out.println("SS: " + serviceServers);
        System.out.println("SC: " + serviceClients);
        System.out.println("AS: " + actionServers);
        System.out.println("AC: " + actionClients);
        System.out.println("\\textbf{" + projName + "} & " + loc + " & " + nodes +" & " + topics + " & " + services + " & " + actions + " & " + publishers + " & " + subscriptions + " & " + serviceServers + " & " + serviceClients + "  & " + actionServers + " & " + actionClients + " \\\\ \n\\cline{1-12}");
    }
    @Test
    public void pubsub_minimal() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/pubsub_minimal/publisher_local_function.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/pubsub_minimal/subscriber_member_function.py"))

                .withWorkDir("ros-test-outputs/liSAROSTest/pubsub_minimal").build();
        r.dumpResults();
        printInfo(r, "pubsub\\_minimal");
    }

    @Test
    public void pubsub_minimal_oldschool() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/pubsub_minimal_oldschool/publisher_old_school.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/pubsub_minimal_oldschool/subscriber_old_school.py"))

                .withWorkDir("ros-test-outputs/liSAROSTest/pubsub_minimal_oldschool").build();
        r.dumpResults();
        printInfo(r, "pubsub\\_minimal\\_oldschool");

    }

    @Test
    public void services_minimal() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/services_minimal/client.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/services_minimal/service.py"))

                .withWorkDir("ros-test-outputs/liSAROSTest/services_minimal").build();
        r.dumpResults();
        printInfo(r, "services\\_minimal");
    }

    @Test
    public void actions_minimal() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/actions_minimal/client.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/actions_minimal/server.py"))

                .withWorkDir("ros-test-outputs/liSAROSTest/actions_minimal").build();
        r.dumpResults();
        printInfo(r, "actions\\_minimal");
    }

    @Test
    public void fruit_collectors() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/fruit_collectors/collector_node.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/fruit_collectors/vision_node.py"))

                .withWorkDir("ros-test-outputs/liSAROSTest/fruit_collectors").build();
        r.dumpResults();
        printInfo(r, "fruit\\_collectors");
    }

    @Test
    public void mechaship() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/mechaship/mechaship_classify_node.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/mechaship/mechaship_classify_sub_node.py"))
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/mechaship/mechaship_detect_node.py"))
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/mechaship/mechaship_detect_sub_node.py"))
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/mechaship/mechaship_navigation2_node.py"))
                .withWorkDir("ros-test-outputs/liSAROSTest/mechaship").build();
        r.dumpResults();
        printInfo(r, "mechaship");
    }

    @Test
    public void virtuoso_perception() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso_perception/buoy_lidar_node.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/virtuoso_perception/channel_node.py"))
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso_perception/classify_buoys_node.py"))
                .withWorkDir("ros-test-outputs/liSAROSTest/virtuoso_perception").build();
        r.dumpResults();
        printInfo(r, "virtuoso\\_perception");
    }

    @Test
    public void zumopi() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/zumopi/telemetry_pkg_camera.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/zumopi/telemetry_pkg_serial_interface.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/zumopi/telemetry_pkg_gui.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/zumopi/telemetry_pkg_ups_reading.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/zumopi/telemetry_pkg_wifi_gen.py")
                )
                .withWorkDir("ros-test-outputs/liSAROSTest/zumopi").build();
        r.dumpResults();
        printInfo(r, "zumopi");
    }

    @Test
    public void poly_dust() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/poly_dust/poly_dust_server.py"))
                .withWorkDir("ros-test-outputs/liSAROSTest/poly_dust").build();
        r.dumpResults();
        printInfo(r, "poly\\_dust\\_server");
    }

    @Test
    public void virtuoso() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_mapping_occupancy_map_generator_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/enter_and_exit_enter_exit_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/stereo_buoy_stereo_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/gymkhana_gymkhana_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/perception_main.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_change_goal.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/scan_code_scan_code_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_sensors_lidar_republish.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_test_waypoint_generator.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/wildlife_encounter_wildlife_encounter_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_test_forward.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_sensors_laser_to_pcd_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/stereo_dock_stereo_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/docking_docking_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/camera_processing_resize_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/vrx_mission_interpreter.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_test_yaw_left.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_controller_basic_pid_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_test_stop.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_sensors_gx3_republish.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_controller_choose_pid_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_sensors_camera_info_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/buoys_buoy_cam_filter_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/code_scan_code_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_controller.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_diamond.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_waypoints.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_localization_localization_debugger_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/station_keeping_station_keeping_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/wayfinding_wayfinding_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/channel_nav_channel_nav_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_sensors_f9p_gps_republish.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_controller_velocity_pid_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/dock_find_dock_entrances_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_navigation_waypoints_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_controller_cmd_vel_generator_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_test_backward.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_circle.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/buoys_buoy_lidar_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_test_data_publisher.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/acoustic_perception_acoustic_perception_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/buoys_classify_buoys_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/buoys_channel_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_station_keeping.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_set_goal.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/safety_check_safety_check_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/camera_processing_noise_filter_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_test_left.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/testing_test_yaw_right.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/heartbeat_main.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/deprecated_basic_pid_rotatin_thrusters.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/virtuoso/virtuoso_navigation_translate_node.py")
                )

                .withWorkDir("ros-test-outputs/liSAROSTest/virtuoso").build();
        r.dumpResults();
        printInfo(r, "virtuoso");
    }

    @Test
    public void Catch2023_hichewns() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/Catch2023_hichewns/cal_rtheta_joint_publisher.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/Catch2023_hichewns/cal_rtheta_cal.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/Catch2023_hichewns/cal_rtheta_index.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/Catch2023_hichewns/cal_rtheta_xy_to_rtheta_index_red.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/Catch2023_hichewns/cal_rtheta_joy_operation.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/Catch2023_hichewns/cal_rtheta_joysub_cal.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/Catch2023_hichewns/cal_rtheta_shooting_index.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/Catch2023_hichewns/cal_rtheta_state_index_red.py")
                )
                .withWorkDir("ros-test-outputs/liSAROSTest/Catch2023_hichewns").build();
        r.dumpResults();
        printInfo(r, "Catch2023\\_hichewns");
    }

    @Test
    public void spatialtopologyteleoperation() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
        .withNode(
                new PythonROSNodeBuilder(
                        "ros-tests/spatial-topology-teleoperation/waffle_topology_generate_topology.py")
        )
        .withNode(
                new PythonROSNodeBuilder(
                        "ros-tests/spatial-topology-teleoperation/waffle_topology_deproject_scan.py")
        )
        .withWorkDir("ros-test-outputs/liSAROSTest/spatial-topology-teleoperation").build();
        r.dumpResults();
        printInfo(r, "ProjectMarch");

    }
    @Test
    public void ProjectMarch() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ProjectMarch/gait_selection_gait_selection_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ProjectMarch/mujoco_sim_mujoco_sim_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ProjectMarch/mujoco_reader_mujoco_reader_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ProjectMarch/test_setup_gait_selection_test_setup_gait_selection_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ProjectMarch/mujoco_writer_mujoco_writer_node.py")
                )

                .withWorkDir("ros-test-outputs/liSAROSTest/Catch2023_hichewns").build();
        r.dumpResults();
        printInfo(r, "ProjectMarch");
    }
    @Test
    public void solarros() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/solar-ros/watcher_deliver_client.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/solar-ros/watcher_watcher_sub.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/solar-ros/watcher_watcher_pub.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/solar-ros/poly_dust_poly_dust_server.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/solar-ros/poly_dust_poly_dust_client.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/solar-ros/watcher_deliver_server.py")
                )
                .build();
        r.dumpResults();
        printInfo(r, "ProjectMarch");
    }
    @Test
    public void _5g_drone_ROS2() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/5g_drone_ROS2/api_communication_api_listener.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/5g_drone_ROS2/relais_control_relais_controller.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/5g_drone_ROS2/camera_camera_controller.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/5g_drone_ROS2/failsafe_failsafe.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/5g_drone_ROS2/test_controls_test_controller.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/5g_drone_ROS2/drone_status_drone_status.py")
                )
                .build();
        r.dumpResults();
        printInfo(r, "5g\\_drone\\_ROS2");
    }
    @Test
    public void eml4842_gps_nav() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/eml4842_gps_nav/gps_nav_vehicle_controller.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/eml4842_gps_nav/gps_nav_vehicle_simulator.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/eml4842_gps_nav/gps_nav_goal_pose_visualizer.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/eml4842_gps_nav/gps_nav_route_pose_visualizer.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/eml4842_gps_nav/gps_nav_goal_pose_creator.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/eml4842_gps_nav/gps_nav_route_pose_provider.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/eml4842_gps_nav/gps_nav_motion_spec_provider.py")
                )
                .build();
        r.dumpResults();
        printInfo(r, "eml4842\\_gps\\_nav");
    }
    @Test
    public void MARVROS() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/MARV-ROS/marv_driver_marv_status_sender.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/MARV-ROS/marv_driver_marv_can_bridge.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/MARV-ROS/marv_driver_marv_sbg_interface.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/MARV-ROS/marv_driver_marv_heartbeat_acu.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/MARV-ROS/marv_driver_marv_power_management.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/MARV-ROS/marv_driver_marv_scenario_starter.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/MARV-ROS/marv_driver_marv_logger.py")
                )
                .build();
        r.dumpResults();
        printInfo(r, "MARV-ROS");
    }
    @Test
    public void ROSLLM() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ROS-LLM/llm_robot_arx5_arm_robot.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ROS-LLM/llm_input_llm_audio_input.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ROS-LLM/llm_robot_turtle_robot.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ROS-LLM/llm_robot_turtle_robot.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/ROS-LLM/llm_output_llm_audio_output.py")
                )
                .build();
        r.dumpResults();
        printInfo(r, "ROS-LLM");
    }
    @Test
    public void VoitureAutonome() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/VoitureAutonome/my_package_communication_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/VoitureAutonome/client_client_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/VoitureAutonome/hardware_nav_v1_servo.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/VoitureAutonome/hardware_nav_v1_motor.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/VoitureAutonome/my_package_lidar_preprocessing.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/VoitureAutonome/my_package_plotter_node.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/VoitureAutonome/hardware_nav_v1_driver.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/VoitureAutonome/my_package_brain_node_MTD.py")
                )
                .build();
        r.dumpResults();
        printInfo(r, "VoitureAutonome");
    }
    @Test
    public void module89() throws Exception {
        ROSApplication r = new RosApplicationBuilder()
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_camera_fake.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_chessai_fake.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_pseudo_state.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_chessboard_classifier.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_encoder_dynamixel.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_chessboard_locator_dope.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_example_service.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_chessboard_detector_fake.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_chess_engine.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_chessboard_tracker_dual.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_chess_client.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_GameController.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_dataset_gatherer.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_camera_sim.py")
                )
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/module89/scripts_client_chessboard_locator.py")
                )
                .build();
        r.dumpResults();
        printInfo(r, "module89");
    }


}
