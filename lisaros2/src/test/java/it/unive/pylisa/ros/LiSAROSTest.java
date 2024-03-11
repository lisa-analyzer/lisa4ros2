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
                                "ros-tests/zumopi/camera.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/zumopi/gui.py"))
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/zumopi/serial_interface.py"))
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/zumopi/ups_reading.py"))
                .withNode(
                        new PythonROSNodeBuilder(
                                "ros-tests/zumopi/wifi_stats.py"))
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
}
