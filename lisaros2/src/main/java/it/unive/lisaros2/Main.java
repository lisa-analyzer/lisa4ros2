package it.unive.lisaros2;

import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.ROSApplication;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import it.unive.ros.models.rclpy.ROSNetwork;

public class Main {

    public static void main(
            String[] args)
            throws Exception {
		if (args.length < 2) {
			System.err.println("LiSAROS2 needs at least two arguments: the file(s) to analyze and the working directory as last argument");
			System.exit(-1);
		}

        RosApplicationBuilder rob = new RosApplicationBuilder();
        try {
            for (int i = 0; i < args.length - 1; i++) {
                rob.withNode(new PythonROSNodeBuilder(args[i]));
            }
            //rob.withNode(new PythonROSNodeBuilder("ros-tests/lasagna/pasticcio02.py"));
            rob.withWorkDir(args[args.length - 1]);
        } catch (ROSNodeBuildException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            throw new Exception(e);
        }

        ROSApplication ra = rob.build();
        System.out.println(ra.getRosNetwork().getNetworkEvents().size());
        ROSNetwork n = ra.getRosNetwork();
        //n.processEvents();
        ra.dumpResults();
    }
}
