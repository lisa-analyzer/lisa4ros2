package it.unive.pylisa;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAReport;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.warnings.Warning;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.program.Program;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.checks.DataframeDumper;
import it.unive.pylisa.checks.DataframeStructureConstructor;
import java.io.IOException;

import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.ROSApplication;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import it.unive.ros.models.rclpy.ROSNetwork;
import org.apache.commons.io.FilenameUtils;

public class PyLiSA {

	public static void main(
			String[] args)
			throws Exception {
		/*if (args.length < 2) {
			System.err.println("PyLiSA needs two arguments: the file to analyze and the working directory");
			System.exit(-1);
		}

		String file = args[0];
		String workdir = args[1];
		String extension = FilenameUtils.getExtension(file);
		PyFrontend translator = new PyFrontend(file, extension.equals("ipynb"));
		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.optimize = true;
		conf.workdir = workdir;
		conf.jsonOutput = true;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.semanticChecks.add(new DataframeDumper());
		conf.semanticChecks.add(new DataframeStructureConstructor());

		PyFieldSensitivePointBasedHeap heap = new PyFieldSensitivePointBasedHeap();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		DataframeGraphDomain df = new DataframeGraphDomain();
		conf.abstractState = new SimpleAbstractState<>(heap, df, type);

		LiSA lisa = new LiSA(conf);
		LiSAReport report = lisa.run(program);
		if (!report.getWarnings().isEmpty()) {
			System.out.println("The analysis generated the following warnings:");
			for (Warning w : report.getWarnings())
				System.out.println("  " + w);
		}*/
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
		n.processEvents();
		ra.dumpResults();
	}

}
