package se.oru.coordination.coordination_oru.tests.customTests;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.Calendar;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.OMPLPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Robots trying to move between two openspaces connected by narrow corridor.")
public class ROPOD_BRSU_0 {

	private static boolean deleteDir(File dir) {
	    if (dir.isDirectory()) {
	        String[] children = dir.list();
	        for (int i=0; i<children.length; i++) {
	            boolean success = deleteDir(new File(dir, children[i]));
	            if (!success) {
	                return false;
	            }
	        }
	    }
	    return dir.delete();
	}

	private static void writeStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), true)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	private static void initStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), false)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 14.0;
        int CONTROL_PERIOD = 1000;

        final int numOfSimulationIterations = (args != null) ? Integer.parseInt(args[0]) : 20;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD,1000.0,MAX_VEL,MAX_ACCEL);
//		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
			}
		});

		Coordinate footprint1 = new Coordinate(-0.25,0.25);
		Coordinate footprint2 = new Coordinate(0.25,0.25);
		Coordinate footprint3 = new Coordinate(0.25,-0.25);
		Coordinate footprint4 = new Coordinate(-0.25,-0.25);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/BRSU_Floor0.yaml";
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setMap(yamlFile);
		RVizVisualization viz = new RVizVisualization();
		viz.setMap(yamlFile);
		// BrowserVisualization viz = new BrowserVisualization();
		// viz.setMap(yamlFile);
		// viz.setInitialTransform(20.0, 9.0, 2.0);
		tec.setVisualization(viz);
		
		tec.setUseInternalCriticalPoints(true);
		tec.setYieldIfParking(false);
        tec.setBreakDeadlocks(true);
        tec.setQuiet(true);
		
		Missions.loadLocationAndPathData("missions/BRSU_Floor0.txt");

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		OMPLPlanner omplPlanner = new OMPLPlanner();
		omplPlanner.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		omplPlanner.setMapResolution(res);
		omplPlanner.setRadius(0.1);
		omplPlanner.setFootprint(tec.getDefaultFootprint());
		omplPlanner.setTurningRadius(4.0);
        omplPlanner.setDistanceBetweenPathPoints(0.3);
        omplPlanner.setHolonomicRobot(true); // ROPOD Robots are holonomic
		
		//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
		tec.setMotionPlanner(omplPlanner);
		
		boolean cachePaths = false;
		String outputDir = "paths";
		boolean clearOutput = false;
		if (clearOutput) {
			deleteDir(new File(outputDir));
			new File(outputDir).mkdir();
		}
                
        int numOfRobotsPerSide = 5;
		int[] robotIDs = new int[] {1,2,3,4,5, 6, 7, 8, 9, 10};
		int locationCounter = 0;
        String[] startPositions = new String[]{"RE_S_0", "RE_S_1", "AH_S_0", "AW_S_0", "SA_S_0", "AH_S_1", "P3_S_0", "P1_S_0", "P2_S_0", "SA_S_1"};
        String[] goalPositions = new String[]{"AH_E_0", "AW_E_0", "SA_E_1", "RE_E_1", "AH_E_1", "P3_E_0", "P1_E_0", "P2_E_0", "SA_E_0", "RE_E_0"};
		for (int robotID : robotIDs) {
            tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, CONTROL_PERIOD, tec.getTemporalResolution(), 3));

            String startLocName = startPositions[robotID-1];
            String endLocName = goalPositions[robotID-1];

            Pose startLoc = Missions.getLocation(startLocName);
            Pose endLoc = Missions.getLocation(endLocName);

			locationCounter += 1;
			
			tec.placeRobot(robotID, startLoc);
			System.out.println("Placed Robot" + robotID + " in " + startLocName);

			//If path exists and we have cachePaths flag set to true, load and save computed paths
			String pathFilename = outputDir+File.separator+startLocName+"-"+endLocName+".path";
			String pathFilenameInv = outputDir+File.separator+endLocName+"-"+startLocName+".path";
			PoseSteering[] path = null;
			PoseSteering[] pathInv = null;
			File f = new File(pathFilename);
			if(!cachePaths || (cachePaths && !f.exists())) { 
				omplPlanner.setStart(startLoc);
				omplPlanner.setGoals(endLoc);
				omplPlanner.plan();
				path = omplPlanner.getPath();
				pathInv = omplPlanner.getPathInv();
				if (cachePaths) {
					Missions.writePath(pathFilename, path);
					Missions.writePath(pathFilenameInv, pathInv);
				}
			}
			else {
				path = Missions.loadPathFromFile(pathFilename);
				pathInv = Missions.loadPathFromFile(pathFilenameInv);
			}
			
			Mission m = new Mission(robotID, path, startLocName, endLocName, Missions.getLocation(startLocName), Missions.getLocation(endLocName));
            Missions.enqueueMission(m);
            if (numOfSimulationIterations > 1) {
                Mission m1 = new Mission(robotID, pathInv, endLocName, startLocName, Missions.getLocation(endLocName), Missions.getLocation(startLocName));
                Missions.enqueueMission(m1);
            }
		}

		System.out.println("Added missions " + Missions.getMissions());
		
		final String statFilename = System.getProperty("user.home")+File.separator+"stats.txt";
		String header = "#";
		for (int robotID : robotIDs) header += (robotID + "\t");
		initStat(statFilename, header);

		//Sleep a little so we can start Rviz and perhaps screencapture ;)
		//Create rviz config file by uncommenting the following line
		RVizVisualization.writeRVizConfigFile(robotIDs);
		//To visualize, run "rosrun rviz rviz -d ~/config.rviz"
		Thread.sleep(5000);
		
		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 0; i < robotIDs.length; i++) {
			final int robotID = robotIDs[i];
			//For each robot, create a thread that dispatches the "next" mission when the robot is free
			
			Thread t = new Thread() {
				@Override
				public void run() {
					boolean firstTime = true;
					int sequenceNumber = 0;
					int totalIterations = numOfSimulationIterations;
					// if (robotID%2 == 0 && numOfSimulationIterations > 1) totalIterations = numOfSimulationIterations -1;
					String lastDestination = "R_"+(robotID-1);
					long startTime = Calendar.getInstance().getTimeInMillis();
					while (true && totalIterations > 0) {
						synchronized(tec) {
							if (tec.isFree(robotID)) {
								if (!firstTime) {
									long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
									System.out.println("Time to reach " + lastDestination + " (Robot" + robotID + "): " + elapsed);
									String stat = "";
									for (int i = 1; i < robotID; i++) stat += "\t";
									stat += elapsed;
									writeStat(statFilename, stat);
								}
								startTime = Calendar.getInstance().getTimeInMillis();
								firstTime = false;
								Mission m = Missions.getMission(robotID,sequenceNumber);
								tec.addMissions(m);
								tec.computeCriticalSections();
								tec.startTrackingAddedMissions();
								sequenceNumber = (sequenceNumber+1)%Missions.getMissions(robotID).size();
								lastDestination = m.getToLocation();
								totalIterations--;
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(100); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					System.out.println("Robot" + robotID + " is done!");
				}
			};
			//Start the thread!
			t.start();
		}

	}

}