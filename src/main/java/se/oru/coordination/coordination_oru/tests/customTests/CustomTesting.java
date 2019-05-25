package se.oru.coordination.coordination_oru.tests.customTests;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.OMPLPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Robots start at their charging locations." + 
"Each moves to a source location of a task followed by target location for the task."+
"After this they come back to their charging locations.")
public abstract class CustomTesting {
    protected static OMPLPlanner.PLANNER_TYPE plannerType_ = OMPLPlanner.PLANNER_TYPE.LIGHTNING;
    protected static int nRobots_ = 10;

    protected static double MAX_ACCEL = 3.0;
    protected static double MAX_VEL = 14.0;
    protected static int CONTROL_PERIOD = 1000;

    protected static Coordinate footprint1_ = new Coordinate(-0.25,0.25);
    protected static Coordinate footprint2_ = new Coordinate(0.25,0.25);
    protected static Coordinate footprint3_ = new Coordinate(0.25,-0.25);
    protected static Coordinate footprint4_ = new Coordinate(-0.25,-0.25);

    protected static String logFilename_;
    protected static FleetVisualization viz;
    protected static OMPLPlanner omplPlanner_;
    protected static TrajectoryEnvelopeCoordinatorSimulation tec_;

    protected static String mapConfig_;
    protected static String missionConfig_;
    protected static String mapName_;

    protected static void parseArguments(String[] args) {
        if (args != null && args.length >= 3 )
        {
            nRobots_ = Integer.parseInt(args[0]);
            plannerType_ = OMPLPlanner.PLANNER_TYPE.valueOf(Integer.parseInt(args[1]));
            mapName_ = args[2];
        }
        else {
            System.out.println("ERROR: Insuficient number of arguments passed!!");
        }
    }

    protected static void initializeTest() {
        mapConfig_ = "maps/" + mapName_ + ".yaml";
        missionConfig_ = "generated/testingData/" + mapName_ + "-" + nRobots_ + "Problems.txt";
    }

    protected static void setupTEC() {
        //Instantiate a trajectory envelope coordinator.
        //The TrajectoryEnvelopeCoordinatorSimulation implementation provides
        // -- the factory method getNewTracker() which returns a trajectory envelope tracker
        // -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
        //You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
        tec_ = new TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD,1000.0,MAX_VEL,MAX_ACCEL);
        //tec_ = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
        tec_.addComparator(new Comparator<RobotAtCriticalSection> () {
            @Override
            public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                CriticalSection cs = o1.getCriticalSection();
                RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
                RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
                return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
            }
        });
        tec_.addComparator(new Comparator<RobotAtCriticalSection> () {
            @Override
            public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
            }
        });

        tec_.setDefaultFootprint(footprint1_, footprint2_, footprint3_, footprint4_);

        //Need to setup infrastructure that maintains the representation
        tec_.setupSolver(0, 100000000);

        setupVisualization();
        tec_.setVisualization(viz);

        tec_.setUseInternalCriticalPoints(true);
        tec_.setYieldIfParking(false);
        tec_.setBreakDeadlocks(true);
        tec_.setQuiet(true);

        //MetaCSPLogging.setLevel(tec_.getClass().getSuperclass(), Level.FINEST);

        //Instantiate a simple motion planner
        setupMotionPlanner(tec_.getDefaultFootprint(), plannerType_, true);

        //In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
        tec_.setMotionPlanner(omplPlanner_);

        logFilename_ = getLogFileName(omplPlanner_.getOriginalFilename(), plannerType_);
    }

    protected static void setupVisualization() {
        //Setup a simple GUI (null means empty map, otherwise provide yaml file)

        //viz = new JTSDrawingPanelVisualization();
        //viz.setMap(mapConfig_);
        
        viz = new RVizVisualization();
        viz.setMap(mapConfig_);

        // viz = new BrowserVisualization();
        // viz.setMap(yamlFile);
        // viz.setInitialTransform(20.0, 9.0, 2.0);
    }

    protected static void setupMotionPlanner(Coordinate[] footprint,
                                                  OMPLPlanner.PLANNER_TYPE plannerType,
                                                  boolean isHolonomicRobot) {
        //Instantiate a simple motion planner
        omplPlanner_ = new OMPLPlanner();
        omplPlanner_.setMapFilename("maps"+File.separator+Missions.getProperty("image", mapConfig_));
        double res = Double.parseDouble(Missions.getProperty("resolution", mapConfig_));
        omplPlanner_.setMapResolution(res);
        omplPlanner_.setRadius(0.1);
        omplPlanner_.setFootprint(footprint);
        omplPlanner_.setTurningRadius(4.0);
        omplPlanner_.setDistanceBetweenPathPoints(0.3);
        omplPlanner_.setHolonomicRobot(isHolonomicRobot);
        omplPlanner_.setPlannerType(plannerType);
    }

    protected static void appendToFile(String filePath, String text) {
		File file = new File(filePath);
		FileWriter fr = null;
		try {
			// Below constructor argument decides whether to append or override
			fr = new FileWriter(file, true);
			fr.write(text);

		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			try {
				fr.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
    }
    
    protected static String getLogFileName(String experienceDBName, OMPLPlanner.PLANNER_TYPE type) {
        String plannerID = "_unknown";
        if (type == OMPLPlanner.PLANNER_TYPE.LIGHTNING)
            plannerID = "_lightning";
        else if (type == OMPLPlanner.PLANNER_TYPE.THUNDER)
            plannerID = "_thunder";
        else if (type == OMPLPlanner.PLANNER_TYPE.SIMPLE_RRT_CONNECT)
            plannerID = "_rrt_connect";
        else if (type == OMPLPlanner.PLANNER_TYPE.SIMPLE_RRT_STAR)
            plannerID = "_rrt_star";

        String filename = "generated/experienceLogs/" + experienceDBName + plannerID + ".log";
        return filename;
    }

    protected static String getCurrentTime() {
        SimpleDateFormat sdf = new SimpleDateFormat("dd-MM-yyyy kk:mm:ss");
	    return sdf.format(Calendar.getInstance().getTime());
    }

    protected static int[] addMissions() {
        int[] robotIDs = new int[nRobots_];
        ArrayList<String> chargingPos = new ArrayList<String>();
        ArrayList<String> sourcePos = new ArrayList<String>();
        ArrayList<String> targetPos = new ArrayList<String>();

        for (int i = 0; i < nRobots_; i++) {
            robotIDs[i] = i+1;
            chargingPos.add("C_"+Integer.toString(i));
            sourcePos.add("S_"+Integer.toString(i));
            targetPos.add("T_"+Integer.toString(i));
        }

        for (int robotID : robotIDs) {
            tec_.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, CONTROL_PERIOD, tec_.getTemporalResolution(), 3));

            String chargingPosName = chargingPos.get(robotID-1);
            String sourcePosName = sourcePos.get(robotID-1);
            String targetPosName = targetPos.get(robotID-1);

            Pose chargingPose = Missions.getLocation(chargingPosName);
            Pose sourcePose = Missions.getLocation(sourcePosName);
            Pose targetPose = Missions.getLocation(targetPosName);

            tec_.placeRobot(robotID, chargingPose);
            String robotTag = "[Robot-" + robotID + "]";
            System.out.println(robotTag + " placed at " + chargingPosName);

            // Setup charging station to source mission
            omplPlanner_.setStart(chargingPose);
            omplPlanner_.setGoals(sourcePose);
            omplPlanner_.clearObstacles();
            appendToFile(logFilename_, robotTag + " Start planning from " + chargingPosName + " to " + sourcePosName + "\n");
            omplPlanner_.plan();
            appendToFile(logFilename_, robotTag + " planning from " + chargingPosName + " to " + sourcePosName + " complete\n");
            PoseSteering[] pathCtoS = omplPlanner_.getPath();
            Mission missionCtoS = new Mission(robotID, pathCtoS, chargingPosName, sourcePosName, Missions.getLocation(chargingPosName), Missions.getLocation(sourcePosName));
            Missions.enqueueMission(missionCtoS);

            // Setup source to target mission
            omplPlanner_.setStart(sourcePose);
            omplPlanner_.setGoals(targetPose);
            appendToFile(logFilename_, robotTag + " Start planning from " + sourcePosName + " to " + targetPosName + "\n");
            omplPlanner_.plan();
            appendToFile(logFilename_, robotTag + " planning from " + sourcePosName + " to " + targetPosName + " complete\n");
            PoseSteering[] pathStoT = omplPlanner_.getPath();
            Mission missionStoT = new Mission(robotID, pathStoT, sourcePosName, targetPosName, Missions.getLocation(sourcePosName), Missions.getLocation(targetPosName));
            Missions.enqueueMission(missionStoT);

            // Setup target to charging station mission
            omplPlanner_.setStart(targetPose);
            omplPlanner_.setGoals(chargingPose);
            appendToFile(logFilename_, robotTag + " Start planning from " + targetPosName + " to " + chargingPosName + "\n");
            omplPlanner_.plan();
            appendToFile(logFilename_, robotTag + " planning from " + targetPosName + " to " + chargingPosName + " complete\n");
            PoseSteering[] pathTtoC = omplPlanner_.getPath();
            Mission missionTtoC= new Mission(robotID, pathTtoC, targetPosName, chargingPosName, Missions.getLocation(targetPosName), Missions.getLocation(chargingPosName));
            Missions.enqueueMission(missionTtoC);
		}

        System.out.println("Added missions " + Missions.getMissions());
        
        return robotIDs;
    }

    protected static int[] initializeRobots() {
        int[] robotIDs = new int[nRobots_];
        for (int i = 0; i < nRobots_; i++) {
            robotIDs[i] = i+1;
            tec_.setForwardModel(robotIDs[i], new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, CONTROL_PERIOD, tec_.getTemporalResolution(), 3));
            String chargingPosName = "C_" + Integer.toString(i);
            Pose chargingPose = Missions.getLocation(chargingPosName);
            tec_.placeRobot(robotIDs[i], chargingPose);
            String robotTag = "[Robot-" + robotIDs[i] + "]";
            System.out.println(robotTag + " placed at " + chargingPosName);
            Missions.enqueueMission(getMission(robotIDs[i], 0));
        }
        return robotIDs;
    }

    protected static Mission getMission(int robotID, int missionNumber) {
        String chargingPosName = "C_" + Integer.toString(robotID - 1);
        String sourcePosName = "S_" + Integer.toString(robotID - 1);
        String targetPosName = "T_" + Integer.toString(robotID - 1);

        Pose chargingPose = Missions.getLocation(chargingPosName);
        Pose sourcePose = Missions.getLocation(sourcePosName);
        Pose targetPose = Missions.getLocation(targetPosName);

        String robotTag = "[Robot-" + robotID + "]";

        Mission mission = null;
        if (missionNumber == 0) {
            // Setup charging station to source mission
            omplPlanner_.setStart(chargingPose);
            omplPlanner_.setGoals(sourcePose);
            appendToFile(logFilename_, robotTag + " Start planning from " + chargingPosName + " to " + sourcePosName + "\n");
            omplPlanner_.plan();
            appendToFile(logFilename_, robotTag + " planning from " + chargingPosName + " to " + sourcePosName + " complete\n");
            PoseSteering[] pathCtoS = omplPlanner_.getPath();
            mission = new Mission(robotID, pathCtoS, chargingPosName, sourcePosName, Missions.getLocation(chargingPosName), Missions.getLocation(sourcePosName));
        }
        else if (missionNumber == 1) {
            // Setup source to target mission
            omplPlanner_.setStart(sourcePose);
            omplPlanner_.setGoals(targetPose);
            appendToFile(logFilename_, robotTag + " Start planning from " + sourcePosName + " to " + targetPosName + "\n");
            omplPlanner_.plan();
            appendToFile(logFilename_, robotTag + " planning from " + sourcePosName + " to " + targetPosName + " complete\n");
            PoseSteering[] pathStoT = omplPlanner_.getPath();
            mission = new Mission(robotID, pathStoT, sourcePosName, targetPosName, Missions.getLocation(sourcePosName), Missions.getLocation(targetPosName));
        }
        else if (missionNumber == 2) {
            // Setup target to charging station mission
            omplPlanner_.setStart(targetPose);
            omplPlanner_.setGoals(chargingPose);
            appendToFile(logFilename_, robotTag + " Start planning from " + targetPosName + " to " + chargingPosName + "\n");
            omplPlanner_.plan();
            appendToFile(logFilename_, robotTag + " planning from " + targetPosName + " to " + chargingPosName + " complete\n");
            PoseSteering[] pathTtoC = omplPlanner_.getPath();
            mission = new Mission(robotID, pathTtoC, targetPosName, chargingPosName, Missions.getLocation(targetPosName), Missions.getLocation(chargingPosName));
        }
        else {
            System.out.println("ERROR! Mission number must be less than 3!");
        }

        return mission;
    }

    public static void main(String[] args) throws InterruptedException {
        parseArguments(args);
        initializeTest();
        setupTEC();

        Missions.loadLocationAndPathData(missionConfig_);
        appendToFile(logFilename_, "\n\nTest \"" + mapName_ + "\" started at " + getCurrentTime() + "\n");

        int[] robotIDs = addMissions();
        // int[] robotIDs = initializeRobots();

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
                    int missionNumber = 0;
                    String sourceLocation = "";
                    String destinationLocation = "";
                    long startTime = Calendar.getInstance().getTimeInMillis();
                    String robotTag = "[ROBOT-" + robotID + "]";
                    int maxNumOfMissions = 3;
                    while (true) {
                        synchronized(tec_) {
                            if (tec_.isFree(robotID)) {
                                if (!firstTime) {
                                    appendToFile(logFilename_, robotTag + " Mission " + missionNumber + " from " + sourceLocation +
                                    " to " + destinationLocation + " completed at " + getCurrentTime() + "\n");
                                    long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
                                    appendToFile(logFilename_, robotTag + " Time to complete mission " + missionNumber + " : " + elapsed/1000.0 + "s\n");

                                    missionNumber = missionNumber+1;
                                    if (missionNumber >= maxNumOfMissions)
                                        break;
                                }
                                startTime = Calendar.getInstance().getTimeInMillis();

                                if (Missions.getMissions(robotID).size() <= missionNumber)
                                    Missions.enqueueMission(getMission(robotID, missionNumber));

                                Mission m = Missions.getMission(robotID, missionNumber);
                                sourceLocation = m.getFromLocation();
                                destinationLocation = m.getToLocation();
                                appendToFile(logFilename_, robotTag + " Start Mission " + missionNumber + " from " + sourceLocation +
                                " to " + destinationLocation + " at " + getCurrentTime() + "\n");
                                tec_.addMissions(m);
                                tec_.computeCriticalSections();
                                tec_.startTrackingAddedMissions();
                                firstTime = false;
                            }
                        }
                        //Sleep for a little (2 sec)
                        try { Thread.sleep(100); }
                        catch (InterruptedException e) { e.printStackTrace(); }
                    }
                    System.out.println("Robot" + robotID + " is done!");
                    appendToFile(logFilename_, robotTag + " done!\n");
                }
            };
            //Start the thread!
            t.start();
        }
    }

}
