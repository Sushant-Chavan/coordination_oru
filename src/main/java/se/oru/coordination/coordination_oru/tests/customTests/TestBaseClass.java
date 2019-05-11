package se.oru.coordination.coordination_oru.tests.customTests;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Comparator;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.OMPLPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Base class for custom tests")
public abstract class TestBaseClass {
    protected static int nSimulations_ = 1;
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
    protected static String testName;

    protected static void parseArguments(String[] args) {
        if (args != null)
        {
            nSimulations_ = Integer.parseInt(args[0]);
            if (args.length >= 2)
            {
                plannerType_ = OMPLPlanner.PLANNER_TYPE.valueOf(Integer.parseInt(args[1]));
            }
        }
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
        String plannerID = "_simple";
        if (type == OMPLPlanner.PLANNER_TYPE.LIGHTNING)
            plannerID = "_lightning";
        else if (type == OMPLPlanner.PLANNER_TYPE.THUNDER)
            plannerID = "_thunder";

        String filename = "generated/experienceLogs/" + experienceDBName + plannerID + ".log";
        return filename;
    }

    protected static String getCurrentTime() {
        SimpleDateFormat sdf = new SimpleDateFormat("dd-MM-yyyy kk:mm:ss");
	    return sdf.format(Calendar.getInstance().getTime());
    }
}
