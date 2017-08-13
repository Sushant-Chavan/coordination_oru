package se.oru.coordination.coordination_oru.tests;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Coordination of 4 robots along wave-like paths obtained with the ReedsSheppCarPlanner in opposing directions.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner4 {

	public static int MIN_DELAY = 500;
	public static int MAX_DELAY = 0;

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker (also abstract)
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add a comparator to determine robot orderings thru critical sections
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
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


		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/map-empty.yaml";
		tec.setupGUI(yamlFile);
		//tec.setupGUI(null);

		tec.setUseInternalCriticalPoints(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		String mapFile = "maps"+File.separator+Missions.getProperty("image", yamlFile);
		rsp.setMapFilename(mapFile);
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRobotRadius(1.1);
		rsp.setTurningRadius(4.0);
		rsp.setNumInterpolationPoints(100);
		double deltaY = 3;
		double height = deltaY/2;
		double mapHeight = -1;

		try {
			BufferedImage img = ImageIO.read(new File(mapFile));
			mapHeight = img.getHeight()*res*0.9;
		}
		catch (IOException e) { e.printStackTrace(); }


		int[] robotIDs = new int[] {1,2,3,4};
		for (int index = 0; index < robotIDs.length; index++) {
			int robotID = robotIDs[index];
			//You probably also want to provide a non-trivial forward model
			//(the default assumes that robots can always stop)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
			ArrayList<Pose> posesRobot = new ArrayList<Pose>();
			//if (index%2==0) {
			if (robotID%2==0) {
				posesRobot.add(new Pose(2.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(10.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(18.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(26.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(34.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(42.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(50.0,mapHeight-deltaY-height*index,0.0));
			}
			else {
				posesRobot.add(new Pose(50.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(42.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(34.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(26.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(18.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(10.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(2.0,mapHeight-height*(index-1),Math.PI));
			}
			tec.placeRobot(robotID, posesRobot.get(0));
			ArrayList<PoseSteering> pathsRobot = new ArrayList<PoseSteering>();
			for (int i = 0; i < posesRobot.size()-1; i++) {
				rsp.setStart(posesRobot.get(i));
				rsp.setGoal(posesRobot.get(i+1));
				rsp.clearObstacles();
				if (!rsp.plan()) throw new Error ("No path between " + posesRobot.get(i) + " and " + posesRobot.get(i+1));			
				PoseSteering[] path = rsp.getPath();
				if (i == 0) pathsRobot.add(path[0]);
				for (int j = 1; j < path.length; j++) pathsRobot.add(path[j]);
			}
			ArrayList<PoseSteering> pathsRobotInv = new ArrayList<PoseSteering>();
			pathsRobotInv.addAll(pathsRobot);
			Collections.reverse(pathsRobotInv);

			Missions.putMission(new Mission(robotID, pathsRobot.toArray(new PoseSteering[pathsRobot.size()])));
			Missions.putMission(new Mission(robotID, pathsRobotInv.toArray(new PoseSteering[pathsRobotInv.size()])));
		}

		System.out.println("Added missions " + Missions.getMissions());

		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
						Mission m = Missions.getMission(robotID, iteration%2);
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) {
								tec.computeCriticalSections();
								if (MAX_DELAY-MIN_DELAY > 0) {
									long delay = MIN_DELAY+rand.nextInt(MAX_DELAY-MIN_DELAY);
									//Sleep for a random delay in [minDelay,maxDelay]
									try { Thread.sleep(delay); }
									catch (InterruptedException e) { e.printStackTrace(); }
								}
								tec.startTrackingAddedMissions();
								iteration++;
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			//Start the thread!
			t.start();
		}

	}

}
