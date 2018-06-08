package se.oru.coordination.coordination_oru.simulation2D;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashSet;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.UI.Callback;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import geometry_msgs.Point;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.Dependency;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;

public class TrajectoryEnvelopeCoordinatorSimulation extends TrajectoryEnvelopeCoordinator {

	protected static final long START_TIME = Calendar.getInstance().getTimeInMillis();
	protected double MAX_VELOCITY;
	protected double MAX_ACCELERATION;
	protected int trackingPeriodInMillis;
	protected boolean useInternalCPs = true;
	protected AbstractMotionPlanner mp = null;

	public int getTrackingPeriod() {
		return trackingPeriodInMillis;
	}

	public double getTemporalResolution() {
		return 1000.0;
	}

	public void setMotionPlanner(AbstractMotionPlanner mp) {
		this.mp = mp;
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>MAX_VELOCITY</code> = 10.0</li>
	 * <li><code>MAX_ACCELERATION</code> = 1.0</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation() {
		this(1000, 1000, 10.0, 1.0, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION) {
		this(CONTROL_PERIOD, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(double MAX_VELOCITY, double MAX_ACCELERATION) {
		this(1000, 1000, MAX_VELOCITY, MAX_ACCELERATION, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <li><code>MAX_VELOCITY</code> = 10.0</li>
	 * <li><code>MAX_ACCELERATION</code> = 1.0</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 * and given parameters.
	 * @param CONTROL_PERIOD The control period of the coordinator (e.g., 1000 msec)
	 * @param TEMPORAL_RESOLUTION The temporal resolution at which the control period is specified (e.g., 1000)
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION) {
		this(CONTROL_PERIOD, TEMPORAL_RESOLUTION, 10.0, 1.0, 30);
	}

	private ArrayList<Integer> computeStoppingPoints(PoseSteering[] poses) {
		ArrayList<Integer> ret = new ArrayList<Integer>();
		double prevTheta = poses[0].getTheta();
		if (poses.length > 1) prevTheta = Math.atan2(poses[1].getY() - poses[0].getY(), poses[1].getX() - poses[0].getX());
		for (int i = 0; i < poses.length-1; i++) {
			double theta = Math.atan2(poses[i+1].getY() - poses[i].getY(), poses[i+1].getX() - poses[i].getX());
			double deltaTheta = (theta-prevTheta);
			prevTheta = theta;
			if (Math.abs(deltaTheta) > Math.PI/2 && Math.abs(deltaTheta) < 1.9*Math.PI) {
				ret.add(i);
			}
		}
		return ret;
	}

	@Override
	public boolean addMissions(Mission... missions) {
		if (this.useInternalCPs) {
			for (Mission m : missions) {
				PoseSteering[] path = m.getPath();
				ArrayList<Integer> sps = computeStoppingPoints(path);
				for (Integer i : sps) m.setStoppingPoint(path[i-1].getPose(), 100);				
			}
		}
		if (!super.addMissions(missions)) {
			for (Mission m : missions) {
				m.clearStoppingPoints();
				return false;
			}			
		}
		return true;
	}


	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with given parameters.
	 * @param CONTROL_PERIOD The control period of the coordinator (e.g., 1000 msec)
	 * @param TEMPORAL_RESOLUTION The temporal resolution at which the control period is specified (e.g., 1000)
	 * @param MAX_VELOCITY The maximum speed of a robot (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 * @param MAX_ACCELERATION The maximum acceleration/deceleration of a robot (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 * @param trackingPeriodInMillis The tracking period in milliseconds (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION, int trackingPeriodInMillis) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
		this.MAX_VELOCITY = MAX_VELOCITY;
		this.MAX_ACCELERATION = MAX_ACCELERATION;
		this.trackingPeriodInMillis = trackingPeriodInMillis;
	}

	/**
	 * Enable (default) or disable the use of internal critical points in the {@link TrajectoryEnvelopeTrackerRK4} trackers.
	 * @param value <code>true</code> if these critical points should be used to slow down, <code>false</code> otherwise.
	 */
	public void setUseInternalCriticalPoints(boolean value) {
		this.useInternalCPs = value;
	}

	@Override
	public AbstractTrajectoryEnvelopeTracker getNewTracker(TrajectoryEnvelope te, TrackingCallback cb) {

		TrajectoryEnvelopeTrackerRK4 ret = new TrajectoryEnvelopeTrackerRK4(te, trackingPeriodInMillis, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, this, cb) {

			//Method for measuring time in the trajectory envelope tracker
			@Override
			public long getCurrentTimeInMillis() {
				return Calendar.getInstance().getTimeInMillis()-START_TIME;
			}
		};
		//ret.setUseInternalCriticalPoints(this.useInternalCPs);
		ret.setUseInternalCriticalPoints(false);
		return ret;
	}

	//Method for measuring time in the trajectory envelope coordinator
	@Override
	public long getCurrentTimeInMillis() {
		return Calendar.getInstance().getTimeInMillis()-START_TIME;
	}
	
	private Geometry[] getObstaclesFromWaitingRobots(int robotID) {
		//Compute one obstacle per robot that is waiting for this robot, placed in the waiting robot's waiting pose
		ArrayList<Geometry> ret = new ArrayList<Geometry>();
		for (Dependency dep : getCurrentDependencies()) {
			int drivingID = dep.getDrivingRobotID();
			int waitingID = dep.getWaitingRobotID();
			if (robotID == drivingID) {
				Pose waitingPose  = dep.getWaitingTrajectoryEnvelope().getTrajectory().getPose()[dep.getWaitingPoint()];
				ret.add(makeObstacles(waitingID, waitingPose)[0]);
			}
		}
		return ret.toArray(new Geometry[ret.size()]);
	}
	
	@Override
	protected void rePlanPath(HashSet<Integer> robotsToReplan) {
		if (this.mp == null) throw new Error("Please provide a motion planner for replanning!");		
		for (int robotID : robotsToReplan) {
			int currentWaitingIndex = -1;
			Pose currentWaitingPose = null;
			Pose currentWaitingGoal = null;
			PoseSteering[] oldPath = null;
			for (Dependency dep : getCurrentDependencies()) {
				if (dep.getWaitingRobotID() == robotID) {
					currentWaitingIndex = dep.getWaitingPoint();
					currentWaitingPose = dep.getWaitingPose();
					Trajectory traj = dep.getWaitingTrajectoryEnvelope().getTrajectory();
					oldPath = traj.getPoseSteering();
					currentWaitingGoal = oldPath[oldPath.length-1].getPose();
					break;
				}
			}
			mp.setStart(currentWaitingPose);
			mp.setGoals(currentWaitingGoal);
			mp.clearObstacles();
			Geometry[] obstacles = getObstaclesFromWaitingRobots(robotID);
			mp.addObstacles(obstacles);			
			metaCSPLogger.info("Attempting to re-plan path of Robot" + robotID + "...");
			if (mp.plan()) {
				PoseSteering[] newPath = mp.getPath();
//				System.out.println("REPLANNING for Robot" + robotID + " SUCCEEDED!");
//				System.out.println("OLD PATH IS " + oldPath.length + " POSES LONG");
//				System.out.println("NEW PATH IS " + newPath.length + " POSES LONG");
				PoseSteering[] newCompletePath = new PoseSteering[newPath.length+currentWaitingIndex];
				for (int i = 0; i < newCompletePath.length; i++) {
					if (i < currentWaitingIndex) newCompletePath[i] = oldPath[i];
					else newCompletePath[i] = newPath[i-currentWaitingIndex];
				}
				replacePath(robotID, newPath);
				replanningSpawned.remove(robotsToReplan);
				break;
			}
		}
	}

}
