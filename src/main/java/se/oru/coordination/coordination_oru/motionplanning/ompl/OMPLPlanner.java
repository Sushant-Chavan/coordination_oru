package se.oru.coordination.coordination_oru.motionplanning.ompl;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.OMPLPlannerLib.PathPose;
import se.oru.coordination.coordination_oru.util.GeometrySmoother;
import se.oru.coordination.coordination_oru.util.GeometrySmoother.SmootherControl;

public class OMPLPlanner extends AbstractMotionPlanner {

	private double robotRadius = 1.0;
	private PointerByReference path = null;
	private IntByReference pathLength = null;
	private double distanceBetweenPathPoints = 0.5;
	private double turningRadius = 1.0;
	private Coordinate[] collisionCircleCenters = null;
    private boolean isHolonomicRobot = false;
    private PLANNER_TYPE plannerType = PLANNER_TYPE.LIGHTNING;
	
	public static OMPLPlannerLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("omplMotionPlanner", "OmplMotionPlanner");
		INSTANCE = Native.loadLibrary("omplMotionPlanner", OMPLPlannerLib.class);
	}
	
	@Override
	public void setFootprint(Coordinate ... coords) {
		super.setFootprint(coords);
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newCoords = new Coordinate[coords.length+1];
		for (int i = 0; i < coords.length; i++) {
			newCoords[i] = coords[i];
		}
		newCoords[newCoords.length-1] = coords[0];
		Polygon footprint = gf.createPolygon(newCoords);
		GeometrySmoother gs = new GeometrySmoother(gf);
		SmootherControl sc = new SmootherControl() {
	        public double getMinLength() {
	            return robotRadius;
	        }
	        
	        public int getNumVertices(double length) {
	            return (int)(length/(2*robotRadius))+2;
	        }
	    };
	    gs.setControl(sc);
	    Polygon smoothFootprint = gs.smooth(footprint, 1.0);
		collisionCircleCenters = smoothFootprint.getCoordinates();
	}
	
	public Coordinate[] getCollisionCircleCenters() {
		return collisionCircleCenters;
	}
	
	public OMPLPlanner() {
		deleteDir(new File(TEMP_MAP_DIR));
		new File(TEMP_MAP_DIR).mkdir();
	}

	public void setCirclePositions(Coordinate ... circlePositions) {
		this.collisionCircleCenters = circlePositions;
	}

	@Deprecated
	public void setRobotRadius(double rad) {
		this.robotRadius = rad;
	}

	public void setRadius(double rad) {
		this.robotRadius = rad;
	}

	public void setDistanceBetweenPathPoints(double maxDistance) {
		this.distanceBetweenPathPoints = maxDistance;
	}

	public void setTurningRadius(double rad) {
		this.turningRadius = rad;
    }
    
    public String getOriginalFilename() {
        String[] bits = this.mapFilenameBAK.split("/");
        String experienceDBNameWithExtension = bits[bits.length-1];
        String[] extensionBits = experienceDBNameWithExtension.split("\\.");
        return extensionBits[0];
    }

    public void setHolonomicRobot(boolean isHolonomicRobot) {
        this.isHolonomicRobot = isHolonomicRobot;
    }

    public boolean getHolonomicRobot() {
        return this.isHolonomicRobot;
    }

    public enum PLANNER_TYPE {
        SIMPLE(0),
        LIGHTNING(1),
        THUNDER(2);
    
        private int value;
        private static Map map = new HashMap();
    
        private PLANNER_TYPE(int value) {
            this.value = value;
        }
    
        static {
            for (PLANNER_TYPE planner_type : PLANNER_TYPE.values()) {
                map.put(planner_type.value, planner_type);
            }
        }
    
        public static PLANNER_TYPE valueOf(int planner_type) {
            return (PLANNER_TYPE) map.get(planner_type);
        }
    
        public int getValue() {
            return value;
        }
    }

    public void setPlannerType(PLANNER_TYPE type) {
        this.plannerType = type;
    }

    public PLANNER_TYPE getPlannerType() {
        return this.plannerType;
    }

	@Override
	public boolean doPlanning() {
        ArrayList<PoseSteering> finalPath = new ArrayList<PoseSteering>();
        String experienceDBName = this.getOriginalFilename();
        int mode = this.isReplan ? 1 : 0;
		for (int i = 0; i < this.goal.length; i++) {
			Pose start_ = null;
			Pose goal_ = this.goal[i];
			if (i == 0) start_ = this.start;
			else start_ = this.goal[i-1];
			path = new PointerByReference();
			pathLength = new IntByReference();
			if (collisionCircleCenters == null) {
                if (!INSTANCE.plan(mapFilename, mapResolution, robotRadius,
                     start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(),
                     goal_.getY(), goal_.getTheta(), path, pathLength, 
                     distanceBetweenPathPoints, turningRadius)) return false;
			}
			else {
				double[] xCoords = new double[collisionCircleCenters.length];
				double[] yCoords = new double[collisionCircleCenters.length];
				int numCoords = collisionCircleCenters.length;
				for (int j = 0; j < collisionCircleCenters.length; j++) {
					xCoords[j] = collisionCircleCenters[j].x;
					yCoords[j] = collisionCircleCenters[j].y;
				}
				metaCSPLogger.info("Path planning with " + collisionCircleCenters.length + " circle positions");
				if (this.mapFilename != null) {
                    if (!INSTANCE.plan_multiple_circles(mapFilename, mapResolution,
                        robotRadius, xCoords, yCoords, numCoords, start_.getX(),
                        start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(),
                        goal_.getTheta(), path, pathLength, distanceBetweenPathPoints,
                        turningRadius, this.plannerType.ordinal(), experienceDBName,
                        mode, this.isHolonomicRobot)) return false;
				}
				else {
                    if (!INSTANCE.plan_multiple_circles_nomap(xCoords, yCoords, numCoords,
                        start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(),
                        goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints,
                        turningRadius, this.plannerType.ordinal(), mode, this.isHolonomicRobot)) return false;
				}
			}
			final Pointer pathVals = path.getValue();
			final PathPose valsRef = new PathPose(pathVals);
			valsRef.read();
			int numVals = pathLength.getValue();
			if (numVals == 0) return false;
			PathPose[] pathPoses = (PathPose[])valsRef.toArray(numVals);
			if (i == 0) finalPath.add(new PoseSteering(pathPoses[0].x, pathPoses[0].y, pathPoses[0].theta, 0.0));
			for (int j = 1; j < pathPoses.length; j++) finalPath.add(new PoseSteering(pathPoses[j].x, pathPoses[j].y, pathPoses[j].theta, 0.0));
			INSTANCE.cleanupPath(pathVals);
		}
		this.pathPS = finalPath.toArray(new PoseSteering[finalPath.size()]);
		return true;
	}

}
