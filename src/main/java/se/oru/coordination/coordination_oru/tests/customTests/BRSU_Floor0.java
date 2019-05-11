package se.oru.coordination.coordination_oru.tests.customTests;

import java.util.ArrayList;
import java.util.Calendar;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "10 Robots start at their charging locations." + 
"Each moves to a source location of a task followed by target location for the task."+
"After this they come back to their charging locations.")
public class BRSU_Floor0 extends TestBaseClass {

    protected static void initializeTest() {
        mapConfig_ = "maps/BRSU_Floor0.yaml";
        missionConfig_ = "generated/testingData/BRSU_Floor0-10Problems.txt";
        testName = "BRSU_Floor0";
        nRobots_ = 10;
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
        initializeTest();
        parseArguments(args);
        setupTEC();

        Missions.loadLocationAndPathData(missionConfig_);
        appendToFile(logFilename_, "\n\nTest \"" + testName + "\" started at " + getCurrentTime() + "\n");

        // int[] robotIDs = addMissions_new();
        int[] robotIDs = initializeRobots();

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
                    int totalIterations = nSimulations_;
                    String sourceLocation = "";
                    String destinationLocation = "";
                    long startTime = Calendar.getInstance().getTimeInMillis();
                    String robotTag = "[ROBOT-" + robotID + "]";
                    int maxNumOfMissions = 3;
                    while (true) {
                        synchronized(tec_) {
                            if (tec_.isFree(robotID)) {
                                if (!firstTime) {
                                    appendToFile(logFilename_, robotTag + " Mission from " + sourceLocation +
                                    " to " + destinationLocation + " completed at " + getCurrentTime() + "\n");
                                    long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
                                    appendToFile(logFilename_, robotTag + "Time to complete mission " + elapsed/1000.0 + "s\n");

                                    missionNumber = (missionNumber+1)%maxNumOfMissions;
                                    if (missionNumber == 0)
                                        totalIterations--;
                                    if (totalIterations <= 0)
                                    {
                                        break;
                                    }
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
