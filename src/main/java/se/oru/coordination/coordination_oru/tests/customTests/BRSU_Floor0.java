package se.oru.coordination.coordination_oru.tests.customTests;

import java.util.Calendar;
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

    public static void main(String[] args) throws InterruptedException {
        initializeTest();
        parseArguments(args);
        setupTEC();

        Missions.loadLocationAndPathData(missionConfig_);
        appendToFile(logFilename_, "\n\nTest \"" + testName + "\" started at " + getCurrentTime() + "\n");

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
                                    appendToFile(logFilename_, robotTag + " Time to complete mission " + elapsed/1000.0 + "s\n");

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
