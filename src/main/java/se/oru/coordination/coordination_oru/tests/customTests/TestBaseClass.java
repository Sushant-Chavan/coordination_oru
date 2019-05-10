package se.oru.coordination.coordination_oru.tests.customTests;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
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

@DemoDescription(desc = "10 Robots start at their charging locations." + 
"Each moves to a source location of a task followed by target location for the task."+
"After this they come back to their charging locations.")
public class TestBaseClass {
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
