package se.oru.coordination.coordination_oru.util;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.servlet.ServletContextHandler;
import org.eclipse.jetty.servlet.ServletHolder;
import org.eclipse.jetty.websocket.api.RemoteEndpoint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.RobotReport;

public class BrowserVisualization implements FleetVisualization {
	
	private ArrayList<String> msgQueue = new ArrayList<String>();
	private static final int UPDATE_PERIOD = 10;

	public BrowserVisualization() {
		this("localhost");
	}
	
	public BrowserVisualization(String serverHostNameOrIP) {
		BrowserVisualization.setupVizMessageServer();
        Thread updateThread = new Thread("Visualization update thread") {
        	public void run() {
        		while (true) {
        			sendMessages();
        			try { Thread.sleep(UPDATE_PERIOD); }
        			catch (InterruptedException e) { e.printStackTrace(); }
        		}
        	}
        };
        updateThread.start();
        BrowserVisualization.setupVizServer(serverHostNameOrIP);
	}
	
	private static void setupVizServer(String serverHostNameOrIP) {
		Server server = new Server(8080);
		server.setHandler(new BrowserVisualizationServer(serverHostNameOrIP));
		try {
			server.start();
			//server.join();
		}
        catch (Throwable t) { t.printStackTrace(System.err); }
	}
	
	private static void setupVizMessageServer() {
        Server server = new Server();
        ServerConnector connector = new ServerConnector(server);
        connector.setPort(8081);
        server.addConnector(connector);

        // Setup the basic application "context" for this application at "/"
        // This is also known as the handler tree (in jetty speak)
        ServletContextHandler context = new ServletContextHandler(ServletContextHandler.SESSIONS);
        context.setContextPath("/");
        server.setHandler(context);
        
        // Add a websocket to a specific path spec
        ServletHolder holderEvents = new ServletHolder("ws-events", BrowserVisualizationServlet.class);
        context.addServlet(holderEvents, "/fleet-events/*");
        
        try {
            server.start();
            server.dump(System.err);
            //server.join();
        }
        catch (Throwable t) { t.printStackTrace(System.err); }		
	}
	
	private void enqueueMessage(String message) {
		if (BrowserVisualizationSocket.ENDPOINTS != null && BrowserVisualizationSocket.ENDPOINTS.size() > 0) {
			synchronized (BrowserVisualizationSocket.ENDPOINTS) {
				this.msgQueue.add(message);
			}
		}
	}
	
	private void sendMessages() {
		if (BrowserVisualizationSocket.ENDPOINTS != null && BrowserVisualizationSocket.ENDPOINTS.size() > 0) {
			synchronized (BrowserVisualizationSocket.ENDPOINTS) {
				for (String message : this.msgQueue) {
					sendMessage(message);
				}
				msgQueue.clear();
				sendUpdate();
			}
		}
	}
	
	private void sendMessage(String text) {
		if (BrowserVisualizationSocket.ENDPOINTS != null) {
			for (RemoteEndpoint rep : BrowserVisualizationSocket.ENDPOINTS) {
				try {
					rep.sendString(text);
				}
				catch(IOException e) { e.printStackTrace(); }
			}
		}
	}

	@Override
	public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String... extraStatusInfo) {
		double x = rr.getPose().getX();
		double y = rr.getPose().getY();
		double theta = rr.getPose().getTheta();
		String name = "R"+te.getRobotID();
		String extraData = " : " + rr.getPathIndex();
		if (extraStatusInfo != null) {
			for (String st : extraStatusInfo) {
				extraData += (" | " + st);
			}
		}
		Geometry geom = null;
		if (rr.getPathIndex() != -1) geom = TrajectoryEnvelope.getFootprint(te.getFootprint(), x, y, theta);
		else geom = TrajectoryEnvelope.getFootprint(te.getFootprint(), te.getTrajectory().getPose()[0].getX(), te.getTrajectory().getPose()[0].getY(), te.getTrajectory().getPose()[0].getTheta()); 
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(name, geom, "#ff0000", -1, true, extraData) + "}";
		enqueueMessage(jsonString);
	}

	@Override
	public void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor) {
		Geometry arrow = createArrow(rrWaiting.getPose(), rrDriving.getPose());
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(dependencyDescriptor, arrow, "#adccff", 1000, true, null) + "}";
		enqueueMessage(jsonString);
	}
	
	private String geometryToJSONString(String name, Geometry geom, String color, long age, boolean filled, String extraData) {
		String ret = "{ \"name\" : \"" + name + "\", \"color\" : \"" + color + "\", ";
		if (age > 0) ret += " \"age\" : " + age + ", ";
		ret += " \"filled\" : " + filled + ", ";
		if (extraData != null && !extraData.trim().equals("")) ret += " \"extraData\" : \"" + extraData + "\", ";		
		ret += "\"coordinates\" : [";
		Coordinate[] coords = geom.getCoordinates();
		for (int i = 0; i < coords.length; i++) {
			ret += "{\"x\" : " + coords[i].x + ", \"y\" : " + coords[i].y + "}";
			if (i < coords.length-1) ret += ", ";
		}
		ret += "]}";
		return ret;
	}

	@Override
	public void addEnvelope(TrajectoryEnvelope te) {
		GeometricShapeDomain dom = (GeometricShapeDomain)te.getEnvelopeVariable().getDomain();
		Geometry geom = dom.getGeometry();
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+te.getID(), geom, "#efe007", -1, false, null) + "}";
		enqueueMessage(jsonString);
	}

	@Override
	public void removeEnvelope(TrajectoryEnvelope te) {
		String jsonString = "{ \"operation\" : \"removeGeometry\","
				+ "\"data\" : "
				+ "{ \"name\" : \""+ "_"+te.getID() +"\" }}";
		enqueueMessage(jsonString);
	}

	@Override
	public void updateVisualization() {
		// This method does nothing - reason:
		// Viz change events are buffered and sent by an internal thread
		// in bursts every UPDATE_PERIOD ms to avoid blocking of RemoteEndpopints
	}
	
	public void sendUpdate() {
		String callRefresh = "{ \"operation\" : \"refresh\" }";
		sendMessage(callRefresh);
	}
		
	private Geometry createArrow(Pose pose1, Pose pose2) {		
		GeometryFactory gf = new GeometryFactory();
		double aux = 1.8;
		double distance = (1.6)*Math.sqrt(Math.pow((pose2.getX()-pose1.getX()),2)+Math.pow((pose2.getY()-pose1.getY()),2));
		double theta = Math.atan2(pose2.getY() - pose1.getY(), pose2.getX() - pose1.getX());
		Coordinate[] coords = new Coordinate[8];
		coords[0] = new Coordinate(0.0,-0.3);
		coords[1] = new Coordinate(distance-aux,-0.3);
		coords[2] = new Coordinate(distance-aux,-0.8);
		coords[3] = new Coordinate(distance,0.0);
		coords[4] = new Coordinate(distance-aux,0.8);
		coords[5] = new Coordinate(distance-aux,0.3);
		coords[6] = new Coordinate(0.0,0.3);
		coords[7] = new Coordinate(0.0,-0.3);
		Polygon arrow = gf.createPolygon(coords);
		AffineTransformation at = new AffineTransformation();
		at.scale(1/1.6, 1/1.6);
		at.rotate(theta);
		at.translate(pose1.getX(), pose1.getY());
		Geometry ret = at.transform(arrow);
		return ret;
	}

	public void setMap(String mapYAMLFile) {
		try {
			File file = new File(mapYAMLFile);
			BufferedReader br = new BufferedReader(new FileReader(file));
			String imageFileName = null;
			String st;
			while((st=br.readLine()) != null){
				String key = st.substring(0, st.indexOf(":")).trim();
				String value = st.substring(st.indexOf(":")+1).trim();
				if (key.equals("image")) imageFileName = file.getParentFile()+File.separator+value;
				else if (key.equals("resolution")) BrowserVisualizationSocket.resolution = Double.parseDouble(value);
				else if (key.equals("origin")) {
					String x = value.substring(1, value.indexOf(",")).trim();
					String y = value.substring(value.indexOf(",")+1, value.indexOf(",", value.indexOf(",")+1)).trim();
					BrowserVisualizationSocket.origin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
				}
			}
			br.close();
			BrowserVisualizationSocket.map = ImageIO.read(new File(imageFileName));
		}
		catch (IOException e) { e.printStackTrace(); }
	}

	@Override
	public int periodicEnvelopeRefreshInMillis() {
		return 1000;
	}

}