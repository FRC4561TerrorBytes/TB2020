package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A helper class with methods to get vision data from NetworkTables
 * @author Zane
 */
public class VisionData {

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    
    /**
     * TODO Return false if the pi has not pushed data in a while
     * @return True if NetworkTable values have been created
     */
    public static boolean isReady() {
        return table.getEntry("detected?").exists();
    }
    /**
     * @return True if a vision target has been detected
     */
    public static boolean isDetected() {
        return table.getEntry("detected?").getBoolean(false);
    }
    /**
     * @return The horizontal angle between the robot and center of vision target in degrees
     * Returns 0.0 by default if there is no data, check using {@link isReady()}
     */
    public static double getXAngle() {
        return table.getEntry("xAngle").getDouble(0.0);
    }
    /**
     * @return The horizonal distance between the robot and center of vision target in inches
     * Returns 0.0 by default if there is no data, check using {@link isReady()}
     */
    public static double getDistance() {
        return table.getEntry("distance").getDouble(0.0);
    }
    
}