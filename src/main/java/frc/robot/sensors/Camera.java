package frc.robot.sensors;

import java.lang.invoke.MethodHandles;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Represents a Limelight to track AprilTags. */
public class Camera extends Sensor4237
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public class PeriodicData
    {
        //INPUTS
        private DoubleEntry yawEntry;

        //OUTPUTS
        private DoubleArrayEntry poseEntry;
    }
    
    private String cameraName;
    private LimelightHelpers.PoseEstimate mt2PoseEstimate;

    private final PeriodicData periodicData = new PeriodicData();

    private NetworkTable ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME); // custom table for AdvantageScope testing

    public Camera(String cameraName)
    {   
        super("Camera");
        this.cameraName = cameraName;
        
        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cameraName);

        periodicData.poseEntry = ASTable.getDoubleArrayTopic(cameraName + " pose").getEntry(new double[3]);
        periodicData.yawEntry = ASTable.getDoubleTopic("GyroYaw").getEntry(0.0);

        mt2PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cameraName);
    }

    /** @return the MT2 robot pose with a blue driverstration origin*/
    public Pose2d getPose()
    {
        return mt2PoseEstimate.pose;
    }

    /** @return the timestamp calculated by LimelightHelpers */
    public double getTimestamp()
    {
        return mt2PoseEstimate.timestampSeconds;
    }

    /** @return the number of tags visible */
    public int getTagCount()
    {
        return mt2PoseEstimate.tagCount;
    }

    /** @return the average distance from tags in meters */
    public double getAverageTagDistance()
    {
        return mt2PoseEstimate.avgTagDist;
    }

    @Override
    public void readPeriodicInputs() 
    {
        LimelightHelpers.SetRobotOrientation(cameraName, periodicData.yawEntry.get(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public void writePeriodicOutputs() 
    {
        // LL publishes a 3D pose in a weird format, so to make it readable
        // we need to create our own double array and publish that
        double[] poseArray = new double[3];
        poseArray[0] = mt2PoseEstimate.pose.getX();
        poseArray[1] = mt2PoseEstimate.pose.getY();
        poseArray[2] = mt2PoseEstimate.pose.getRotation().getDegrees();

        // put the pose from LL onto the Network Table so AdvantageScope can read it
        periodicData.poseEntry.set(poseArray);        
    }

    @Override
    public void runPeriodicTask()
    {
    }
}
