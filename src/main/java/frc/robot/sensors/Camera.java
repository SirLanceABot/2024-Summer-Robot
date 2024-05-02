package frc.robot.sensors;

import java.lang.invoke.MethodHandles;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
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
        
        // Network Table Entry variables named with LL convention (not camelcase)
        // because they match the name of the values sent to the Network Tables by the LL
        private NetworkTableEntry botpose_wpiblue;
        private NetworkTableEntry camerapose_targetspace;

        // Instance variables named with our convention (yes camelcase)
        private boolean isTargetFound;
        private double[] botPoseWPIBlue;

        private DoubleArrayEntry dblArrayEntry;

    }

    public static final int TRANSLATION_X_METERS_INDEX = 0;
    public static final int TRANSLATION_Y_METERS_INDEX = 1;
    public static final int TRANSLATION_Z_METERS_INDEX = 2;
    public static final int ROTATION_ROLL_DEGREES_INDEX = 3;
    public static final int ROTATION_PITCH_DEGREES_INDEX = 4;
    public static final int ROTATION_YAW_DEGREES_INDEX = 5;
    public static final int TOTAL_LATENCY_INDEX = 6;
    public static final int TAG_COUNT_INDEX = 7;
    public static final int AVERAGE_TAG_DISTANCE_FROM_CAMERA_INDEX = 9;
    

    private final PeriodicData periodicData = new PeriodicData();
    private double[] poseForAS = {0.0, 0.0, 0.0};   // custom array of values to make a pose that AS can read
    private NetworkTable cameraTable;   // offical LL table (includes ALL of the things LL chooses to publish)

    private NetworkTable ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME); // custom table for AdvantageScope testing
    private double[] defaultArray = {0.0, 0.0, 0.0};

    public Camera(String cameraName)
    {   
        super("Camera");
        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cameraName);

        // Assign the Network Table variable in the constructor so the camName parameter can be used
        cameraTable = NetworkTableInstance.getDefault().getTable(cameraName);   // official limelight table

        periodicData.dblArrayEntry = ASTable.getDoubleArrayTopic(cameraName).getEntry(defaultArray);

        periodicData.botpose_wpiblue = cameraTable.getEntry("botpose_wpiblue");

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cameraName);
    }

    /** @return false if no target is found, true if target is found */
    public boolean isTargetFound()
    {
        return periodicData.isTargetFound;
    }

    /**
     * @param poseArray double array from LL Network Table
     * @return a Pose2d that can be used throught the robot code
     */
    public Pose2d convertArrayToPose(double[] poseArray)
    {
        return new Pose2d(
            new Translation2d(poseArray[TRANSLATION_X_METERS_INDEX], poseArray[TRANSLATION_Y_METERS_INDEX]),
            new Rotation2d(Units.degreesToRadians(poseArray[ROTATION_YAW_DEGREES_INDEX])));
    }

    /** @return the robot pose on the field with a blue driverstration origin*/
    public Pose2d getBotPoseBlue()
    {
        return convertArrayToPose(periodicData.botPoseWPIBlue);
    }

    /** @return the total latency from LL measurements */
    public double getTotalLatencyBlue()
    {
        return periodicData.botPoseWPIBlue[TOTAL_LATENCY_INDEX];
    }

    //** @return the number of tags visible */
    public int getTagCount()
    {
        return (int) periodicData.botPoseWPIBlue[TAG_COUNT_INDEX];
    }

    /** @return the average distance from the target in meters */
    public double getAverageDistanceFromTarget()
    {
        return periodicData.botPoseWPIBlue[AVERAGE_TAG_DISTANCE_FROM_CAMERA_INDEX];
    }

    @Override
    public void readPeriodicInputs() 
    {
        periodicData.botPoseWPIBlue = periodicData.botpose_wpiblue.getDoubleArray(new double[11]);
    }

    @Override
    public void writePeriodicOutputs() 
    {
        // LL publishes a 3D pose in a weird format, so to make it readable
        // in AS we need to create our own double array and publish that
        poseForAS[0] = periodicData.botPoseWPIBlue[0];
        poseForAS[1] = periodicData.botPoseWPIBlue[1];
        poseForAS[2] = periodicData.botPoseWPIBlue[5];
        
        // put the pose from LL onto the Network Table so AdvantageScope can read it
        // ASTable.getEntry(cameraName).setDoubleArray(poseForAS);
        periodicData.dblArrayEntry.set(poseForAS);
    }

    @Override
    public void runPeriodicTask()
    {
    }
}
