package frc.robot.sensors;

import java.lang.invoke.MethodHandles;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
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
        private NetworkTableEntry botpose_wpiblue;  // MegaTag1
        private NetworkTableEntry botpose_orb_wpiblue; // MegaTag2

        // Instance variables named with our convention (yes camelcase)
        private boolean isTargetFound;
        private double[] botPoseWPIBlue;
        private double[] botPoseOrbWPIBlue;

        private DoubleArrayEntry megaTag1Entry;
        private DoubleArrayEntry megaTag2Entry;
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
    private double[] megaTag1Pose = {0.0, 0.0, 0.0};    // custom array of values to make a MT1 pose that AS can read
    private double[] megaTag2Pose = {0.0, 0.0, 0.0};    // custom array of values to make a MT2 pose that AS can read
    private NetworkTable cameraTable;   // offical LL table (includes ALL of the things LL chooses to publish)

    private NetworkTable ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME); // custom table for AdvantageScope testing
    private double[] defaultArray = {0.0, 0.0, 0.0};

    public Camera(String cameraName)
    {   
        super("Camera");
        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cameraName);

        // Assign the Network Table variable in the constructor so the camName parameter can be used
        cameraTable = NetworkTableInstance.getDefault().getTable(cameraName);   // official limelight table

        periodicData.megaTag1Entry = ASTable.getDoubleArrayTopic(cameraName).getEntry(defaultArray);
        periodicData.megaTag2Entry = ASTable.getDoubleArrayTopic(cameraName).getEntry(defaultArray);

        periodicData.botpose_wpiblue = cameraTable.getEntry("botpose_wpiblue");
        periodicData.botpose_orb_wpiblue = cameraTable.getEntry("botpose_orb_wpiblue");

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cameraName);
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
        // return convertArrayToPose(periodicData.botPoseWPIBlue);  // MegaTag1
        return convertArrayToPose(periodicData.botPoseOrbWPIBlue);  // MegaTag2
    }

    /** @return the total latency from LL measurements */
    public double getTotalLatencyBlue()
    {
        // return periodicData.botPoseWPIBlue[TOTAL_LATENCY_INDEX]; // MegaTag1
        return periodicData.botPoseOrbWPIBlue[TOTAL_LATENCY_INDEX]; // MegaTag2
    }

    //** @return the number of tags visible */
    public int getTagCount()
    {
        // return (int) periodicData.botPoseWPIBlue[TAG_COUNT_INDEX];   // MT1
        return (int) periodicData.botPoseOrbWPIBlue[TAG_COUNT_INDEX];   // MT2
    }

    /** @return the average distance from the target in meters */
    public double getAverageDistanceFromTarget()
    {
        // return periodicData.botPoseWPIBlue[AVERAGE_TAG_DISTANCE_FROM_CAMERA_INDEX];
        return periodicData.botPoseOrbWPIBlue[AVERAGE_TAG_DISTANCE_FROM_CAMERA_INDEX];
    }

    @Override
    public void readPeriodicInputs() 
    {
        periodicData.botPoseWPIBlue = periodicData.botpose_wpiblue.getDoubleArray(new double[11]);
        periodicData.botPoseOrbWPIBlue = periodicData.botpose_orb_wpiblue.getDoubleArray(new double[11]);
    }

    @Override
    public void writePeriodicOutputs() 
    {
        // LL publishes a 3D pose in a weird format, so to make it readable
        // in AS we need to create our own double array and publish that
        megaTag1Pose[0] = periodicData.botPoseWPIBlue[TRANSLATION_X_METERS_INDEX];
        megaTag1Pose[1] = periodicData.botPoseWPIBlue[TRANSLATION_Y_METERS_INDEX];
        megaTag1Pose[2] = periodicData.botPoseWPIBlue[ROTATION_YAW_DEGREES_INDEX];

        megaTag2Pose[0] = periodicData.botPoseOrbWPIBlue[TRANSLATION_X_METERS_INDEX];
        megaTag2Pose[1] = periodicData.botPoseOrbWPIBlue[TRANSLATION_Y_METERS_INDEX];
        megaTag2Pose[2] = periodicData.botPoseOrbWPIBlue[ROTATION_YAW_DEGREES_INDEX];
        
        // put the pose from LL onto the Network Table so AdvantageScope can read it
        // ASTable.getEntry(cameraName).setDoubleArray(poseForAS);
        periodicData.megaTag1Entry.set(megaTag1Pose);
        periodicData.megaTag2Entry.set(megaTag2Pose);
    }

    @Override
    public void runPeriodicTask()
    {
    }
}
