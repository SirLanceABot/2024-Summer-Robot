package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.motors.TalonFX4237;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This class creates a Pivot to set the angle of the flywheel.
 */
public class Pivot extends Subsystem4237
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    
    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here

    @FunctionalInterface
    private interface Function
    {
        public abstract StatusCode apply();
    }

    private class PeriodicData
    {
        // INPUTS
        private double canCoderAbsolutePosition;
        private double canCoderRotationalPosition;
        private double motorEncoderRotationalPosition;

        private DoubleEntry angleEntry;
        private DoubleEntry canCoderAngleEntry;
        
        // private boolean isToggleSwitchActive;
        // private boolean isPIDSet;

        // OUTPUTS
        private boolean isBadAngle = false;
   }

    public class ClassConstants
    {
        //for PID
        private final double kP = 0.75;
        private final double kI = 0.0;
        private final double kD = 0.0;
        private final double kS = 0.1;
        private int slotId = 0;

        //limits
        private final double FORWARD_SOFT_LIMIT = 64.0; //66 degrees is the top
        private final double REVERSE_SOFT_LIMIT = 27.0; //22 degrees is the bottom *add 2 to the limit for correct value

        private final double MAGNET_OFFSET = -0.218994140625;

        public final double DEFAULT_ANGLE = 32.0;
        public final double INTAKE_FROM_SOURCE_ANGLE = 60.0;   //TODO: Check angle
        public final double SHOOT_TO_AMP_ANGLE = 64.0;
        public final double DEFAULT_ANGLE_TOLERANCE = 0.3;

        //for manually moving Pivot
        private final double UP_MOTOR_SPEED = 0.1;
        private final double DOWN_MOTOR_SPEED = -0.1;
    }
    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***

    private StringEntry setupEntry;

    private String canCoderName = "pivotCANcoder";
    private final int SETUP_ATTEMPT_LIMIT = 5;
    private int setupErrorCount = 0;

    private final NetworkTable angleNetworkTable;    
    
    private boolean logPeriodicData = true;

    // Put all class variables and instance variables here
    private final TalonFX4237 motor = new TalonFX4237(Constants.Pivot.MOTOR_PORT, Constants.Pivot.MOTOR_CAN_BUS, "pivotMotor");
    private final CANcoder cancoder = new CANcoder(Constants.Pivot.CANCODER_PORT, Constants.Pivot.CANCODER_CAN_BUS);
    private final PeriodicData periodicData = new PeriodicData();
    public final ClassConstants classConstants = new ClassConstants();
    private PIDController PIDcontroller = new PIDController(classConstants.kP, classConstants.kI, classConstants.kD);
    // private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ks, kv, ka);
    private final InterpolatingDoubleTreeMap speakerAngleShotMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap ampAnglePassMap = new InterpolatingDoubleTreeMap();
    /*add     private Measure<Angle> setpoint; // should this be initialized to current position? */

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Pivot. 
     */
    public Pivot()
    {
        super("Pivot");
        System.out.println("  Constructor Started:  " + fullClassName);

        // cancoderLogEntry = new StringLogEntry(log, "/cancoders/setup", "Setup");
        // doubleLogEntry = new DoubleLogEntry(log, "cancoders/" + canCoderName, "Degrees");
        // SmartDashboard.putNumber("kP", 0.0);

        angleNetworkTable = NetworkTableInstance.getDefault().getTable(Constants.NETWORK_TABLE_NAME);
        setupEntry = angleNetworkTable.getStringTopic("PivotCANcoderSetup").getEntry("");
        periodicData.angleEntry = angleNetworkTable.getDoubleTopic("motorAngle").getEntry(classConstants.DEFAULT_ANGLE);
        periodicData.canCoderAngleEntry = angleNetworkTable.getDoubleTopic("canCoderAngle").getEntry(classConstants.DEFAULT_ANGLE);


        configCANcoder();
        configPivotMotor();
        configShotMap();
        configPassMap();

        PIDcontroller.setTolerance(classConstants.DEFAULT_ANGLE_TOLERANCE);

        
        // setDefaultCommand(setAngleCommand(() -> classConstants.DEFAULT_ANGLE));
        // periodicData.canCoderRotationalPosition = pivotAngle.getPosition().getValueAsDouble();
        // periodicData.motorEncoderRotationalPosition = motor.getPosition();
        // motor.setPosition(periodicData.canCoderRotationalPosition);

        // PIDcontroller.enableContinuousInput(classConstants.REVERSE_SOFT_LIMIT, classConstants.FORWARD_SOFT_LIMIT);
       
        System.out.println(" Construction Finished: " + fullClassName);
    }

    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configCANcoder()
    {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration(); // gets all the factory default values
        // overlay factory defaults with our values as needed; anything not set below will have the factory default from above
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfig.MagnetSensor.MagnetOffset = classConstants.MAGNET_OFFSET;
        setup(() -> cancoder.getConfigurator().apply(canCoderConfig), "Setup CANcoder"); // send all values to the device
    }

    private void configPivotMotor()
    {
        // Factory Defaults
        motor.setupFactoryDefaults();
        motor.setupInverted(true);
        motor.setupBrakeMode();
        /*remove*/motor.setupPositionConversionFactor(200.0 / 360.0); // CTRE says not to use this method for units conversion; not accurate
        /*add motor.setupRemoteCANCoder(Constants.Pivot.CANCODER_PORT);*/
        motor.setSafetyEnabled(false);
        /*remove*/motor.setPosition((cancoder.getAbsolutePosition().getValueAsDouble()) * 360.0);
        motor.setupPIDController(classConstants.slotId, classConstants.kP, classConstants.kI, classConstants.kD);
        
        // Soft Limits
        motor.setupForwardSoftLimit(classConstants.FORWARD_SOFT_LIMIT, false);
        motor.setupReverseSoftLimit(classConstants.REVERSE_SOFT_LIMIT, false);

        //Hard Limits
        motor.setupForwardHardLimitSwitch(true, true);
        motor.setupReverseHardLimitSwitch(true, true);
    }

     /** 
     * Check the CANCoder for an error and print a message.
     * @param message The message to print
     */
    private void setup(Function func, String message)
    {
        StatusCode errorCode = StatusCode.OK;
        int attemptCount = 0;
        String logMessage = "";
        
        do
        {
            errorCode = func.apply();
            logMessage = canCoderName + " : " + message + " " + errorCode;

            if(errorCode == StatusCode.OK)
                System.out.println(">> >> " + logMessage);
            else
                DriverStation.reportWarning(logMessage, true);
            //cancoderLogEntry.append(logMessage);
            setupEntry.set(logMessage);
            attemptCount++;
        }
        while(errorCode != StatusCode.OK && attemptCount < SETUP_ATTEMPT_LIMIT);

        setupErrorCount += (attemptCount - 1);
    }

    private void configShotMap()
    {
        // first value is distance from speaker in feet, second value is the pivot angle in degrees
        // These are for the calculated (poseEstimator) distances
        // speakerAngleShotMap.put(4.0, 64.0);
        // speakerAngleShotMap.put(5.0, 62.8);
        // speakerAngleShotMap.put(6.0, 56.7);
        // speakerAngleShotMap.put(7.0, 52.5);
        // speakerAngleShotMap.put(8.0, 49.9);
        // speakerAngleShotMap.put(9.0, 47.6);
        // speakerAngleShotMap.put(10.0, 44.0);
        // speakerAngleShotMap.put(11.0, 42.2);
        // speakerAngleShotMap.put(12.0, 41.3);
        // speakerAngleShotMap.put(13.0, 40.65);
        // speakerAngleShotMap.put(14.0, 39.64);
        // speakerAngleShotMap.put(15.0, 39.1);
        
        // These are for the real world distance
        // speakerAngleShotMap.put(4.5, 64.0);
        // speakerAngleShotMap.put(5.5, 62.8);
        // speakerAngleShotMap.put(6.5, 56.7);
        // speakerAngleShotMap.put(7.5, 52.5);
        // speakerAngleShotMap.put(8.5, 49.9);
        // speakerAngleShotMap.put(9.5, 47.6);
        // speakerAngleShotMap.put(10.5, 44.0);
        // speakerAngleShotMap.put(11.5, 42.2);
        // speakerAngleShotMap.put(12.5, 41.3);
        // speakerAngleShotMap.put(13.5, 40.65);
        // speakerAngleShotMap.put(14.5, 39.64);
        // speakerAngleShotMap.put(15.5, 39.1);

        // EXTRA FUDGE EDITION
        speakerAngleShotMap.put(3.5, 64.0);
        speakerAngleShotMap.put(4.5, 62.8);
        speakerAngleShotMap.put(5.5, 56.7);
        speakerAngleShotMap.put(6.5, 52.5);
        speakerAngleShotMap.put(7.5, 49.9);
        speakerAngleShotMap.put(8.5, 47.6);
        speakerAngleShotMap.put(9.5, 44.0);
        speakerAngleShotMap.put(10.5, 42.2);
        speakerAngleShotMap.put(11.5, 41.3);
        speakerAngleShotMap.put(12.5, 40.65);
        speakerAngleShotMap.put(13.5, 39.64);
        speakerAngleShotMap.put(14.5, 39.1);

        // angleShotMap.put(5.0, 64.0);
        // angleShotMap.put(6.0, 62.8);
        // angleShotMap.put(7.0, 56.7);
        // angleShotMap.put(8.0, 52.5);
        // angleShotMap.put(9.0, 49.9);
        // angleShotMap.put(10.0, 47.6);
        // angleShotMap.put(11.0, 44.0);
        // angleShotMap.put(12.0, 42.2);
        // angleShotMap.put(13.0, 41.3);
        // angleShotMap.put(14.0, 40.65);
        // angleShotMap.put(15.0, 39.64);
        // angleShotMap.put(16.0, 39.1);
    }

    private void configPassMap()
    {
        ampAnglePassMap.put(27.0, 58.1);
        ampAnglePassMap.put(32.0, 50.3);
        ampAnglePassMap.put(37.0, 47.7);
    }

    public void move(double motorSpeed)
    {
        motor.set(motorSpeed);
    }

    public void stopMotor()
    {
        motor.set(0.0);
    }

    public double getCANCoderAngle()
    {
        // returns an angular value of the CANcoder
        return periodicData.canCoderAbsolutePosition * 360.0;
    }

    public double getCANCoderPosition()
    {
        //returns the position of the CANcoder in rotations
        return periodicData.canCoderRotationalPosition;
    }

    public double getMotorEncoderPosition()
    {
        return periodicData.motorEncoderRotationalPosition;
    }
    
    /*remove method */
    public void setAngleOLD(double degrees)
    {
        //setAngle using CANcoder
        degrees = MathUtil.clamp(degrees, classConstants.REVERSE_SOFT_LIMIT, classConstants.FORWARD_SOFT_LIMIT);
        // System.out.println("Target Angle: " + degrees);
        motor.setControlPosition(degrees);
    }

    /*remove method*/
    public void setAngle(double degrees)
    {
        // PIDcontroller.setP(classConstants.kP);
        double positionDegrees = getCANCoderAngle();
        double pidOutput = PIDcontroller.calculate(positionDegrees, degrees);
        double pivotOutput = MathUtil.clamp(pidOutput + Math.copySign(classConstants.kS, pidOutput), -Constants.END_OF_MATCH_BATTERY_VOLTAGE, Constants.END_OF_MATCH_BATTERY_VOLTAGE);
        motor.setVoltage(pivotOutput);
        // SmartDashboard.putNumber("Pivot Output Motor Power", pivotOutput);
        // SmartDashboard.putNumber("Pivot Angle:", getCANCoderAngle());
        // SmartDashboard.putNumber("Passed Degrees", degrees);
        // SmartDashboard.putNumber("positionDegrees", positionDegrees);
        // SmartDashboard.putNumber("pidOutput", pidOutput);
        // SmartDashboard.putNumber("pivotOutput", pivotOutput);
    }

    /*add method*/
    // /**
    //  * Compatibility method for old style passing without Measure
    //  * @param degrees
    //  */
    // public void setAngle(double degrees)
    // {
    //     setAngle(Degrees.of(degrees));
    // }

    /*add method*/
    // /**
    //  * Progress toward using Measure but still a mish-mash of Measure usage and not
    //  * 
    //  * @param angle
    //  */
    // public void setAngle(Measure<Angle> angle)
    // {
    //     setpoint = angle; // save for others' use
    //     motor.setControlPosition(angle.in(Rotation)); // new position for Talon FX internal PID

    //     SmartDashboard.putNumber("setpoint degrees", angle.in(Degrees));
    //     SmartDashboard.putNumber("cancoder position degrees", getCANCoderAngle());
    //     SmartDashboard.putNumber("motor position degrees", motor.getPosition()*360.); // actually integrated cancoder position
    //     // SmartDashboard.putNumber("motor position degrees", getMotorEncoderPosition()*360.);

    //     // test the "at correct position" function
    //     SmartDashboard.putBoolean("Pivot ready", isAtAngle().getAsBoolean());

    //     // temporary code troubleshooting possible problem - check if magnet offset ever changes
    //     cancoder.getConfigurator().refresh(canCoderConfig);
    //     SmartDashboard.putNumber("magnet offset", canCoderConfig.MagnetSensor.MagnetOffset);
    //     var magnetOffsetOkay = MathUtil.isNear(
    //         classConstants.MAGNET_OFFSET,
    //         canCoderConfig.MagnetSensor.MagnetOffset,
    //         Math.abs(classConstants.MAGNET_OFFSET*0.005),
    //         -1.,
    //         +1.);
    //     SmartDashboard.putBoolean("magnet offset okay", magnetOffsetOkay);
    // }

    public BooleanSupplier isAtAngle()
    {
        /*remove*/return () -> PIDcontroller.atSetpoint();
        /*add         return () -> MathUtil.isNear(setpoint.in(Degrees), getCANCoderAngle(), classConstants.DEFAULT_ANGLE_TOLERANCE) ; */
    }

    /**
     * 
     * @param distance (m)
     * @return angle pivot should move to
     */
    public double calculateShootAngleFromDistance(DoubleSupplier distance)
    {
        // System.out.println(angleShotMap.get(distance.getAsDouble() * 3.2808));
        return speakerAngleShotMap.get(distance.getAsDouble() * 3.2808);
    }

    /**
     * 
     * @param distance (m)
     * @return angle pivot should move to
     */
    public double calculatePassAngleFromDistance(DoubleSupplier distance)
    {
        // System.out.println(angleShotMap.get(distance.getAsDouble() * 3.2808));
        return ampAnglePassMap.get(distance.getAsDouble() * 3.2808);
    }

    
    /*remove this method*/
    public Command setAngleOLDCommand(DoubleSupplier angle)
    {
        return Commands.runOnce(() -> setAngleOLD(angle.getAsDouble()), this).withName("Set Angle");
    }

    public Command setAngle(DoubleSupplier angle)
    {
        return Commands.run(() -> setAngle(angle.getAsDouble()), this).withName("Set Angle");
    }

    /*add this method - change name*/
    // public Command setAngleCommand(Supplier<Measure<Angle>> angle)
    // {
    //     return run(() -> setAngle(angle.get())).withName("Set Angle");
    // }

    public Command moveUp()
    {
        return Commands.run(() -> move(classConstants.UP_MOTOR_SPEED), this);
    }

    public Command moveDown()
    {
        return Commands.run(() -> move(classConstants.DOWN_MOTOR_SPEED), this);
    }

    public Command resetMotorEncoderCommand()
    {
        return Commands.runOnce(() -> motor.setPosition(classConstants.REVERSE_SOFT_LIMIT), this).withName("Reset Pivot Position");
    }

    public Command resetToCANCoderCommand()
    {
        // return Commands.runOnce(() -> motor.setPosition((cancoder.getAbsolutePosition().getValueAsDouble() + classConstants.MAGNET_OFFSET) * 360.0), this).withName("Reset Pivot Position");
        return Commands.runOnce(() -> motor.setPosition((cancoder.getAbsolutePosition().getValueAsDouble()) * 360.0), this).withName("Reset Pivot Position");

    }

    public Command resetAngleControl()
    {
        return Commands.runOnce(PIDcontroller::reset);
    }

    public Command stop()
    {
        return Commands.runOnce(() -> stopMotor());
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void readPeriodicInputs()
    {
        //Using CANcoder
        // periodicData.canCoderAbsolutePosition = cancoder.getAbsolutePosition().getValueAsDouble() + classConstants.MAGNET_OFFSET;
        // periodicData.canCoderRotationalPosition = cancoder.getPosition().getValueAsDouble() + classConstants.MAGNET_OFFSET;
        periodicData.canCoderAbsolutePosition = cancoder.getAbsolutePosition().getValueAsDouble();
        periodicData.canCoderRotationalPosition = cancoder.getPosition().getValueAsDouble();
        periodicData.motorEncoderRotationalPosition = motor.getPosition();
    }

    @Override
    public void writePeriodicOutputs()
    {
        if(periodicData.isBadAngle)
        {
            stopMotor();
            System.out.println("Angle is out of Range");
            periodicData.isBadAngle = false;
        }

        periodicData.angleEntry.set(getMotorEncoderPosition());
        periodicData.canCoderAngleEntry.set(getCANCoderAngle());
        SmartDashboard.putNumber("Pivot CANCODER current Angle", getCANCoderAngle());

    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public String toString()
    {
        return "current pivot angle = " + getCANCoderAngle();
    }
}
