package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Candle4237.LEDColor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Degrees;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public final class Commands4237
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



    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private static RobotContainer robotContainer = null;

    // private static double targetPivotAngle;
    // private static double targetFlywheelSpeed;
    // private static double targetDrivetrainRotation;

    private static double distanceToSpeaker;
    private static double distanceToAmpZone;
   
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here
    private Commands4237(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    public static void setRobotContainer(RobotContainer robotContainer)
    {
        if(Commands4237.robotContainer == null)
            Commands4237.robotContainer = robotContainer;
    }

    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    public static Command intakeFromFloorFront()
    {
        
        //TODO: Add timeout in case of sensor failure until we can test live
        if(robotContainer.intake != null && robotContainer.shuttle != null && robotContainer.index != null && robotContainer.indexWheelsProximity != null && robotContainer.firstShuttleProximity != null)
        {
            return
            Commands.either(
                Commands.none(),

                setCandleCommand(LEDColor.kBlue)
                .andThen(
                    robotContainer.intakePositioning.moveDownCommand())
                .andThen(
                    Commands.waitUntil(robotContainer.intakePositioning.isIntakeDownSupplier()).withTimeout(0.5))
                .andThen(
                    robotContainer.pivot.resetAngleControl())
                .andThen(
                    Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                    .deadlineWith(
                        robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.DEFAULT_ANGLE)
                            .finallyDo(robotContainer.pivot::stopMotor)))
                .andThen(
                    Commands.waitUntil(robotContainer.indexWheelsProximity.isDetectedSupplier())
                    .deadlineWith(
                        robotContainer.intake.pickupFrontCommand(),
                        robotContainer.shuttle.moveUpwardCommand(),
                        robotContainer.index.acceptNoteFromShuttleCommand(),
                        setCandleCommand(LEDColor.kGreen)
                            .onlyIf(
                                robotContainer.firstShuttleProximity.isDetectedSupplier())
                            .repeatedly()))
                .andThen(
                    Commands.parallel(
                        robotContainer.index.stopCommand(),
                        robotContainer.intakePositioning.moveUpCommand(),
                        robotContainer.intake.stopCommand(),
                        robotContainer.shuttle.stopCommand()))
                .handleInterrupt( () -> setCandleDefaultRed())
                .withName("Intake From Floor Front"),

                robotContainer.indexWheelsProximity.isDetectedSupplier());
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command intakeFromFloorBack()
    {
        if(robotContainer.intake != null && robotContainer.shuttle != null && robotContainer.index != null && robotContainer.indexWheelsProximity != null)
        {
            return

            Commands.either(
                Commands.none(),

                setCandleCommand(LEDColor.kBlue)
                .andThen(
                    robotContainer.intakePositioning.moveDownCommand())
                .andThen(
                    Commands.waitUntil(robotContainer.intakePositioning.isIntakeDownSupplier()).withTimeout(0.5))
                .andThen(
                    robotContainer.pivot.resetAngleControl())
                .andThen(
                    Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                    .deadlineWith(
                        robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.DEFAULT_ANGLE)
                            .finallyDo(robotContainer.pivot::stopMotor)))
                .andThen(
                    Commands.waitUntil(robotContainer.indexWheelsProximity.isDetectedSupplier())
                    .deadlineWith(
                        robotContainer.intake.pickupBackCommand(),
                        robotContainer.shuttle.moveUpwardCommand(),
                        robotContainer.index.acceptNoteFromShuttleCommand()))
                .andThen(
                    Commands.parallel(
                        robotContainer.index.stopCommand(),
                        robotContainer.intakePositioning.moveUpCommand(),
                        robotContainer.intake.stopCommand(),
                        robotContainer.shuttle.stopCommand()))
                .handleInterrupt( () -> setCandleDefaultRed())
                .withName("Intake From Floor Back"),

                robotContainer.indexWheelsProximity.isDetectedSupplier());
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command burpNoteCommand()
    {
        if(robotContainer.flywheel != null)
        {
            return
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                robotContainer.flywheel.shootCommand(() -> 10.0))
                .withTimeout(3.0)
            .andThen(
                Commands.parallel(
                    robotContainer.flywheel.stopCommand(),
                    setCandleCommand(LEDColor.kRed)))
            .withName("Burp Note");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command intakeFromSource()
    {
        
        if(robotContainer.flywheel != null && robotContainer.indexWheelsProximity != null && robotContainer.pivot != null)
        { 
            return
            setCandleCommand(LEDColor.kYellow)
            .andThen(
            Commands.waitSeconds(1.0)
            .deadlineWith(
                Commands.parallel(
                    robotContainer.flywheel.intakeCommand(),
                    robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.INTAKE_FROM_SOURCE_ANGLE)
                        .finallyDo(robotContainer.pivot::stopMotor))))
            .andThen(
                Commands.waitUntil(robotContainer.indexWheelsProximity.isDetectedSupplier()))
            .andThen(
                Commands.parallel(
                    Commands.waitSeconds(1.0),
                    setCandleCommand(LEDColor.kWhite)))
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                .deadlineWith(
                    robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.DEFAULT_ANGLE)
                        .finallyDo(robotContainer.pivot::stopMotor)))   
            .andThen(
                Commands.parallel(
                    robotContainer.flywheel.stopCommand(),
                    setCandleCommand(LEDColor.kGreen)))
            .handleInterrupt( () -> setCandleDefaultRed())
            .withName("Intake From Source");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command getFlywheelToSpeedCommand(double speed)
    {
        if(robotContainer.flywheel != null)
        {
            return
            setCandleCommand(LEDColor.kBlue)
            .andThen(
                robotContainer.flywheel.shootCommand(() -> speed))
            .handleInterrupt( () -> setCandleDefaultRed())
            .withName("Get Flywheel To Speed");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command getPivotToSourceShootAngleCommand()
    {
        if(robotContainer.pivot != null)
        {
            return
            robotContainer.pivot.resetAngleControl()
            .andThen(
                Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                .deadlineWith(
                    robotContainer.pivot.setAngle( () -> 48.0)
                        .finallyDo(robotContainer.pivot::stopMotor)))
            .withName("Get Pivot Angle to Source Shoot Angle");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command getPivotToStageShootAngleCommand()
    {
        if(robotContainer.pivot != null)
        {
            return
            robotContainer.pivot.resetAngleControl()
            .andThen(
                Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                .deadlineWith(
                    robotContainer.pivot.setAngle( () -> 44.0)
                        .finallyDo(robotContainer.pivot::stopMotor)))
            .withName("Get Pivot Angle to Stage Shoot Angle");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command setAngleToCorrectSpeakerCommand()
    {
        double distance = 0.0;
        double angle = 0.0;
        // distance = (Math.round((distance * 39.37 / 12.0)));
        
        if(robotContainer.drivetrain != null && robotContainer.pivot != null)
        {
            return
            Commands.either(
                robotContainer.pivot.resetAngleControl()
                .andThen(
                    Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                    .deadlineWith(
                    robotContainer.pivot.setAngle(() -> (
                        robotContainer.pivot.calculateShootAngleFromDistance(() -> (
                            robotContainer.drivetrain.getDistanceToRedSpeaker() * 3.2808))))
                                .finallyDo(robotContainer.pivot::stopMotor))),

                robotContainer.pivot.resetAngleControl()
                .andThen(
                    Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                    .deadlineWith(
                    robotContainer.pivot.setAngle(() -> (
                        robotContainer.pivot.calculateShootAngleFromDistance(() -> (
                            robotContainer.drivetrain.getDistanceToBlueSpeaker() * 3.2808))))
                                .finallyDo(robotContainer.pivot::stopMotor))),

                robotContainer.isRedAllianceSupplier());
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command resetPivotAngleCommand()
    {
        if(robotContainer.pivot != null)
        {
            return
            Commands.waitSeconds(2.0)
            .deadlineWith(
                robotContainer.pivot.moveDown())
            .andThen(
                robotContainer.pivot.stop())
            .andThen(
                robotContainer.pivot.resetMotorEncoderCommand());
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command setFlywheelToCorrectVelocityCommand()
    {
        DoubleSupplier distance;
        DoubleSupplier velocity;
        // distance = (Math.round((distance * 39.37 / 12.0)));
        
        if(robotContainer.drivetrain != null && robotContainer.flywheel != null)
        {
            return
            Commands.either(
                robotContainer.flywheel.shootCommand( () -> (
                    robotContainer.flywheel.calculateShootVelocityFromDistance(() -> (
                        robotContainer.drivetrain.getDistanceToRedSpeaker() * 3.2808)))),

                robotContainer.flywheel.shootCommand( () -> (
                    robotContainer.flywheel.calculateShootVelocityFromDistance(() -> (
                        robotContainer.drivetrain.getDistanceToBlueSpeaker() * 3.2808)))),

                robotContainer.isRedAllianceSupplier());

        }
        else
        {
            return Commands.none();
        }
    }

    public static Command rotateToSpeakerCommand()
    {
        if(robotContainer.drivetrain !=null)
        {
            return
            Commands.either(
                robotContainer.drivetrain.rotateToRedSpeakerCommand()
                .until(
                    robotContainer.drivetrain.isAlignedWithRedSpeaker())
                .withTimeout(1.0)
                .withName("Rotate to Red Speaker"),

                robotContainer.drivetrain.rotateToBlueSpeakerCommand()
                .until(
                    robotContainer.drivetrain.isAlignedWithBlueSpeaker())
                .withTimeout(1.0)
                .withName("Rotate to Blue Speaker"),

                robotContainer.isRedAllianceSupplier());
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command shootFromSubWooferCommand()
    {
        if(robotContainer.pivot != null && robotContainer.index  != null && robotContainer.flywheel != null)
        {
            return
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.waitUntil(isReadyToShoot(63.5, 65.0, 1.0, 3.0))
                                        .withTimeout(1.0)
                .deadlineWith(
                    robotContainer.flywheel.shootCommand(() -> 65.0),
                    robotContainer.pivot.setAngle(() -> 63.5)
                        .finallyDo(robotContainer.pivot::stopMotor)))
            .andThen(
                Commands.waitSeconds(0.5)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelCommand()))
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.parallel(
                    robotContainer.flywheel.stopCommand(),

                    Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                    .deadlineWith(
                        robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.DEFAULT_ANGLE)
                            .finallyDo(robotContainer.pivot::stopMotor)),
                            
                    robotContainer.index.stopCommand(),
                    setCandleCommand(LEDColor.kRed)))
            .withName("Subwoofer Shoot");
        }
        else
        {
            return Commands.none();
        }
    }

    // Timeout might need tobe increased to 3 seconds
    public static Command shootFromAnywhereCommand()
    {
        if(robotContainer.pivot != null && robotContainer.index  != null && robotContainer.flywheel != null && robotContainer.drivetrain != null)
        {
            return
            robotContainer.drivetrain.stopCommand()
            .andThen(
                Commands.waitUntil(robotContainer.drivetrain.isStopped()))
            .andThen(
                Commands.parallel(
                    setCandleCommand(LEDColor.kPurple),
                    Commands.runOnce(() -> setDistanceToSpeaker(() -> robotContainer.drivetrain.getDistanceToSpeaker()))))
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.parallel(
                    Commands.waitUntil(isRotatedAndReadyToShoot(
                        robotContainer.pivot.calculateShootAngleFromDistance(getDistanceToSpeaker()),
                     robotContainer.flywheel.calculateShootVelocityFromDistance(getDistanceToSpeaker()),
                      robotContainer.drivetrain.getAngleToSpeaker(),
                       robotContainer.pivot.classConstants.DEFAULT_ANGLE_TOLERANCE.in(Degrees),
                        robotContainer.flywheel.DEAULT_SPEED_TOLERANCE,
                         1.0))
                                                .withTimeout(1.0)
                    .deadlineWith(

                        robotContainer.flywheel.shootCommand(() -> robotContainer.flywheel.calculateShootVelocityFromDistance(getDistanceToSpeaker())),

                        robotContainer.pivot.setAngle(() -> robotContainer.pivot.calculateShootAngleFromDistance(getDistanceToSpeaker()))
                            .finallyDo(robotContainer.pivot::stopMotor),

                        robotContainer.drivetrain.rotateToSpeakerCommand())))             
            .andThen(
                Commands.waitSeconds(0.5)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelCommand()))
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.parallel(
                    robotContainer.flywheel.stopCommand(),

                    Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                    .deadlineWith(
                        robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.DEFAULT_ANGLE)
                            .finallyDo(robotContainer.pivot::stopMotor)),

                    robotContainer.index.stopCommand(),
                    setCandleCommand(LEDColor.kRed)))
            .withName("Shoot From Anywhere");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command passToAmpZoneCommand()
    {
        if(robotContainer.pivot != null && robotContainer.index  != null && robotContainer.flywheel != null && robotContainer.drivetrain != null)
        {
            return
            Commands.parallel(
                setCandleCommand(LEDColor.kPurple),
                Commands.runOnce(() -> setDistanceToAmpZone(() -> robotContainer.drivetrain.getDistanceToAmpZone())))
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.parallel(
                    Commands.waitUntil(isReadyToShoot(
                        robotContainer.pivot.calculatePassAngleFromDistance(getDistanceToAmpZone()),
                     robotContainer.flywheel.calculatePassVelocityFromDistance(getDistanceToAmpZone()),
                      robotContainer.pivot.classConstants.DEFAULT_ANGLE_TOLERANCE.in(Degrees),
                       robotContainer.flywheel.DEAULT_SPEED_TOLERANCE))
                                                .withTimeout(1.0)
                    .deadlineWith(
                        robotContainer.flywheel.shootCommand(() -> robotContainer.flywheel.calculatePassVelocityFromDistance(getDistanceToAmpZone())),

                        Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                        .deadlineWith(
                            robotContainer.pivot.setAngle(() -> robotContainer.pivot.calculatePassAngleFromDistance(getDistanceToAmpZone()))
                                .finallyDo(robotContainer.pivot::stopMotor)))))
            .andThen(
                Commands.waitSeconds(0.5)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelCommand()))
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.parallel(
                    robotContainer.flywheel.stopCommand(),

                    Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                    .deadlineWith(
                        robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.DEFAULT_ANGLE)
                            .finallyDo(robotContainer.pivot::stopMotor)),

                    robotContainer.index.stopCommand(),
                    setCandleCommand(LEDColor.kRed)))
            .withName("Pass To Amp");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command raiseClimbToChainCommand()
    {
        if(robotContainer.climb != null && robotContainer.intakePositioning != null && robotContainer.pivot != null)
        {
            return
            robotContainer.pivot.resetAngleControl()
            .andThen(
                Commands.parallel(
                    // robotContainer.candle.setRainbowCommand(),
                    setCandleCommand(LEDColor.kRainbow),
                    robotContainer.climb.moveToChainCommand(),
                    robotContainer.intakePositioning.moveUpCommand(),

                    Commands.waitUntil(robotContainer.pivot.isAtAngle()).withTimeout(0.5)
                    .deadlineWith(
                        robotContainer.pivot.setAngle( () -> 25.0)
                            .finallyDo(robotContainer.pivot::stopMotor))));
                    // Commands.run(() -> robotContainer.candle.setRainbow()));
        }
        else
        {
            return
            Commands.none();
        }
    }

    public static Command extendAmpCommand()
    {
        if(robotContainer.pivot != null && robotContainer.ampAssist != null)
        {
            return
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                robotContainer.pivot.resetAngleControl())

            .andThen(
                Commands.waitUntil(robotContainer.pivot.isAtAngle())
                                    .withTimeout(1.0)
                .deadlineWith(
                    robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.SHOOT_TO_AMP_ANGLE)
                        .finallyDo(robotContainer.pivot::stopMotor)))
                        
            .andThen(
                robotContainer.ampAssist.extendCommand())
            .withName("Extend AmpAssist to Shoot Command");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command shootToAmpCommand()
    {
        if(robotContainer.pivot != null && robotContainer.flywheel != null && robotContainer.index != null)
        {
            // 14 ft per sec
            return
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                Commands.runOnce(() -> robotContainer.flywheel.removeDefaultCommand()))
            .andThen(
                Commands.waitUntil(() -> (robotContainer.flywheel.isAtSpeed(13.0).getAsBoolean()))
                                        .withTimeout(1.0)
                .deadlineWith(
                    robotContainer.flywheel.shootCommand(() -> 13.0)))
            .andThen(
                Commands.waitSeconds(0.5)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelAmpCommand()))
            .andThen(
                    // robotContainer.flywheel.stopCommand(),
                    // robotContainer.pivot.setAngleCommand(() -> robotContainer.pivot.classConstants.DEFAULT_ANGLE),
                    robotContainer.index.stopCommand())
                    // setCandleCommand(LEDColor.kRed)))
            .withName("Shoot to amp Command");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command retractAmpCommand()
    {
        if(robotContainer.pivot != null && robotContainer.ampAssist != null)
        {
            return
            // setCandleCommand(LEDColor.kPurple)
            // .andThen(
            Commands.parallel(
                robotContainer.ampAssist.retractCommand(),
                Commands.waitSeconds(0.5))
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.waitUntil(() -> robotContainer.pivot.isAtAngle().getAsBoolean())
                                        .withTimeout(1.0)
                .deadlineWith(
                    Commands.parallel(
                        robotContainer.flywheel.stopCommand(),

                        Commands.waitUntil(robotContainer.pivot.isAtAngle())
                        .deadlineWith(
                            robotContainer.pivot.setAngle(() -> robotContainer.pivot.classConstants.DEFAULT_ANGLE)
                                .finallyDo(robotContainer.pivot::stopMotor)))))
                                
            .andThen(
                Commands.parallel(
                    setCandleCommand(LEDColor.kRed),
                    Commands.runOnce(() -> robotContainer.flywheel.setDefaultCommand(robotContainer.flywheel.stopCommand()))))
            .withName("Extend AmpAssist to Shoot Command");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command moveAmpAssistCommand()
    {
        if(robotContainer.ampAssist != null)
        {
            return
            Commands.either(
                extendAmpCommand(), 
                retractAmpCommand(), 
                robotContainer.ampAssist.isAmpAssistRetracted())
            .withName("Move AmpAssist Command");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command driveToAmpCommand()
    {
        Pose2d ampPose = new Pose2d(1.83, 7.70, new Rotation2d(-90.0));
        PathPlannerPath pathToAmp = PathPlannerPath.fromPathFile("to Amp");
        PathConstraints constraints = new PathConstraints(
            1.5, 4.5,
            Units.degreesToRadians(1080), Units.degreesToRadians(1728));

        if(robotContainer.drivetrain != null && robotContainer.pivot != null && robotContainer.ampAssist != null)
        {
            return
            AutoBuilder.pathfindThenFollowPath(pathToAmp, constraints);
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command autonomousShootCommand()
    {
        if(robotContainer.pivot != null && robotContainer.index  != null && robotContainer.flywheel != null && robotContainer.drivetrain != null && robotContainer.indexWheelsProximity != null)
        {
            return
            Commands.either(
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.waitUntil(isReadyToShoot(48.0, 60.0))
                                                .withTimeout(1.0)
                .deadlineWith(
                    robotContainer.flywheel.shootCommand(() -> 60.0),
                    robotContainer.pivot.setAngle(() -> 48.0)
                        .finallyDo(robotContainer.pivot::stopMotor)))
            .andThen(
                Commands.waitSeconds(0.2)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelCommand()))
            .andThen(
                Commands.parallel(
                    robotContainer.index.stopCommand(),
                    setCandleCommand(LEDColor.kRed))),
                
                Commands.waitSeconds(0.1),

                robotContainer.indexWheelsProximity.isDetectedSupplier()
            )
            .withName("Autonomous Shoot");
        }
        else
        {
            return Commands.waitSeconds(0.1);

        }
    }

    public static Command autonomousStageShootCommand()
    {
        if(robotContainer.pivot != null && robotContainer.index  != null && robotContainer.flywheel != null && robotContainer.drivetrain != null && robotContainer.indexWheelsProximity != null)
        {
            return
            Commands.either(
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.waitUntil(isReadyToShoot(44.0, 55.0))
                                                .withTimeout(1.0)
                .deadlineWith(
                    robotContainer.flywheel.shootCommand(() -> 55.0),
                    robotContainer.pivot.setAngle(() -> 42.0)
                        .finallyDo(robotContainer.pivot::stopMotor)))
            .andThen(
                Commands.waitSeconds(0.2)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelCommand()))
            .andThen(
                Commands.parallel(

                    robotContainer.index.stopCommand(),

                    setCandleCommand(LEDColor.kRed))),
                
                Commands.waitSeconds(0.1),

                robotContainer.indexWheelsProximity.isDetectedSupplier()
            )
            .withName("Autonomous Shoot");
        }
        else
        {
            return Commands.waitSeconds(0.1);

        }
    }

    public static Command autonomousAmpShootCommand()
    {
        if(robotContainer.pivot != null && robotContainer.index  != null && robotContainer.flywheel != null && robotContainer.drivetrain != null && robotContainer.indexWheelsProximity != null)
        {
            return
            Commands.either(
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.waitUntil(isReadyToShoot(44.0, 55.0))
                                                .withTimeout(1.0)
                .deadlineWith(
                    robotContainer.flywheel.shootCommand(() -> 55.0),
                    robotContainer.pivot.setAngle(() -> 44.0)
                        .finallyDo(robotContainer.pivot::stopMotor)))
            .andThen(
                Commands.waitSeconds(0.5)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelCommand()))
            .andThen(
                Commands.parallel(
                    robotContainer.index.stopCommand(),

                    setCandleCommand(LEDColor.kRed))),
                
                    Commands.waitSeconds(0.1),

                robotContainer.indexWheelsProximity.isDetectedSupplier()
            )
            .withName("Autonomous Shoot");
        }
        else
        {
            return Commands.waitSeconds(0.1);
        }
    }

    public static Command autonomousFirstShootCommand()
    {
        if(robotContainer.pivot != null && robotContainer.index  != null && robotContainer.flywheel != null && robotContainer.drivetrain != null && robotContainer.indexWheelsProximity != null)
        {
            return
            Commands.either(
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.waitUntil(isReadyToShoot(46.0, 60.0))
                                        .withTimeout(1.0)
                .deadlineWith(
                    robotContainer.flywheel.shootCommand(() -> 60.0),
                    robotContainer.pivot.setAngle(() -> 46.0)
                        .finallyDo(robotContainer.pivot::stopMotor)))
            .andThen(
                Commands.waitSeconds(0.2)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelCommand()))
            .andThen(
                Commands.parallel(
                    robotContainer.index.stopCommand(),

                    setCandleCommand(LEDColor.kRed))),
                
                Commands.waitSeconds(0.1),


                robotContainer.indexWheelsProximity.isDetectedSupplier()
            )
            .withName("Autonomous Shoot");
        }
        else
        {
            return Commands.waitSeconds(0.1);
        }
    }

    public static Command autonomousShootFromSubWooferCommand()
    {
        if(robotContainer.index  != null && robotContainer.flywheel != null && robotContainer.pivot != null)
        {
            return
            setCandleCommand(LEDColor.kPurple)
            .andThen(
                robotContainer.pivot.resetAngleControl())
            .andThen(
                Commands.waitUntil(isReadyToShoot(64.0, 65.0, 1.0, 3.0))
                                        .withTimeout(1.0)
                .deadlineWith(
                    robotContainer.flywheel.shootCommand(() -> 65.0),
                    robotContainer.pivot.setAngle(() -> 64.0)
                        .finallyDo(robotContainer.pivot::stopMotor)))    
            .andThen(
                Commands.waitSeconds(0.5)
                .deadlineWith(
                    robotContainer.index.feedNoteToFlywheelCommand()))
            .andThen(
                Commands.parallel(
                    robotContainer.index.stopCommand(),
                    setCandleCommand(LEDColor.kRed)))
            .withName("Subwoofer Shoot");
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command autonomousFinishIntakeCommand()
    {
        if(robotContainer.intake != null && robotContainer.shuttle != null && robotContainer.index != null && robotContainer.indexWheelsProximity != null && robotContainer.firstShuttleProximity != null)
        {
            return
            Commands.either(
                Commands.none(),

            setCandleCommand(LEDColor.kBlue)
            .andThen(
                Commands.waitUntil(robotContainer.indexWheelsProximity.isDetectedSupplier())
                .deadlineWith(
                    robotContainer.intake.pickupFrontCommand(),
                    robotContainer.shuttle.moveUpwardCommand(),
                    robotContainer.index.acceptNoteFromShuttleCommand(),
                    setCandleCommand(LEDColor.kGreen)
                        .onlyIf(
                        robotContainer.firstShuttleProximity.isDetectedSupplier())
                        .repeatedly()
                    ))
            .andThen(
                Commands.parallel(
                    robotContainer.index.stopCommand(),
                    robotContainer.intake.stopCommand(),
                    robotContainer.shuttle.stopCommand()))
            .withTimeout(1.5)
            .withName("Autonomous Finish Intake"),

            robotContainer.indexWheelsProximity.isDetectedSupplier());
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command autonomousFinishIntakeSlowlyCommand()
    {
        if(robotContainer.intake != null && robotContainer.shuttle != null && robotContainer.index != null && robotContainer.indexWheelsProximity != null && robotContainer.firstShuttleProximity != null)
        {
            return
            Commands.either(
                Commands.none(),

            setCandleCommand(LEDColor.kBlue)
            .andThen(
                Commands.waitUntil(robotContainer.indexWheelsProximity.isDetectedSupplier())
                .deadlineWith(
                    robotContainer.intake.pickupFrontCommand(),
                    robotContainer.shuttle.moveUpwardCommand(),
                    robotContainer.index.acceptNoteFromShuttleSlowlyCommand(),
                    setCandleCommand(LEDColor.kGreen)
                        .onlyIf(
                        robotContainer.firstShuttleProximity.isDetectedSupplier())
                        .repeatedly()
                    ))
            .andThen(
                Commands.parallel(
                    robotContainer.index.stopCommand(),
                    robotContainer.intake.stopCommand(),
                    robotContainer.shuttle.stopCommand()))
            .withTimeout(1.5)
            .withName("Autonomous Finish Intake"),

            robotContainer.indexWheelsProximity.isDetectedSupplier());
        }
        else
        {
            return Commands.none();
        }
    }

    public static Command autonomousFinishBackIntakeCommand()
    {
        if(robotContainer.intake != null && robotContainer.shuttle != null && robotContainer.index != null && robotContainer.indexWheelsProximity != null && robotContainer.firstShuttleProximity != null)
        {
            return
            Commands.either(
                Commands.none(),

            setCandleCommand(LEDColor.kBlue)
            .andThen(
                Commands.waitUntil(robotContainer.indexWheelsProximity.isDetectedSupplier())
                .deadlineWith(
                    robotContainer.intake.pickupBackCommand(),
                    robotContainer.shuttle.moveUpwardCommand(),
                    robotContainer.index.acceptNoteFromShuttleCommand(),
                    setCandleCommand(LEDColor.kGreen)
                        .onlyIf(
                        robotContainer.firstShuttleProximity.isDetectedSupplier())
                        .repeatedly()
                    ))
            .andThen(
                Commands.parallel(
                    robotContainer.index.stopCommand(),
                    robotContainer.intake.stopCommand(),
                    robotContainer.shuttle.stopCommand()))
            .withTimeout(1.5)
            .withName("Autonomous Finish Intake"),

            robotContainer.indexWheelsProximity.isDetectedSupplier());
        }
        else
        {
            return Commands.none();
        }
    }

    private static void setDistanceToSpeaker(DoubleSupplier distanceToSpeaker)
    {
        Commands4237.distanceToSpeaker = distanceToSpeaker.getAsDouble();
        // System.out.println("Distance: " + Commands4237.distanceToSpeaker);
    }

    private static void setDistanceToAmpZone(DoubleSupplier distanceToAmpZone)
    {
        Commands4237.distanceToAmpZone = distanceToAmpZone.getAsDouble();
        // System.out.println("Distance: " + Commands4237.distanceToAmpZone);
    }

    private static DoubleSupplier getDistanceToSpeaker()
    {
        return () -> Commands4237.distanceToSpeaker;
    }

    private static DoubleSupplier getDistanceToAmpZone()
    {
        return () -> Commands4237.distanceToAmpZone;
    }

    private static BooleanSupplier isReadyToShoot(double targetAngle, double targetSpeed)
    {
        return isReadyToShoot(targetAngle, targetSpeed, robotContainer.pivot.classConstants.DEFAULT_ANGLE_TOLERANCE.in(Degrees), robotContainer.flywheel.DEAULT_SPEED_TOLERANCE);   
    }

    private static BooleanSupplier isReadyToShoot(double targetAngle, double targetSpeed, double angleTolerance, double speedTolerance)
    {
        return () ->
        {
            boolean isAtAngle = false;
            boolean isAtSpeed = false;

            isAtAngle = robotContainer.pivot.isAtAngle().getAsBoolean();
            isAtSpeed = robotContainer.flywheel.isAtSpeed(targetSpeed, speedTolerance).getAsBoolean();

            if(isAtAngle)
            {
                System.out.println(" Angle =" + targetAngle);
                
            }

            if(isAtSpeed)
            {
                System.out.println(" Speed =" + targetSpeed);
                
            }

            return (isAtAngle && isAtSpeed);
        };
        
    }

    private static BooleanSupplier isRotatedAndReadyToShoot(double targetAngle, double targetSpeed, double targetRotation)
    {
        return isRotatedAndReadyToShoot(
            targetAngle,
         targetSpeed,
          targetRotation,
           robotContainer.pivot.classConstants.DEFAULT_ANGLE_TOLERANCE.in(Degrees),
            robotContainer.flywheel.DEAULT_SPEED_TOLERANCE,
             robotContainer.drivetrain.DEFAULT_ALIGNMENT_TOLERANCE);   
    }

    private static BooleanSupplier isRotatedAndReadyToShoot(double targetAngle, double targetSpeed, double targetRotation, double angleTolerance, double speedTolerance, double rotationTolerance)
    {
        return () ->
        {
            boolean isAtAngle = false;
            boolean isAtSpeed = false;
            boolean isAtRotation = false;

            isAtAngle = robotContainer.pivot.isAtAngle().getAsBoolean();
            isAtSpeed = robotContainer.flywheel.isAtSpeed(targetSpeed, speedTolerance).getAsBoolean();
            isAtRotation = robotContainer.drivetrain.isAtRotation(targetRotation, rotationTolerance).getAsBoolean();

            return (isAtAngle && isAtSpeed && isAtRotation);
        };
        
    }

    public static Command resetGyroCommand()
    {
        if(robotContainer.gyro !=null)
        {
            return
            Commands.either(
                robotContainer.gyro.setYawRedCommand(),
                robotContainer.gyro.setYawBlueCommand(), 
                robotContainer.isRedAllianceSupplier());
        }
        else
        {
            return Commands.none();
        }
    }

    //FIXME commands shouldn't call commands
    public static Command setCandleCommand(LEDColor ledColor)
    {
        if(robotContainer.candle != null)
        {
            switch(ledColor)
            {
                case kRed:
                    return robotContainer.candle.setRedCommand();

                case kYellow:
                    return robotContainer.candle.setYellowCommand();

                case kGreen:
                    return robotContainer.candle.setGreenCommand();

                case kBlue:
                    return robotContainer.candle.setBlueCommand();

                case kPurple:
                    return robotContainer.candle.setPurpleCommand();

                case kWhite:
                    return robotContainer.candle.setWhiteCommand();
                    
                case kRainbow:
                    return robotContainer.candle.setRainbowCommand();

                case kOff:
                    return robotContainer.candle.stopCommand();

                default:
                    return Commands.none();
            }
        }
        else
        {
            return 
            Commands.none();
        }
    }

    private static void setCandleDefaultRed()
    {
        if(robotContainer.candle != null)
        {
            robotContainer.candle.setRed(false);
        }
        else
        {
            // do nothing
        }
    } 

    public static Command resetPivotToCANCoderAngleCommand()
    {
        if(robotContainer.pivot != null)
        {
            return 
            robotContainer.pivot.stop()
            .andThen(
                robotContainer.pivot.resetToCANCoderCommand())
            .andThen(
                Commands.waitSeconds(0.5))
            .withName("Reset Pivot Angle");
        }
        else
        {
            return Commands.none();
        }
    }
}