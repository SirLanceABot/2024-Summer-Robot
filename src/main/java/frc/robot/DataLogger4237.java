package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.invoke.MethodHandles;
import java.util.HashMap;
import java.util.stream.Collectors;

public final class DataLogger4237 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private static NetworkTableInstance nti = NetworkTableInstance.getDefault();
    private static DataLog log;

    // Command Scheduler Logs
    private static final HashMap<String, Integer> currentCommands = new HashMap<String, Integer>();
    private static StringEntry initializeCommandLogEntry;
    private static StringEntry interruptCommandLogEntry;
    private static StringEntry finishCommandLogEntry;
    private static StringEntry executeCommandLogEntry;

    private static boolean useDataLog = true;
    private static boolean useConsole = false;
    private static boolean useShuffleboardLog = false;

    private DataLogger4237()
    {}

    public static void start()
    {
        DataLogManager.logNetworkTables(false); // calls DataLogManager.start();
        log = DataLogManager.getLog();
        
        config4237Logs();
        configNetworkTableLog();
        configCameraLog();
        configCommandSchedulerLog();
        // PathPlannerLogging.logCurrentPose(null);
    }

    public static void displayCommandsToConsole(boolean useConsole)
    {
        DataLogger4237.useConsole = useConsole;
    }

    public static void displayCommandsToShuffleboardLog(boolean useShuffleboardLog)
    {
        DataLogger4237.useShuffleboardLog = useShuffleboardLog;
    }

    public NetworkTable getTeam4237Table()
    {
        return NetworkTableInstance.getDefault().getTable(Constants.NETWORK_TABLE_NAME);
    }
    
    private static void config4237Logs()
    {
        nti.startEntryDataLog(log, "/" + Constants.NETWORK_TABLE_NAME, "NT:/" + Constants.NETWORK_TABLE_NAME);
        nti.startEntryDataLog(log, "/" + Constants.ADVANTAGE_SCOPE_TABLE_NAME, "NT:/" + Constants.ADVANTAGE_SCOPE_TABLE_NAME);
    }

    private static void configNetworkTableLog()
    {
        // Log joystick values
        DriverStation.startDataLog(log, true);

        // Log Network Table values
        nti.startEntryDataLog(log, "/FMSInfo", "NT:/FMSInfo");
        // nti.startEntryDataLog(log, "/SmartDashboard", "NT:/SmartDashboard");
        // nti.startEntryDataLog(log, "/Shuffleboard", "NT:/Shuffleboard");
        // nti.startEntryDataLog(log, "/LiveWindow", "NT:/LiveWindow");
        nti.startConnectionDataLog(log, "NTConnection");
    }

    private static void configCameraLog()
    {
        nti.startEntryDataLog(log, "/" + Constants.Camera.CAMERA_1_BOT_POSE, "NT:/" + Constants.Camera.CAMERA_1_BOT_POSE);
        nti.startEntryDataLog(log, "/" + Constants.Camera.CAMERA_2_BOT_POSE, "NT:/" + Constants.Camera.CAMERA_2_BOT_POSE);
        nti.startEntryDataLog(log, "/" + Constants.Camera.CAMERA_3_BOT_POSE, "NT:/" + Constants.Camera.CAMERA_3_BOT_POSE);
        nti.startEntryDataLog(log, "/" + Constants.Camera.CAMERA_4_BOT_POSE, "NT:/" + Constants.Camera.CAMERA_4_BOT_POSE);
    }

    private static void configCommandSchedulerLog()
    {
        NetworkTable nt; 
        
        nt = nti.getTable(Constants.NETWORK_TABLE_NAME);
        initializeCommandLogEntry = nt.getStringTopic("Commands/initialize").getEntry("");
        interruptCommandLogEntry = nt.getStringTopic("Commands/interrupt").getEntry("");
        finishCommandLogEntry = nt.getStringTopic("Commands/finish").getEntry("");
        executeCommandLogEntry = nt.getStringTopic("Commands/execute").getEntry("");

        configCommandInitializeLog();
        configCommandInterruptLog();
        configCommandFinishLog();
        configCommandExecuteLog();
    }

    /**
     * Log commands that run the initialize method.
     */
    private static void configCommandInitializeLog()
    {
        CommandScheduler.getInstance().onCommandInitialize(
            (command) -> 
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String requirements = command.getRequirements().stream()
                    .map(subsystem -> subsystem.getClass().getSimpleName())
                    .collect(Collectors.joining(", ", "{", "}"));

                if(useConsole)
                {
                    System.out.println("Command initialized : " + key + " " + requirements);                    
                }
                if(useDataLog)
                {
                    initializeCommandLogEntry.set(key + " " + requirements);                    
                }
                if(useShuffleboardLog)
                {
                    Shuffleboard.addEventMarker("Command initialized",
                        key + " " + requirements, EventImportance.kNormal);                    
                }

                currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that have been interrupted.
     */
    private static void configCommandInterruptLog()
    {
        CommandScheduler.getInstance().onCommandInterrupt(
            (command) ->
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String runs = " after " + currentCommands.getOrDefault(key, 0) + " runs";

                if(useConsole)
                {
                    System.out.println("Command interrupted : " + key + runs);                    
                }
                if(useDataLog)
                {
                    interruptCommandLogEntry.set(key + runs);                    
                } 
                if(useShuffleboardLog)
                {
                    Shuffleboard.addEventMarker("Command interrupted", key, EventImportance.kNormal);
                }

                currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that run the finish method.
     */
    private static void configCommandFinishLog()
    {
        CommandScheduler.getInstance().onCommandFinish(
            (command) ->
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String runs = " after " + currentCommands.getOrDefault(key, 0) + " runs";

                if(useConsole)
                {
                    System.out.println("Command finished : " + key + runs);                    
                }
                if(useDataLog)
                {
                    finishCommandLogEntry.set(key + runs);                    
                } 
                if(useShuffleboardLog)
                {
                    Shuffleboard.addEventMarker("Command finished", key, EventImportance.kNormal);                    
                }

                currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that run the execute() method.
     * 
     * <p>This can generate a lot of events so logging is suppressed except for the first
     * occurrence of execute(). Total count of execute() is logged at command end.
     * 
     * <p>Recompile without the if/else to get all execute() logged.
     */
    private static void configCommandExecuteLog()
    {
        CommandScheduler.getInstance().onCommandExecute(
            (command) ->
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();

                if(currentCommands.getOrDefault(key, 0) == 0) // suppress all but first execute
                {
                    if(useConsole)
                    {
                        System.out.println("Command executed : " + key);                        
                    }
                    if(useDataLog)
                    {
                        executeCommandLogEntry.set(key);             
                    }
                    if(useShuffleboardLog)
                    {
                        Shuffleboard.addEventMarker("Command executed", key, EventImportance.kNormal);                        
                    }

                    currentCommands.put(key, 1); // first time through count is 1
                }
                else
                {
                    // Increment total count to log when the command ends.
                    currentCommands.put(key, currentCommands.get(key) + 1);
                }
            }
        );
    }
}
