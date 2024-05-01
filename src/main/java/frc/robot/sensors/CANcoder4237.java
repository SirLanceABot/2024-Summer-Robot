package frc.robot.sensors;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.spns.SpnValue;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
// import edu.wpi.first.util.datalog.DataLog;
// import edu.wpi.first.util.datalog.DoubleLogEntry;
// import edu.wpi.first.util.datalog.StringLogEntry;
// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class CANcoder4237 extends Sensor4237
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    @FunctionalInterface
    private interface Function
    {
        public abstract StatusCode doAction();
    }

    private class PeriodicData
    {
        private double relativePosition = 0.0;
        private double absolutePosition = 0.0;
    }


    // *** CLASS & INSTANCE VARIABLES ***
    private final String cancoderName;
    private final CANcoder cancoder;
    private final PeriodicData periodicData;

    private final NetworkTable networkTable;    
    private StringEntry strEntry;
    private DoubleEntry dblEntry;

    // final static DataLog log = DataLogManager.getLog();
    // private StringLogEntry stringLogEntry;
    // private DoubleLogEntry doubleLogEntry;

    private final int SETUP_ATTEMPT_LIMIT = 5;
    private int setupErrorCount = 0;

    private boolean logPeriodicData = false;


    public CANcoder4237(int deviceID, String canbus, String cancoderName)
    {
        super(cancoderName);
        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cancoderName);

        this.cancoderName = cancoderName;

        networkTable = NetworkTableInstance.getDefault().getTable(Constants.NETWORK_TABLE_NAME);
        strEntry = networkTable.getStringTopic("cancoders/setup").getEntry("");
        dblEntry = networkTable.getDoubleTopic("cancoders/values").getEntry(0.0);

        // stringLogEntry = new StringLogEntry(log, "/cancoders/setup/" + cancoderName, "Setup");
        // doubleLogEntry = new DoubleLogEntry(log, "cancoders/values/" + cancoderName, "Degrees");

        cancoder = new CANcoder(deviceID);
        periodicData = new PeriodicData();
        
        System.out.println("  Constructor Finished: " + fullClassName + " >> " + cancoderName);
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
            errorCode = func.doAction();
            logMessage = cancoderName + " : " + message + " " + errorCode;

            if(errorCode == StatusCode.OK)
                System.out.println(">> >> " + logMessage);
            else
                DriverStation.reportWarning(logMessage, true);
            
            strEntry.set(logMessage);
            // stringLogEntry.append(logMessage);
            attemptCount++;
        }
        while(errorCode != StatusCode.OK && attemptCount < SETUP_ATTEMPT_LIMIT);

        setupErrorCount += (attemptCount - 1);
    }
    
    /**
     * Clear all sticky faults.
     */
    public void clearStickyFaults()
    {
        setup(() -> cancoder.clearStickyFaults(), "Clear Sticky Faults");
    }

    public void setupFactoryDefaults()
    {
        setup(() -> cancoder.getConfigurator().apply(new CANcoderConfiguration()), "Setup Factory Defaults");
    }

    public void setupAbsoluteSensorRange_0To1()
    {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        setup(() -> cancoder.getConfigurator().refresh(magnetSensorConfigs), "Refresh AbsoluteSensorRange_0To1");

        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        setup(() -> cancoder.getConfigurator().apply(magnetSensorConfigs), "Setup AbsoluteSensorRange_0To1");
    }

    public void setupAbsoluteSensorRange_PlusMinusHalf()
    {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        setup(() -> cancoder.getConfigurator().refresh(magnetSensorConfigs), "Refresh AbsoluteSensorRange_PlusMinusHalf");

        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        setup(() -> cancoder.getConfigurator().apply(magnetSensorConfigs), "Setup AbsoluteSensorRange_PlusMinusHalf");
    }

    public void setupMagnetOffset(double magnetOffset)
    {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        setup(() -> cancoder.getConfigurator().refresh(magnetSensorConfigs), "Refresh MagnetOffset_" + magnetOffset);

        magnetSensorConfigs.MagnetOffset = magnetOffset;
        setup(() -> cancoder.getConfigurator().apply(magnetSensorConfigs), "Setup MagnetOffset_" + magnetOffset);
    }

    public void setupSensorDirection_ClockwisePositive()
    {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        setup(() -> cancoder.getConfigurator().refresh(magnetSensorConfigs), "Refresh SensorDirection_ClockwisePositive");

        magnetSensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        setup(() -> cancoder.getConfigurator().apply(magnetSensorConfigs), "Setup SensorDirection_ClockwisePositive");
    }

    public void setupSensorDirection_CounterClockwisePositive()
    {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        setup(() -> cancoder.getConfigurator().refresh(magnetSensorConfigs), "Refresh SensorDirection_CounterClockwisePositive");

        magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        setup(() -> cancoder.getConfigurator().apply(magnetSensorConfigs), "Setup SensorDirection_CounterClockwisePositive");
    }

    public double getPosition()
    {
        return periodicData.relativePosition;
    }

    public void setPosition(double position)
    {
        cancoder.setPosition(position);
    }

    public double getAbsolutePosition()
    {
        return periodicData.absolutePosition;
    }

    public void logPeriodicData(boolean isEnabled)
    {
        logPeriodicData = isEnabled;
    }

    /**
     * Logs the sticky faults
     */
    public void logStickyFaults()
    {
        int faultsCount = 0;
        strEntry = networkTable.getStringTopic("cancoders/faults").getEntry("");
        // stringLogEntry = new StringLogEntry(log, "/cancoders/faults", "Faults");

        if(cancoder.getStickyFault_BadMagnet().getValue())
        {
            strEntry.set(cancoderName + " : " + SpnValue.StickyFault_CANCODER_BadMagnet);
            // stringLogEntry.append(cancoderName + " : " + SpnValue.StickyFault_CANCODER_BadMagnet);
            faultsCount++;
        }        

        if(cancoder.getStickyFault_BootDuringEnable().getValue())
        {
            strEntry.set(cancoderName + " : " + SpnValue.StickyFault_BootDuringEnable);
            // stringLogEntry.append(cancoderName + " : " + SpnValue.StickyFault_BootDuringEnable);
            faultsCount++;
        }

        if(cancoder.getStickyFault_Hardware().getValue())
        {
            strEntry.set(cancoderName + " : " + SpnValue.StickyFault_Hardware);
            // stringLogEntry.append(cancoderName + " : " + SpnValue.StickyFault_Hardware);
            faultsCount++;
        }

        if(cancoder.getStickyFault_Undervoltage().getValue())
        {
            strEntry.set(cancoderName + " : " + SpnValue.StickyFault_Undervoltage);
            // stringLogEntry.append(cancoderName + " : " + SpnValue.StickyFault_Undervoltage);
            faultsCount++;
        }

        if(faultsCount == 0)
            strEntry.set(cancoderName + " : No Sticky Faults");
            // stringLogEntry.append(cancoderName + " : No Sticky Faults");

        clearStickyFaults();
    }

    @Override
    public void readPeriodicInputs()
    {
        periodicData.relativePosition = cancoder.getPosition().getValueAsDouble();
        periodicData.absolutePosition = cancoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public void writePeriodicOutputs()
    {
        if(logPeriodicData)
            dblEntry.set(periodicData.absolutePosition);
            // doubleLogEntry.append(periodicData.absolutePosition);
    }

    @Override
    public void runPeriodicTask()
    {

    }

    @Override
    public String toString()
    {
        CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
        cancoder.getConfigurator().refresh(cancoderConfiguration);

        return cancoderConfiguration.toString();
    }
}
