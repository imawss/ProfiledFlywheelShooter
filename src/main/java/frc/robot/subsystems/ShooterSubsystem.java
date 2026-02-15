package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterProfile;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motor;
    
    private final Map<String, ShooterProfile> availableProfiles;
    private final SendableChooser<String> profileChooser;
    private ShooterProfile activeProfile;
    private String lastSelectedProfileName = "";

    private double targetRPM = 0.0;
    private double commandedVoltage = 0.0;
    private boolean motorConfigured = false;
    
    private static final double CIM_FREE_SPEED_RPM = 5330.0;  
    private static final double MAX_VOLTAGE = 12.0;
    
    public ShooterSubsystem() {
        motor = new SparkMax(ShooterConstants.MOTOR_ID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(40);

        config.voltageCompensation(12.0);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        motorConfigured = true;
        
        DriverStation.reportWarning(
            "Shooter: Open-loop mode (NO ENCODER) - ID " + ShooterConstants.MOTOR_ID,
            false
        );

        availableProfiles = ShooterConstants.createAllProfiles();
        profileChooser = new SendableChooser<>();
        
        boolean defaultSet = false;
        for (Map.Entry<String, ShooterProfile> entry : availableProfiles.entrySet()) {
            String profileName = entry.getKey();
            ShooterProfile profile = entry.getValue();
            
            if (profileName.equals(ShooterConstants.DEFAULT_PROFILE_NAME) && !defaultSet) {
                profileChooser.setDefaultOption(profile.getDisplayName(), profileName);
                defaultSet = true;
            } else {
                profileChooser.addOption(profile.getDisplayName(), profileName);
            }
        }
        
        SmartDashboard.putData("Shooter/Profile Selector", profileChooser);
        setActiveProfile(ShooterConstants.DEFAULT_PROFILE_NAME);
        
        DriverStation.reportWarning(
            "⚠️ OPEN LOOP MODE - Accuracy limited without encoder",
            false
        );
    }
    
    @Override
    public void periodic() {

        String selectedProfileName = profileChooser.getSelected();
        if (selectedProfileName != null && !selectedProfileName.equals(lastSelectedProfileName)) {
            setActiveProfile(selectedProfileName);
        }

        if (!motorConfigured) {
            SmartDashboard.putBoolean("Shooter/Motor Configured", false);
        }
        
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Estimated RPM", getEstimatedRPM());
        SmartDashboard.putNumber("Shooter/Commanded Voltage", commandedVoltage);
        SmartDashboard.putNumber("Shooter/Motor Current (A)", motor.getOutputCurrent());
        SmartDashboard.putBoolean("Shooter/Open Loop Mode", true);
        
        if (activeProfile != null) {
            SmartDashboard.putString("Shooter/Active Profile", activeProfile.getName());
            SmartDashboard.putNumber("Shooter/Profile Angle (deg)", activeProfile.getAngleDegrees());
            SmartDashboard.putNumber("Shooter/Profile Min Dist (m)", activeProfile.getMinSafeDistance());
            SmartDashboard.putNumber("Shooter/Profile Max Dist (m)", activeProfile.getMaxSafeDistance());
        }
    }
    
    public void setVelocityForDistance(double distanceMeters) {
        if (activeProfile == null) {
            DriverStation.reportError("No active shooter profile", false);
            return;
        }
        
        double wheelRPM = getRPMForDistance(distanceMeters);
        setVelocityRPM(wheelRPM);
        
        SmartDashboard.putNumber("Shooter/Last Distance (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter/Last Commanded RPM", wheelRPM);
    }
    
    public void setVelocityRPM(double wheelRPM) {
        targetRPM = wheelRPM;

        double motorRPM = wheelRPM * ShooterConstants.GEAR_RATIO;

        commandedVoltage = (motorRPM / CIM_FREE_SPEED_RPM) * MAX_VOLTAGE;

        commandedVoltage = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, commandedVoltage));

        motor.setVoltage(commandedVoltage);
    }
    
    public void stop() {
        targetRPM = 0.0;
        commandedVoltage = 0.0;
        motor.setVoltage(0);
    }

    public double getWheelRPM() {
        return getEstimatedRPM();
    }
    
    public double getMotorRPM() {
        return getEstimatedRPM() * ShooterConstants.GEAR_RATIO;
    }
    
    private double getEstimatedRPM() {

        return (commandedVoltage / MAX_VOLTAGE) * CIM_FREE_SPEED_RPM / ShooterConstants.GEAR_RATIO;
    }
    
    public boolean atTargetVelocity() {
        if (targetRPM == 0.0) return false;

        return commandedVoltage != 0.0; 
    }
    
    public boolean isDistanceInRange(double distanceMeters) {
        if (activeProfile == null) return false;
        return activeProfile.isDistanceInRange(distanceMeters);
    }
    
    public String getActiveProfileName() {
        return activeProfile != null ? activeProfile.getName() : "NONE";
    }
    
    public double getActiveProfileAngle() {
        return activeProfile != null ? activeProfile.getAngleDegrees() : 0.0;
    }
    
    public void setActiveProfile(String profileName) {
        if (!availableProfiles.containsKey(profileName)) {
            DriverStation.reportError(
                "Profile '" + profileName + "' not found. Using default.",
                false
            );
            profileName = ShooterConstants.DEFAULT_PROFILE_NAME;
        }
        
        activeProfile = availableProfiles.get(profileName);
        lastSelectedProfileName = profileName;
        
        DriverStation.reportWarning(
            String.format("Shooter profile: %s (%.1f°, %.1f-%.1fm)",
                activeProfile.getName(),
                activeProfile.getAngleDegrees(),
                activeProfile.getMinSafeDistance(),
                activeProfile.getMaxSafeDistance()),
            false
        );
    }
    
    private double getRPMForDistance(double distance) {
        if (activeProfile == null) {
            DriverStation.reportError("No active profile - using default RPM", false);
            return 3500.0;
        }
        
        if (distance < activeProfile.getMinSafeDistance()) {
            DriverStation.reportWarning(
                String.format("Distance %.2fm below min %.2fm - using edge value",
                    distance, activeProfile.getMinSafeDistance()),
                false
            );
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return activeProfile.getRPMForDistance(activeProfile.getMinSafeDistance());
        }
        
        if (distance > activeProfile.getMaxSafeDistance()) {
            DriverStation.reportWarning(
                String.format("Distance %.2fm exceeds max %.2fm - using edge value",
                    distance, activeProfile.getMaxSafeDistance()),
                false
            );
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return activeProfile.getRPMForDistance(activeProfile.getMaxSafeDistance());
        }
        
        SmartDashboard.putBoolean("Shooter/Distance In Range", true);
        return activeProfile.getRPMForDistance(distance);
    }
}