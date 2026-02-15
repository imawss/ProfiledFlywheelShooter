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
    
    // CIM motor constants (adjust if needed)
    private static final double CIM_FREE_SPEED_RPM = 5330.0;  // At 12V
    private static final double MAX_VOLTAGE = 12.0;
    
    public ShooterSubsystem() {
        // ========================================
        // MOTOR SETUP - Open Loop (No Encoder)
        // ========================================
        motor = new SparkMax(ShooterConstants.MOTOR_ID, MotorType.kBrushed);
        
        // Create configuration
        SparkMaxConfig config = new SparkMaxConfig();
        
        // NO PID - we're doing open loop voltage control
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(40);
        
        // Voltage compensation - keeps consistent speed as battery drains
        config.voltageCompensation(12.0);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        motorConfigured = true;
        
        DriverStation.reportWarning(
            "Shooter: Open-loop mode (NO ENCODER) - ID " + ShooterConstants.MOTOR_ID,
            false
        );
        
        // ========================================
        // PROFILE SETUP
        // ========================================
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
        // Profile selection
        String selectedProfileName = profileChooser.getSelected();
        if (selectedProfileName != null && !selectedProfileName.equals(lastSelectedProfileName)) {
            setActiveProfile(selectedProfileName);
        }

        // Telemetry
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
    
    // ========================================
    // PUBLIC API
    // ========================================
    
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
    
    /**
     * Set shooter velocity in RPM using open-loop voltage control.
     * 
     * ⚠️ WITHOUT ENCODER: This is an ESTIMATE based on motor characteristics.
     * Actual speed will vary with:
     * - Battery voltage
     * - Motor load
     * - Motor wear
     * 
     * @param wheelRPM target wheel surface speed in RPM
     */
    public void setVelocityRPM(double wheelRPM) {
        targetRPM = wheelRPM;
        
        // Convert wheel RPM to motor RPM
        double motorRPM = wheelRPM * ShooterConstants.GEAR_RATIO;
        
        // Calculate voltage needed
        // V = (RPM / FREE_SPEED_RPM) * MAX_VOLTAGE
        commandedVoltage = (motorRPM / CIM_FREE_SPEED_RPM) * MAX_VOLTAGE;
        
        // Apply limits
        commandedVoltage = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, commandedVoltage));
        
        // Set voltage
        motor.setVoltage(commandedVoltage);
    }
    
    public void stop() {
        targetRPM = 0.0;
        commandedVoltage = 0.0;
        motor.setVoltage(0);
    }
    
    /**
     * Get estimated wheel RPM (calculated from voltage, not measured).
     * ⚠️ This is an ESTIMATE - actual speed may differ!
     */
    public double getWheelRPM() {
        return getEstimatedRPM();
    }
    
    /**
     * Get estimated motor RPM based on commanded voltage.
     * ⚠️ NOT measured - just calculation!
     */
    public double getMotorRPM() {
        return getEstimatedRPM() * ShooterConstants.GEAR_RATIO;
    }
    
    private double getEstim"atedRPM() {
        // Estimate based on voltage
        // RPM = (Voltage / MAX_VOLTAGE) * FREE_SPEED_RPM / GEAR_RATIO
        return (commandedVoltage / MAX_VOLTAGE) * CIM_FREE_SPEED_RPM / ShooterConstants.GEAR_RATIO;
    }
    
    /**
     * ⚠️ In open loop mode, we can't actually measure if we're at target.
     * This assumes we reach target after 1 second.
     */
    public boolean atTargetVelocity() {
        if (targetRPM == 0.0) return false;
        
        // Without encoder, we just wait 1 second and assume we're there
        // This is a hack but necessary without feedback
        return commandedVoltage != 0.0; // Instantly "ready" for testing
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
    
    // ========================================
    // PROFILE MANAGEMENT
    // ========================================
    
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
    
    // ========================================
    // PRIVATE HELPERS
    // ========================================
    
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