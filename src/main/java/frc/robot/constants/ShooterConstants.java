package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ShooterProfile;

import java.util.HashMap;
import java.util.Map;

public final class ShooterConstants {
    private ShooterConstants() {}
    
    // ========================================
    // HARDWARE - Single CIM with Spark MAX
    // ========================================
    public static final int MOTOR_ID = 1;  // YOUR CAN ID
    
    // Mechanism properties
    public static final double GEAR_RATIO = 1.5;  // YOUR ACTUAL GEAR RATIO
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

    // ========================================
    // CONTROL - CIM tuning
    // ========================================
    public static final double kP = 0.0001;   // Start low, tune on hardware
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0002; // Feedforward for velocity
    
    public static final double VELOCITY_TOLERANCE_RPM = 100.0;

    // Safety limit (start at 50%, increase as needed)
    public static final double MAX_OUTPUT = 0.8;
    
    // ========================================
    // PHYSICAL MEASUREMENTS
    // ========================================
    public static final double LAUNCH_HEIGHT_METERS = 0.58;
    public static final double TARGET_HEIGHT_METERS = 2.05;

    public static final String DEFAULT_PROFILE_NAME = "BALANCED";
    
    // ========================================
    // PROFILES
    // ========================================
    
    public static Map<String, ShooterProfile> createAllProfiles() {
        Map<String, ShooterProfile> profiles = new HashMap<>();
        
        profiles.put("BALANCED", createBalancedProfile());
        profiles.put("STEEP_CLOSE", createSteepCloseProfile());
        profiles.put("FLAT_LONG", createFlatLongProfile());
        profiles.put("EXPERIMENTAL", createExperimentalProfile());
        
        return profiles;
    }
    
    private static ShooterProfile createBalancedProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        map.put(1.5, 2600.0);
        map.put(2.0, 3000.0);
        map.put(2.5, 3450.0);
        map.put(3.0, 3950.0);
        map.put(3.5, 4500.0);
        map.put(4.0, 5100.0);
        map.put(4.5, 5750.0);
        map.put(5.0, 6450.0);
        
        return new ShooterProfile(
            "BALANCED",
            "45° All-Purpose (1.5-5.0m)",
            45.0,
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            map,
            1.5,
            5.0,
            3800.0
        );
    }

    private static ShooterProfile createSteepCloseProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        map.put(1.0, 2200.0);
        map.put(1.5, 2500.0);
        map.put(2.0, 2900.0);
        map.put(2.5, 3400.0);
        map.put(3.0, 4000.0);
        map.put(3.5, 4700.0);
        
        return new ShooterProfile(
            "STEEP_CLOSE",
            "60° Over Defense (1.0-3.5m)",
            60.0,
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            map,
            1.0,
            3.5,
            3000.0
        );
    }
    
    private static ShooterProfile createFlatLongProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        map.put(2.5, 3800.0);
        map.put(3.0, 4200.0);
        map.put(3.5, 4650.0);
        map.put(4.0, 5150.0);
        map.put(4.5, 5700.0);
        map.put(5.0, 6300.0);
        map.put(5.5, 6950.0);
        map.put(6.0, 7650.0);
        
        return new ShooterProfile(
            "FLAT_LONG",
            "35° Long Range (2.5-6.0m)",
            35.0,
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            map,
            2.5,
            6.0,
            4500.0
        );
    }
    
    private static ShooterProfile createExperimentalProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        map.put(1.5, 2700.0);
        map.put(2.0, 3100.0);
        map.put(2.5, 3550.0);
        map.put(3.0, 4050.0);
        map.put(3.5, 4600.0);
        map.put(4.0, 5200.0);
        map.put(4.5, 5850.0);
        
        return new ShooterProfile(
            "EXPERIMENTAL",
            "Test Config (47° - USE CAUTION)",
            47.0,
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            map,
            1.5,
            4.5,
            3900.0
        );
    }
}