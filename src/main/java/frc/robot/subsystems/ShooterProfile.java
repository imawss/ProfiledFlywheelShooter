package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterProfile {
    private final String name;
    private final String description;
    
    private final double angleDegrees;
    private final double launchHeightMeters;
    private final double targetHeightMeters;

    private final InterpolatingDoubleTreeMap distanceToRPM;
    
    private final double minSafeDistance;
    private final double maxSafeDistance;
    private final double defaultRPM;

    public ShooterProfile(
        String name,
        String description,
        double angleDegrees,
        double launchHeightMeters,
        double targetHeightMeters,
        InterpolatingDoubleTreeMap distanceToRPM,
        double minSafeDistance,
        double maxSafeDistance,
        double defaultRPM
    ) {
        this.name = name;
        this.description = description;
        this.angleDegrees = angleDegrees;
        this.launchHeightMeters = launchHeightMeters;
        this.targetHeightMeters = targetHeightMeters;
        this.distanceToRPM = distanceToRPM;
        this.minSafeDistance = minSafeDistance;
        this.maxSafeDistance = maxSafeDistance;
        this.defaultRPM = defaultRPM;
    }
    
    public String getName() {
        return name;
    }
    
    public String getDescription() {
        return description;
    }
    
    public double getAngleDegrees() {
        return angleDegrees;
    }
    
    public double getLaunchHeightMeters() {
        return launchHeightMeters;
    }
    
    public double getTargetHeightMeters() {
        return targetHeightMeters;
    }
    
    public double getMinSafeDistance() {
        return minSafeDistance;
    }
    
    public double getMaxSafeDistance() {
        return maxSafeDistance;
    }
    
    public double getDefaultRPM() {
        return defaultRPM;
    }
    

    public double getRPMForDistance(double distanceMeters) {
        return distanceToRPM.get(distanceMeters);
    }

    public boolean isDistanceInRange(double distanceMeters) {
        return distanceMeters >= minSafeDistance && distanceMeters <= maxSafeDistance;
    }
    
    public String getDisplayName() {
        return name + " - " + description;
    }
    
    @Override
    public String toString() {
        return String.format("ShooterProfile[%s, %.1f°, %.1f-%.1fm]",
            name, angleDegrees, minSafeDistance, maxSafeDistance);
    }
}