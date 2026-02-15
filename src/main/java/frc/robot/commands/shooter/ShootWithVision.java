package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Continuously adjust shooter speed based on distance from vision.
 * 
 * USE CASE: Teleop shooting while robot moves
 * Automatically tracks target and adjusts RPM as distance changes.
 */
public class ShootWithVision extends Command {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier distanceSupplier;
    
    /**
     * @param shooter shooter subsystem
     * @param distanceSupplier function that returns current distance (from vision)
     */
    public ShootWithVision(ShooterSubsystem shooter, DoubleSupplier distanceSupplier) {
        this.shooter = shooter;
        this.distanceSupplier = distanceSupplier;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        System.out.println("[ShootWithVision] Starting - tracking target");
    }
    
    @Override
    public void execute() {
        // Get current distance from vision
        double distance = distanceSupplier.getAsDouble();
        
        // Update shooter velocity every loop (20ms)
        shooter.setVelocityForDistance(distance);
    }
    
    @Override
    public boolean isFinished() {
        // Never finishes on its own - runs until interrupted
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        System.out.println("[ShootWithVision] Stopped");
    }
}