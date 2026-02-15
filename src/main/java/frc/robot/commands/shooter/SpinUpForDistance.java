package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Spin up shooter AND intake for a specific distance.
 * Intake feeds game piece into shooter.
 */
public class SpinUpForDistance extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;  // ← Added
    private final double distanceMeters;
    
    public SpinUpForDistance(
        ShooterSubsystem shooter, 
        IntakeSubsystem intake,    // ← Added
        double distanceMeters
    ) {
        this.shooter = shooter;
        this.intake = intake;      // ← Added
        this.distanceMeters = distanceMeters;
        addRequirements(shooter, intake);  // ← Both subsystems
    }
    
    @Override
    public void initialize() {
        if (!shooter.isDistanceInRange(distanceMeters)) {
            System.err.printf("[SpinUp] WARNING: %.2fm is outside safe range!%n", distanceMeters);
        }
        
        // Start shooter
        shooter.setVelocityForDistance(distanceMeters);
        
        // Start intake to feed
        intake.setSpeed(0.5);  // ← Run intake at 50% (adjust as needed)
        
        System.out.printf("[SpinUp] Spinning for %.2fm (with intake)%n", distanceMeters);
    }
    
    @Override
    public boolean isFinished() {
        return shooter.atTargetVelocity();
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("[SpinUp] Interrupted");
            shooter.stop();
            intake.stop();  // ← Stop intake too
        } else {
            System.out.println("[SpinUp] Ready to shoot!");
            // Keep both running - they'll stop after shot completes
        }
    }
}