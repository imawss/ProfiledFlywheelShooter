package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShootWithVision extends Command {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier distanceSupplier;

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
        double distance = distanceSupplier.getAsDouble();
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