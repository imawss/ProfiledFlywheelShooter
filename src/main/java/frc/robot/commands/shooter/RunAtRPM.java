package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunAtRPM extends Command {
    private final ShooterSubsystem shooter;
    private final double targetRPM;

    public RunAtRPM(ShooterSubsystem shooter, double targetRPM) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.setVelocityRPM(targetRPM);
        System.out.printf("[RunAtRPM] Set to %.0f RPM%n", targetRPM);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}