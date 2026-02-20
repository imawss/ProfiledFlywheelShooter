package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUpForDistance extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final double distanceMeters;

    public SpinUpForDistance(
        ShooterSubsystem shooter,
        IntakeSubsystem intake,
        double distanceMeters
    ) {
        this.shooter = shooter;
        this.intake = intake;
        this.distanceMeters = distanceMeters;
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        if (!shooter.isDistanceInRange(distanceMeters)) {
            System.err.printf("[SpinUp] WARNING: %.2fm is outside safe range!%n", distanceMeters);
        }

        shooter.setVelocityForDistance(distanceMeters);

        intake.stop();

        System.out.printf("[SpinUp] Spinning up for %.2fm — intake BEKLIYOR...%n", distanceMeters);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return shooter.atTargetVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("[SpinUp] Interrupted — stopping all");
            shooter.stop();
            intake.stop();
        } else {
            System.out.printf("[SpinUp] Ready! Shooter at target for %.2fm%n", distanceMeters);
        }
    }
}