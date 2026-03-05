package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.shooter.SpinUpForDistance;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.Set;

public class RobotContainer {

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem  intake  = new IntakeSubsystem();
    private final CommandXboxController operator = new CommandXboxController(0);
    private final VisionSimulator vision = new VisionSimulator();

    public RobotContainer() {
        configureBindings();
        setupDashboard();
    }

    private void configureBindings() {

        operator.povUp().whileTrue(
            Commands.parallel(
                new ExtendIntake(intake),
                Commands.run(() -> shooter.setVelocityRPM(4000), shooter)
            )
        );

        operator.povDown().whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> shooter.stop(), shooter),
                new RetractIntake(intake)
            )
        );

        operator.b().onTrue(
            Commands.sequence(
                new SpinUpForDistance(shooter, intake, 3.0),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> { shooter.stop(); intake.stop(); }, shooter, intake)
            )
        );

        operator.y().onTrue(
            Commands.sequence(
                new SpinUpForDistance(shooter, intake, 4.0),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> { shooter.stop(); intake.stop(); }, shooter, intake)
            )
        );

        operator.leftTrigger().whileTrue(
            Commands.run(() -> shooter.setVelocityRPM(6000), shooter)
        );

        operator.rightTrigger().onTrue(
            Commands.runOnce(() -> shooter.stop(), shooter)
        );

        operator.leftBumper().whileTrue(
            Commands.run(() -> intake.extend(), intake)
        );

        operator.rightBumper().whileTrue(
            Commands.run(() -> intake.retract(), intake)
        );

        operator.back().onTrue(
            Commands.runOnce(() -> {
                double current = SmartDashboard.getNumber("Test Distance (m)", 2.0);
                double next = Math.max(1.0, current - 0.5);
                SmartDashboard.putNumber("Test Distance (m)", next);
                System.out.printf("Test distance: %.1fm%n", next);
            })
        );

        operator.start().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> {
                    double dist = SmartDashboard.getNumber("Test Distance (m)", 2.0);
                    System.out.printf("=== SHOOTING AT %.1fm ===%n", dist);
                }),
                Commands.defer(() -> {
                    double dist = SmartDashboard.getNumber("Test Distance (m)", 2.0);
                    return new SpinUpForDistance(shooter, intake, dist);
                }, Set.of(shooter, intake)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> { shooter.stop(); intake.stop(); }, shooter, intake)
            )
        );

        operator.x().onTrue(
            Commands.runOnce(() -> {
                shooter.stop();
                intake.stop();
                System.out.println("!!! EMERGENCY STOP !!!");
            }, shooter, intake)
        );
    }

    private void setupDashboard() {
        SmartDashboard.putNumber("Test Distance (m)", 2.5);
        SmartDashboard.putString("Shooter/Controls",
            "B/Y=Shoot | LT(hold)=Spin | RT=Stop | LB=Extend | RB=Retract | Back=DistDown | Start=TestShot | X=ESTOP");
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
            Commands.print("[Auto] Starting shot"),
            new SpinUpForDistance(shooter, intake, 2.5),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> { shooter.stop(); intake.stop(); }, shooter, intake),
            Commands.print("[Auto] Complete")
        );
    }

    private static class VisionSimulator {
        public double getDistance() { return 3.0; }
    }
}