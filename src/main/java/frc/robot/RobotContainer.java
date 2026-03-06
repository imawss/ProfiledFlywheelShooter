package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.shooter.SpinUpForDistance;
import frc.robot.constants.ShooterConstants;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.ShootWithVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.Set;

public class RobotContainer {
    // Subsystems
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    
    // Controllers
    private final CommandXboxController operator = new CommandXboxController(0);
    
    // Vision (replace with real)
    private VisionSimulator vision = new VisionSimulator();
    
    public RobotContainer() {
        configureBindings();
        setupDashboard();
    }
    
    private void configureBindings() {

        operator.povUp().whileTrue(Commands.parallel(
            new ExtendIntake(intake),
            Commands.run(() -> shooter.setVelocityRPM(4000), shooter)
            ));
        operator.povDown().whileTrue(Commands.parallel(
            new RetractIntake(intake),
            Commands.run(() -> shooter.setVelocityRPM(0), shooter)
            ));


        // B = Shoot at 3.0m
        operator.b().onTrue(
            Commands.sequence(
                new SpinUpForDistance(shooter, intake, 3.0),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    //shooter.stop();
                    intake.stop();
                }, shooter, intake)
            )
        );

        /*
        double joyPos = operator.getRightY();
        if (Math.abs(joyPos) < 0.1) {
                joyPos = 0;
            }
        intake.setSpeed(joyPos);
        */

        
        // Y = Shoot at 4.0m
        operator.y().onTrue(
            Commands.sequence(
                new SpinUpForDistance(shooter, intake, 4.0),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    //shooter.stop();
                    intake.stop();
                }, shooter, intake)
            )
        );
        
        
        // Left Trigger
        operator.leftTrigger().onTrue(
            Commands.parallel(
            //Commands.run(() -> intake.setSpeed(5330), intake),
            Commands.run(() -> shooter.setVelocityRPM(4000), shooter)
            )
        );

        operator.rightTrigger().onTrue(
            Commands.run(() -> shooter.stop())
        );
        
        // Left Bumper = Eject
        operator.leftBumper().whileTrue(
            Commands.run(() -> intake.extend(), intake)
        );
        
        // Right Bumper = Manual shooter test (3000 RPM with intake)
        operator.rightBumper().whileTrue(
            Commands.parallel(
                //Commands.run(() -> shooter.setVelocityRPM(3000), shooter),
                Commands.run(() -> intake.retract(), intake)
            )
        );
                
        // D-Pad Up = Increase test distance
        operator.povUp().onTrue(
            Commands.runOnce(() -> {
                double current = SmartDashboard.getNumber("Test Distance (m)", 2.0);
                SmartDashboard.putNumber("Test Distance (m)", current + 0.5);
                System.out.printf("Test distance: %.1fm%n", current + 0.5);
            })
        );
        
        // D-Pad Down = Decrease test distance
        operator.povDown().onTrue(
            Commands.runOnce(() -> {
                double current = SmartDashboard.getNumber("Test Distance (m)", 2.0);
                SmartDashboard.putNumber("Test Distance (m)", Math.max(1.0, current - 0.5));
                System.out.printf("Test distance: %.1fm%n", Math.max(1.0, current - 0.5));
            })
        );
        
        // Start = Shoot at test distance
        operator.start().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> {
                    double dist = SmartDashboard.getNumber("Test Distance (m)", 2.0);
                    System.out.printf("=== SHOOTING AT %.1fm ===%n", dist);
                }),
                Commands.defer(() -> {
                    double dist = SmartDashboard.getNumber("Test Distance (m)", 2.0);
                    return new SpinUpForDistance(shooter, intake, dist);
                }, Set.of(shooter, intake)),  // ← Both subsystems
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    shooter.stop();
                    intake.stop();
                }, shooter, intake)
            )
        );
        
        // X = STOP EVERYTHING
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
            "A/B/Y=Shoot | RT=Vision | LT=Intake | LB=Eject | X=STOP");
    }
    
    public Command getAutonomousCommand() {
        return Commands.sequence(
            Commands.print("[Auto] Starting shot"),
            new SpinUpForDistance(shooter, intake, 2.5),  // Already runs both
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> {
                shooter.stop();
                intake.stop();
            }, shooter, intake),
            Commands.print("[Auto] Complete")
        );
    }

    private static class VisionSimulator {
        public double getDistance() {
            return 3.0;
        }
    }
}