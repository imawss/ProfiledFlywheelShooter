package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterSubsystem;

public class HardwareTest extends Command {
    private final ShooterSubsystem shooter;
    
    private double[] testRPMs = {1000, 2000, 3000, 4000};
    private int currentTestIndex = 0;
    private double testStartTime = 0.0;
    
    private static final double TEST_DURATION = 3.0; 
    
    public HardwareTest(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        currentTestIndex = 0;
        testStartTime = Timer.getFPGATimestamp();
        
        System.out.println("========================================");
        System.out.println("SHOOTER HARDWARE TEST");
        System.out.println("Testing RPM: 1000, 2000, 3000, 4000");
        System.out.println("Watch dashboard for actual values");
        System.out.println("========================================");
        
        startTest(0);
    }
    
    @Override
    public void execute() {
        double elapsed = Timer.getFPGATimestamp() - testStartTime;
        
        if (elapsed > TEST_DURATION) {
            currentTestIndex++;
            
            if (currentTestIndex < testRPMs.length) {
                startTest(currentTestIndex);
                testStartTime = Timer.getFPGATimestamp();
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        return currentTestIndex >= testRPMs.length;
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        
        if (interrupted) {
            System.out.println("[HardwareTest] Interrupted");
        } else {
            System.out.println("========================================");
            System.out.println("HARDWARE TEST COMPLETE");
            System.out.println("Check:");
            System.out.println("1. Did motor spin?");
            System.out.println("2. Did 'Actual RPM' change on dashboard?");
            System.out.println("3. Was 'Error RPM' < 100?");
            System.out.println("4. Did motor sound smooth?");
            System.out.println("========================================");
        }
    }
    
    private void startTest(int index) {
        double rpm = testRPMs[index];
        shooter.setVelocityRPM(rpm);
        
        System.out.printf("Test %d/%d: Target = %.0f RPM%n",
            index + 1, testRPMs.length, rpm);
        System.out.println("  Watch dashboard for actual RPM...");
    }
}