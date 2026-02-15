package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple intake subsystem - feeds game pieces into shooter.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motor;
    
    // CHANGE THIS TO YOUR INTAKE MOTOR CAN ID
    private static final int INTAKE_MOTOR_ID = 2;
    
    public IntakeSubsystem() {
        motor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushed);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);  // Brake mode for intake
        config.smartCurrentLimit(30);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Set intake speed.
     * @param speed -1.0 to 1.0 (positive = intake, negative = eject)
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }
    
    /**
     * Stop intake.
     */
    public void stop() {
        motor.set(0);
    }
    
    /**
     * Run intake forward (pick up game piece).
     */
    public void intake() {
        setSpeed(0.8);  // 80% speed
    }
    
    /**
     * Eject game piece.
     */
    public void eject() {
        setSpeed(-0.5);  // 50% reverse
    }
}