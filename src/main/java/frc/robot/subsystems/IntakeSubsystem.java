package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motor;
    
    private static final int INTAKE_MOTOR_ID = 2;
    
    public IntakeSubsystem() {
        motor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushed);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);  
        config.smartCurrentLimit(30);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setSpeed(double speed) {
        motor.set(speed);
    }
    
    public void stop() {
        motor.set(0);
    }

    public void intake() {
        setSpeed(0.8);  
    }

    public void eject() {
        setSpeed(-0.5);  
    }
}