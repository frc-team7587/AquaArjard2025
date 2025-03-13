package frc.robot.subsystems.Climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.ClimberConfig;

public class ClimberSparkMax implements ClimberIO{
    private final SparkMax climberMotor;

    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController climberController;

    public ClimberSparkMax() {
        climberMotor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);

        climberEncoder = climberMotor.getEncoder();
        climberController = climberMotor.getClosedLoopController();
        climberMotor.configure(ClimberConfig.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setClimberSpeed(double speed){
        climberMotor.set(speed);
    }
    @Override
    public void setClimberVoltage(double voltage){
        climberMotor.setVoltage(voltage);
    }
    @Override
    public void setClimberPosition(double position){
        climberController.setReference(position, ControlType.kPosition);
    }
    @Override
    public double getClimberPosition(){
        return climberEncoder.getPosition();
    }
    @Override
    public void reset(){
        climberEncoder.setPosition(0);
    }
}