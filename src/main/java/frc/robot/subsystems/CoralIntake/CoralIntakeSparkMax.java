package frc.robot.subsystems.CoralIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs.CoralIntakeConfig;

public class CoralIntakeSparkMax implements CoralIntakeIO{
    private final SparkMax pivotMotor;
    private final SparkMax intakeMotor;

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    public CoralIntakeSparkMax(){
        pivotMotor = new SparkMax(CoralIntakeConstants.kPivotMotorID, MotorType.kBrushless);
        intakeMotor = new SparkMax(CoralIntakeConstants.kIntakeMotorID,MotorType.kBrushless);

        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getClosedLoopController();
        pivotMotor.configure(CoralIntakeConfig.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(CoralIntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    @Override
    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }
    @Override
    public void setPivotSpeed(double speed){
        pivotMotor.set(speed);
    }
    @Override
    public void setPivotPosition(double position){
        pivotController.setReference(position, ControlType.kPosition);
    }
    @Override
    public double getPivotPosition(){
        return pivotEncoder.getPosition();
    }
    @Override
    public void reset(){
        pivotEncoder.setPosition(0);
    }
}