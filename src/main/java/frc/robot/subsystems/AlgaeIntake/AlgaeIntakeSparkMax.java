package frc.robot.subsystems.AlgaeIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs.AlgaeIntakeConfig;

public class AlgaeIntakeSparkMax implements AlgaeIntakeIO{
    private final SparkMax pivotMotor;
    private final SparkMax intakeMotor;

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    public AlgaeIntakeSparkMax(){
        pivotMotor = new SparkMax(AlgaeIntakeConstants.kPivotMotorID, MotorType.kBrushless);
        intakeMotor = new SparkMax(AlgaeIntakeConstants.kIntakeMotorID ,MotorType.kBrushless);

        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getClosedLoopController();
        pivotMotor.configure(AlgaeIntakeConfig.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(AlgaeIntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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