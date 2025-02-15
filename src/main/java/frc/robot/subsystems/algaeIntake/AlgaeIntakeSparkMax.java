package frc.robot.subsystems.algaeIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs.AlgaeIntakeConfigs;

public class AlgaeIntakeSparkMax implements AlgaeIntakeIO {
    SparkMax algaePivot;
    SparkMax algaeIntake;

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    public AlgaeIntakeSparkMax() {
        algaePivot = new SparkMax(AlgaeIntakeConstants.kAlgaeMotorLeaderID, MotorType.kBrushless);
        algaeIntake = new SparkMax(AlgaeIntakeConstants.kAlgaeMotorFollowerID, MotorType.kBrushless);
    
        pivotEncoder = algaePivot.getEncoder();
        pivotController = algaeIntake.getClosedLoopController();
        algaePivot.configure(AlgaeIntakeConfigs.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeIntake.configure(AlgaeIntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setIntakeSpeed(double speed){
        algaeIntake.set(speed);
    }
    @Override
    public void setPivotSpeed(double speed){
        algaePivot.set(speed);
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
