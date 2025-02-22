package frc.robot.subsystems.algaeOuttake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs.AlgaeOuttakeConfig;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;

public class AlgaeOuttakeSparkMax implements AlgaeOuttakeIO {
    SparkMax algaePivot;
    SparkMax algaeOuttake;

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    public AlgaeOuttakeSparkMax() {
        algaePivot = new SparkMax(AlgaeIntakeConstants.kAlgaeMotorLeaderID, MotorType.kBrushless);
        algaeOuttake = new SparkMax(AlgaeIntakeConstants.kAlgaeMotorFollowerID, MotorType.kBrushless);
    
        pivotEncoder = algaePivot.getEncoder();
        pivotController = algaeOuttake.getClosedLoopController();
        algaePivot.configure(AlgaeOuttakeConfig.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeOuttake.configure(AlgaeOuttakeConfig.indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setOuttakeSpeed(double speed){
        algaeOuttake.set(speed);
    }
    @Override
    public void setPivotSpeed(double speed){
        algaePivot.set(speed);
    }
    @Override
    public void setPivotPosition(double position){
        pivotController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, AlgaeOuttakeConstants.kFeedForward);
    }
    @Override
    public double getPivotPosition(){
        return pivotEncoder.getPosition();
    }
    @Override
    public void reset(){
        pivotEncoder.setPosition(0);
    }
    @Override
    public void stop() {
        algaeOuttake.set(0);
    }
}
