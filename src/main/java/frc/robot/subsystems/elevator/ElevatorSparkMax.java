package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSparkMax implements ElevatorIO {
    SparkMax elevatorMotorLeader;
    SparkMax elevatorMotorFollower;
    SparkClosedLoopController closedLoopController;

    public ElevatorSparkMax() {
        elevatorMotorLeader = new SparkMax(ElevatorConstants.kElevatorMotorLeaderID, MotorType.kBrushless);
        elevatorMotorFollower = new SparkMax(ElevatorConstants.kElevatorMotorFollowerID, MotorType.kBrushless);
    
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
        leaderConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.2, 0.001, 0.05);

        followerConfig
            .apply(leaderConfig)
            .follow(elevatorMotorLeader);

        elevatorMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        closedLoopController = elevatorMotorLeader.getClosedLoopController();
    }

    @Override
    public void setMotorSpeed(double speed) {
        elevatorMotorLeader.set(speed);
    }

    @Override
    public void stop() {
        elevatorMotorLeader.set(0);
    }

    @Override
    public void setElevatorPosition(double position) {
        closedLoopController.setReference(position, ControlType.kPosition);
    }

    @Override
    public double getElevatorPosition() {
        return elevatorMotorLeader.getEncoder().getPosition();
    }
}
