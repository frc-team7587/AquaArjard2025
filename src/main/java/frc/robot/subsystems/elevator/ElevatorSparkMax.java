package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSparkMax implements ElevatorIO {
    SparkMax elevatorMotorLeader;
    SparkMax elevatorMotorFollower;

    public ElevatorSparkMax() {
        elevatorMotorLeader = new SparkMax(ElevatorConstants.kElevatorMotorLeaderID, MotorType.kBrushless);
        elevatorMotorFollower = new SparkMax(ElevatorConstants.kElevatorMotorFollowerID, MotorType.kBrushless);
    
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        followerConfig
            .apply(leaderConfig)
            .follow(elevatorMotorLeader)
            .inverted(true);

        elevatorMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorSpeed(double speed) {
        elevatorMotorLeader.set(speed);
    }

    @Override
    public void stop() {
        elevatorMotorLeader.set(0);
    }
}
