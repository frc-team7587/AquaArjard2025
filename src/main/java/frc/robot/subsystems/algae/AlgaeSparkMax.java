package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeSparkMax implements AlgaeIO {
    SparkMax algaeMotorLeader;
    SparkMax algaeMotorFollower;

    public AlgaeSparkMax() {
        algaeMotorLeader = new SparkMax(AlgaeConstants.kAlgaeMotorLeaderID, MotorType.kBrushless);
        algaeMotorFollower = new SparkMax(AlgaeConstants.kAlgaeMotorFollowerID, MotorType.kBrushless);
    
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        followerConfig
            .apply(leaderConfig)
            .follow(algaeMotorLeader)
            .inverted(true);

        algaeMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorSpeed(double speed) {
        algaeMotorLeader.set(speed);
    }

    @Override
    public void stop() {
        algaeMotorLeader.set(0);
    }
}
