package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberSparkMax implements ClimberIO {
    SparkMax climberMotorLeader;
    SparkMax climberMotorFollower;

    public ClimberSparkMax() {
        climberMotorLeader = new SparkMax(ClimberConstants.kClimberMotorLeaderID, MotorType.kBrushless);
        climberMotorFollower = new SparkMax(ClimberConstants.kClimberMotorFollowerID, MotorType.kBrushless);
    
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        followerConfig
            .apply(leaderConfig)
            .follow(climberMotorLeader)
            .inverted(true);

        climberMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climberMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorSpeed(double speed) {
        climberMotorLeader.set(speed);
    }

    @Override
    public void stop() {
        climberMotorLeader.set(0);
    }
}
