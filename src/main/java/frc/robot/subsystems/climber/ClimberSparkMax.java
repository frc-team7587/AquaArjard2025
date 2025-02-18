package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs.climberConfig;

public class ClimberSparkMax implements ClimberIO {
    SparkMax climberMotor;

    public ClimberSparkMax() {
        climberMotor = new SparkMax(ClimberConstants.kClimberMotorLeaderID, MotorType.kBrushless);

        climberMotor.configure(climberConfig.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorSpeed(double speed) {
        climberMotor.set(speed);
    }

    @Override
    public void stop() {
        climberMotor.set(0);
    }
}
