package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

public class ElevatorSparkMax implements ElevatorIO {
    SparkMax leftElevatorMotor;
    SparkMax rightElevatorMotor;

    private final RelativeEncoder leftElevatorMotorEncoder;
    private final RelativeEncoder rightElevatorMotorEncoder;

    public ElevatorSparkMax() {
        leftElevatorMotor = new SparkMax(ElevatorConstants.kElevatorMotorLeaderID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.kElevatorMotorFollowerID, MotorType.kBrushless);

        leftElevatorMotorEncoder = leftElevatorMotor.getEncoder();
        rightElevatorMotorEncoder = rightElevatorMotor.getEncoder();
    
        leftElevatorMotor.configure(Configs.ElevatorConfig.elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(Configs.ElevatorConfig.elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorSpeed(double speed) {
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
    }

    @Override
    public void stop() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
    }

    @Override
    public void setElevatorPosition(double position) {
        leftElevatorMotorEncoder.setPosition(position);
        rightElevatorMotorEncoder.setPosition(position);
    }

    @Override
    public double getElevatorPosition() {
        return (leftElevatorMotor.getEncoder().getPosition() + rightElevatorMotor.getEncoder().getPosition()) / 2;
    }

    @Override
    public double getElevatorVelocity() {
        return (leftElevatorMotor.getEncoder().getVelocity() + rightElevatorMotor.getEncoder().getVelocity()) / 2;
    }

    @Override
    public void resetElevatorPosition() {
        leftElevatorMotorEncoder.setPosition(0);
        rightElevatorMotorEncoder.setPosition(0);
    }
}
