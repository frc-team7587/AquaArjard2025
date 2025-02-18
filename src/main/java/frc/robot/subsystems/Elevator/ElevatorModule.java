package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorModule implements ElevatorIO {
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    private final RelativeEncoder leftElevatorMotorEncoder;
    private final RelativeEncoder rightElevatorMotorEncoder;

    public ElevatorModule() {

        leftElevatorMotor = new SparkMax(ElevatorConstants.kElevatorLeftMotorID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.kElevatorRightMotorID, MotorType.kBrushless);

        leftElevatorMotorEncoder = leftElevatorMotor.getEncoder();
        rightElevatorMotorEncoder = rightElevatorMotor.getEncoder();

        leftElevatorMotor.configure(Configs.ElevatorConfig.elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(Configs.ElevatorConfig.elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void elevatorUp(double speed) {
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
    }

    @Override
    public void elevatorDown(double speed) {
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
    }

    @Override
    public void setElevatorPosition(double position) {
        leftElevatorMotorEncoder.setPosition(position);
        rightElevatorMotorEncoder.setPosition(position);
    }

    @Override
    public void elevatorStop() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
    }

    @Override
    public double getElevatorPosition() {
        return (leftElevatorMotorEncoder.getPosition() + rightElevatorMotorEncoder.getPosition()) / 2;
    }

    @Override
    public void resetElevatorPosition() {
        leftElevatorMotorEncoder.setPosition(0);
        rightElevatorMotorEncoder.setPosition(0);
    }
}
