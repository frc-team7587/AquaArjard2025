package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Configs;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorModule implements ElevatorIO {
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    private final RelativeEncoder leftElevatorMotorEncoder;
    private final RelativeEncoder rightElevatorMotorEncoder;

    private final SparkClosedLoopController leftElevatorMotorController;
    private final SparkClosedLoopController rightElevatorMotorController;

    public ElevatorModule() {

        leftElevatorMotor = new SparkMax(ElevatorConstants.kElevatorLeftMotorID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.kElevatorRightMotorID, MotorType.kBrushless);

        leftElevatorMotorEncoder = leftElevatorMotor.getEncoder();
        rightElevatorMotorEncoder = rightElevatorMotor.getEncoder();

        leftElevatorMotorController = leftElevatorMotor.getClosedLoopController();
        rightElevatorMotorController = rightElevatorMotor.getClosedLoopController();        

        leftElevatorMotor.configure(Configs.ElevatorConfig.leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(Configs.ElevatorConfig.rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void elevatorUp(double speed) {
        leftElevatorMotor.set(speed);
    }

    @Override
    public void elevatorDown(double speed) {
        leftElevatorMotor.set(speed);
    }

    @Override
    public void setElevatorPosition(double position) {
        leftElevatorMotorController.setReference(
        position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ElevatorConstants.kFF);
    }

    @Override
    public void elevatorStop() {
        leftElevatorMotor.set(0);
    }

    @Override
    public double getElevatorPosition() {
        return (leftElevatorMotorEncoder.getPosition() + rightElevatorMotorEncoder.getPosition()) / 2;
    }

    @Override
    public void resetElevatorPosition() {
        leftElevatorMotorEncoder.setPosition(0);
    }
}
