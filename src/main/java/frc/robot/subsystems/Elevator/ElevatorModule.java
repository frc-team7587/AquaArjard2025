package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorModule {
     private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    private final RelativeEncoder leftElevatorMotorEncoder;
    private final RelativeEncoder rightElevatorMotorEncoder;
    public ElevatorModule(int leftElevatorMotorID, int rightElevatorMotorID) {
        leftElevatorMotor = new SparkMax(leftElevatorMotorID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(rightElevatorMotorID, MotorType.kBrushless);

        leftElevatorMotorEncoder = leftElevatorMotor.getEncoder();
        rightElevatorMotorEncoder = rightElevatorMotor.getEncoder();

        leftElevatorMotor.configure(Configs.ElevatorConfig.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(Configs.ElevatorConfig.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
}
