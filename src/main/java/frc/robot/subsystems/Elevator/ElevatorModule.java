package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Configs;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorModule implements ElevatorIO {
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    private final RelativeEncoder leftElevatorMotorEncoder;
    private final RelativeEncoder rightElevatorMotorEncoder;

    private final SparkClosedLoopController leftElevatorMotorController;
    private final SparkClosedLoopController rightElevatorMotorController;

    

    private double setpoint =0;

    public ElevatorModule() {

        leftElevatorMotor = new SparkMax(ElevatorConstants.kElevatorLeftMotorID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.kElevatorRightMotorID, MotorType.kBrushless);

        leftElevatorMotorEncoder = leftElevatorMotor.getEncoder();
        rightElevatorMotorEncoder = rightElevatorMotor.getEncoder();

        leftElevatorMotorController = leftElevatorMotor.getClosedLoopController();
        rightElevatorMotorController = rightElevatorMotor.getClosedLoopController();        

        SparkMaxConfig Lconfig = new SparkMaxConfig();
        SparkMaxConfig Rconfig = new SparkMaxConfig();

        Lconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
            .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
        Rconfig
            .apply(Lconfig)
            .follow(10);

        leftElevatorMotor.configure(Lconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(Rconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


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
        position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kFF);
        //position, ControlType.kPosition);
    }
    @Override
    public void resetElevator(){
        leftElevatorMotorEncoder.setPosition(0);
    }

    @Override
    public void elevatorStop() {
        leftElevatorMotor.set(0);
    }

    @Override
    public double getElevatorPosition() {
        return (leftElevatorMotorEncoder.getPosition() + rightElevatorMotorEncoder.getPosition()) / 2;
    }
}


