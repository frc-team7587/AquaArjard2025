package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Configs;
import frc.robot.Configs.ElevatorConfig;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorModule implements ElevatorIO {
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    private final RelativeEncoder leftElevatorMotorEncoder;
    private final RelativeEncoder rightElevatorMotorEncoder;

    private final SparkClosedLoopController leftElevatorMotorController;
    private final SparkClosedLoopController rightElevatorMotorController;

    //private final double countsPerInch = 42.0;
    private final double gravityCompensation = 0.1; // Tune this value - usually between 0.05-0.2


    public ElevatorModule() {

        leftElevatorMotor = new SparkMax(ElevatorConstants.kElevatorLeftMotorID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.kElevatorRightMotorID, MotorType.kBrushless);

        leftElevatorMotorEncoder = leftElevatorMotor.getEncoder();
        rightElevatorMotorEncoder = rightElevatorMotor.getEncoder();

        leftElevatorMotorController = leftElevatorMotor.getClosedLoopController();
        rightElevatorMotorController = rightElevatorMotor.getClosedLoopController();        

        SparkMaxConfig Lconfig = new SparkMaxConfig();
        SparkMaxConfig Rconfig = new SparkMaxConfig();

        Lconfig
            .idleMode(IdleMode.kBrake);
        Lconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 0)
            .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);

        Rconfig
            .idleMode(IdleMode.kBrake)
            .follow(ElevatorConstants.kElevatorLeftMotorID);
        Rconfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 0)
            .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);

        leftElevatorMotor.configure(Lconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(Rconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

       

       
    }

    @Override
    public void elevatorUp(double speed) {
        // leftElevatorMotor.setVoltage(ElevatorConstants.kElevatorVoltage);
        // rightElevatorMotor.setVoltage(ElevatorConstants.kElevatorVoltage);

        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
    }

    @Override
    public void elevatorDown(double speed) {
        leftElevatorMotor.set(speed);
    }

    @Override
    public void setElevatorPosition(double position) {
        
        // double pidOutput = pid.calculate(getElevatorPosition(), position);
        // // Add gravity compensation
        // // The sign is positive because we need to work against gravity
        // // You might need to flip the sign depending on your motor polarity
        // double motorOutput = pidOutput + gravityCompensation;

        // // Clamp the output to valid range
        // motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);
    
        // leftElevatorMotor.set(motorOutput); 
        

        leftElevatorMotorController.setReference(
            position, ControlType.kPosition, ClosedLoopSlot.kSlot0, 
            new ElevatorFeedforward(0.1,1.3,1.5,0.05).calculate(0));





        //new ElevatorFeedforward(1.175,1.625,4.6,0.15).calculate(0));
        

        


       
    }
    @Override
    public void resetElevator(){
        leftElevatorMotorEncoder.setPosition(0);
    }

    @Override
    public void elevatorStop() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);

    }

    @Override
    public double getElevatorPosition() {
        //return leftElevatorMotorEncoder.getPosition() / countsPerInch;
        return leftElevatorMotorEncoder.getPosition();
    }
    @Override
    public double getLvoltage() {
        return leftElevatorMotor.getOutputCurrent();
    }
    @Override
    public double getRvoltage() {
        return rightElevatorMotor.getOutputCurrent();
    }
}


