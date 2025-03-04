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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorModule implements ElevatorIO {
    private final SparkMax leftElevatorMotor;
    private final SparkMax rightElevatorMotor;

    private final RelativeEncoder leftElevatorMotorEncoder;

    private final SparkClosedLoopController leftElevatorMotorController;
    

  // PID controller uses motion profiling to smoothly move setpoint from current position to goal position
  // Feed forward tracks the setpoint's velocity/acceleration to move motors without the need for
  // much PID
  // PID is only used for small corrections
  private ProfiledPIDController pidController =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));

  private ElevatorFeedforward feedforward = ElevatorConstants.kFF;

    public ElevatorModule() {

        leftElevatorMotor = new SparkMax(ElevatorConstants.kElevatorLeftMotorID, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(ElevatorConstants.kElevatorRightMotorID, MotorType.kBrushless);

        leftElevatorMotorEncoder = leftElevatorMotor.getEncoder();
        leftElevatorMotorEncoder.setPosition(0);

        leftElevatorMotorController = leftElevatorMotor.getClosedLoopController();

        SparkMaxConfig Lconfig = new SparkMaxConfig();
        SparkMaxConfig Rconfig = new SparkMaxConfig();
        Lconfig
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .inverted(false);
        Lconfig
            .encoder
            .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.kPositionConversionFactor / 60.0);
        Rconfig
            .apply(Lconfig)
            .follow(leftElevatorMotor, false);

        leftElevatorMotor.configure(Lconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightElevatorMotor.configure(Rconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        pidController.setTolerance(ElevatorConstants.kSetpointToleranceMeters);


    }

    @Override
    public void elevatorUp(double speed) {
        leftElevatorMotor.set(speed);
        
    }

    @Override
    public void elevatorDown(double speed) {
        leftElevatorMotor.set(speed);
    }

    // @Override
    // public void setElevatorPosition(double position) {
    //     leftElevatorMotorController.setReference(
    //     position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kFF);
    //     //position, ControlType.kPosition);
    // }

    @Override
    public void setSetpoint(double setpoint) {
      pidController.setGoal(setpoint);
      pidController.reset(getPosition(), getVelocity());
    }

    @Override
    public double getVelocity() {
      return leftElevatorMotorEncoder.getVelocity();
    }

    @Override
    public double getPosition() {
      // get the absolute position in radians, then convert to meters
      return leftElevatorMotorEncoder.getPosition();
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
        return (leftElevatorMotorEncoder.getPosition());
    }
}


