// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

import java.lang.reflect.Type;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import org.littletonrobotics.junction.Logger;

import frc.robot.Configs;
import frc.robot.subsystems.swerve.SwerveDriveIO.SwerveDriveIOInputs;

public class SwerveModule implements SwerveDriveIO {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());



  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), SparkBase.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  @Override
  public double getDrivingPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDrivingPosition'");
  }

  @Override
  public double getTurningPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTurningPosition'");
  }

  @Override
  public double getDrivingVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDrivingVelocity'");
  }

  @Override
  public double getTurningVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTurningVelocity'");
  }

  @Override
  public void resetEncoder() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetEncoder'");
  }

  @Override
  public void updateInputs(SwerveDriveIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void setDriveVoltage(double volts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveVoltage'");
  }

  @Override
  public void setTurnVoltage(double volts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTurnVoltage'");
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveVelocity'");
  }

  @Override
  public void setTurnPosition(double angle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTurnPosition'");
  }

  @Override
  public void setDrivePIDFF(double p, double i, double d, double ff) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDrivePIDFF'");
  }

  @Override
  public void setTurnPIDFF(double p, double i, double d, double ff) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTurnPIDFF'");
  }

  @Override
  public double getTurnPositionError(double angle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTurnPositionError'");
  }

@Override
  public void setDriveBrakeMode(boolean enable) {
    Configs.MAXSwerveModule.drivingConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    m_drivingSpark.configure(
      Configs.MAXSwerveModule.drivingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    Configs.MAXSwerveModule.turningConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    m_turningSpark.configure(
      Configs.MAXSwerveModule.turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

public Rotation2d getAngle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAngle'");
}

}