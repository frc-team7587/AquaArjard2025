// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import static frc.robot.Constants.ModuleConstants.kDrivingD;
import static frc.robot.Constants.ModuleConstants.kDrivingFF;
import static frc.robot.Constants.ModuleConstants.kDrivingI;
import static frc.robot.Constants.ModuleConstants.kDrivingP;
import static frc.robot.Constants.ModuleConstants.kTurningD;
import static frc.robot.Constants.ModuleConstants.kTurningFF;
import static frc.robot.Constants.ModuleConstants.kTurningI;
import static frc.robot.Constants.ModuleConstants.kTurningP;
import static frc.robot.Constants.ModuleConstants.kWheelDiameterMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveIO.SwerveDriveIOInputs;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger.*;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public class Module {
  private static final double WHEEL_RADIUS = kWheelDiameterMeters / 2;
  public static final double ODOMETRY_FREQUENCY = 250.0;

  private final SwerveDriveIO io;
  private final SwerveDriveIOInputs inputs = new SwerveDriveIOInputs();
  private final int index;

  // private final SimpleMotorFeedforward driveFeedforward;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(SwerveDriveIO io, int index) {
    this.io = io;
    this.index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        io.setDrivePIDFF(kDrivingP, kDrivingI, kDrivingD, kDrivingFF);
        io.setTurnPIDFF(kTurningP, kTurningI, kTurningD, kTurningFF);
        break;
      case SIM:
        io.setDrivePIDFF(1, 0, 0, 0.0935);
        io.setTurnPIDFF(5, 0, 0, 0.12);
        break;
      default:
        io.setDrivePIDFF(0, 0, 0, 0);
        io.setTurnPIDFF(0, 0, 0, 0);
        break;
    }

    setBrakeMode(true);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), (LoggableInputs) inputs);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    // if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
    // turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    // }

    // Run closed loop turn control
    if (angleSetpoint != null) {
      io.setTurnPosition(angleSetpoint.getRadians());
      Logger.recordOutput(
          "Drive/Module" + Integer.toString(index) + "/Turn Setpoint", angleSetpoint);

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90 degrees, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        // double adjustSpeedSetpoint = speedSetpoint *
        // Math.cos(io.getTurnPositionError(angleSetpoint.getRadians()));

        // Run drive controller
        // double velocityRadPerSec = adjustSpeedSetpoint;

        io.setDriveVelocity(speedSetpoint);
      }
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS;
      Rotation2d angle =
          inputs.odometryTurnPositions[i].plus(
              turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    state.optimize(getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = state.angle;
    speedSetpoint = state.speedMetersPerSecond;

    return state;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  public void runCharacterization(double driveVolts, double angleVolts) {
    // Closed loop turn control
    angleSetpoint = null;
    speedSetpoint = null;

    // Open loop drive control
    io.setDriveVoltage(driveVolts);
    io.setTurnVoltage(angleVolts);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset == null) {
      return inputs.turnPosition;
    } else {
      return inputs.turnPosition.plus(turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMeterPerSec;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  public void setTurnVoltage(double voltage) {
    setTurnVoltage(voltage);
  }
}
