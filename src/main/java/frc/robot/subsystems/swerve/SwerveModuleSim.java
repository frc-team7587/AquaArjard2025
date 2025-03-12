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

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class SwerveModuleSim implements SwerveDriveIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 4.71), DCMotor.getNEO(1));
  private DCMotorSim turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.004, 150.0 / 7.0),
          DCMotor.getNeo550(1));

  private final Rotation2d turnAbsoluteInitPosition =
      new Rotation2d(); // new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController driveFeedback = new PIDController(0.1, 0.0, 0.0);
  private final PIDController turnFeedback = new PIDController(10.0, 0.0, 0.0);

  private double lastTurnSetpoint = 0;
  private double lastTime = Timer.getFPGATimestamp();

  public SwerveModuleSim() {
    turnFeedback.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void updateInputs(SwerveDriveIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.drivePositionMeters = driveSim.getAngularPositionRad() * ModuleConstants.kWheelDiameterMeters / 2;
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveVelocityMeterPerSec =
        driveSim.getAngularVelocityRadPerSec() * ModuleConstants.kWheelDiameterMeters / 2;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    velocityRadPerSec *= 2 / ModuleConstants.kWheelDiameterMeters;
    setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec));
  }

  @Override
  public void setTurnPosition(double angle) {
    double dtheta = angle - lastTurnSetpoint;
    if (dtheta > Math.PI) {
      dtheta -= 2 * Math.PI;
    } else if (dtheta < -Math.PI) {
      dtheta += 2 * Math.PI;
    }

    double setpointVelocity = dtheta / (Timer.getFPGATimestamp() - lastTime);
    lastTime = Timer.getFPGATimestamp();
    lastTurnSetpoint = angle;
    double ffOutput = turnFeedforward.calculate(setpointVelocity);
    setTurnVoltage(turnFeedback.calculate(turnSim.getAngularPositionRad(), angle) + ffOutput);
  }

  @Override
  public void setDrivePIDFF(double p, double i, double d, double ff) {
    driveFeedback.setPID(p, i, d);
    driveFeedforward.setKv(ff);
  }

  @Override
  public void setTurnPIDFF(double p, double i, double d, double ff) {
    turnFeedback.setPID(p, i, d);
    turnFeedforward.setKv(ff);
  }

  @Override
  public double getTurnPositionError(double angle) {
    return turnFeedback.getPositionError();
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
public SwerveModuleState getState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getState'");
}

@Override
public SwerveModulePosition getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
}

@Override
public void setDesiredState(SwerveModuleState desiredState) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDesiredState'");
}

@Override
public void resetEncoder() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetEncoder'");
}



@Override
public void setDriveBrakeMode(boolean enabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveBrakeMode'");
}

@Override
public void setTurnBrakeMode(boolean enabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTurnBrakeMode'");
}
}