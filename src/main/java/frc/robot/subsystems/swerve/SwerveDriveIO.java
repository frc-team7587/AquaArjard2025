package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveDriveIO {

    @AutoLog
    public static class SwerveDriveIOInputs {
      public double drivePositionRad = 0.0;
      public double drivePositionMeters = 0.0;
      public double driveVelocityRadPerSec = 0.0;
      public double driveVelocityMeterPerSec = 0.0;
      public double driveAppliedVolts = 0.0;
      public double[] driveCurrentAmps = new double[] {};
  
      public Rotation2d turnAbsolutePosition = new Rotation2d();
      public Rotation2d turnPosition = new Rotation2d();
      public double turnVelocityRadPerSec = 0.0;
      public double turnAppliedVolts = 0.0;
      public double[] turnCurrentAmps = new double[] {};
  
      public double[] odometryTimestamps = new double[] {};
      public double[] odometryDrivePositionsRad = new double[] {};
      public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }
    /**
     * Get the position of the driving motor.
     * @return The position of the driving motor in rotations.
     */
    public double getDrivingPosition();

    /**
     * Get the position of the turning motor, subtracting the chassis angular offset.
     * @return The position of the turning motor in rotations, subtracting the chassis angular offset.
     */
    public double getTurningPosition();

    /**
     * Gets the velocity of the driving motor.
     * @return The velocity of the driving motor in revolutions per minute.
     */
    public double getDrivingVelocity();

    /**
     * Gets the velocity of the turning motor.
     * @return The velocity of the turning motor in revolutions per minute.
     */
    public double getTurningVelocity();

    /**
     * Gets the state of the swerve module.
     * @return The swerve module state.
     */
    public SwerveModuleState getState();

    /**
     * Gets the position of the swerve module.
     * @return The swerve module position.
     */
    public SwerveModulePosition getPosition();

    /**
     * Sets the swerve module to the desired state.
     * @param desiredState The state to set the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState);

    /** Resets the driving encoder. */
    public void resetEncoder();

    public void updateInputs(SwerveDriveIOInputs inputs);
    public void setDriveVoltage(double volts);
    public void setTurnVoltage(double volts);
    public void setDriveVelocity(double velocityRadPerSec);
    public void setTurnPosition(double angle);
    public void setDrivePIDFF(double p, double i, double d, double ff);
    public void setTurnPIDFF(double p, double i, double d, double ff);
    public double getTurnPositionError(double angle);

    public void setDriveBrakeMode(boolean enabled);

    public void setTurnBrakeMode(boolean enabled);




}