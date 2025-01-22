package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveDriveIO {
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
}