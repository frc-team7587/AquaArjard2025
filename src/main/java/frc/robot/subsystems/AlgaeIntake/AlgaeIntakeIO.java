package frc.robot.subsystems.AlgaeIntake;

public interface AlgaeIntakeIO {
    /**
     * Sets the speed of the intake motors.
     * @param speed The speed to set the intake motor to.
     */
    public void setIntakeSpeed(double speed);

    /**
     * Sets the speed of the pivot motor.
     * @param speed The speed to set the pivot motor to.
     */
    public void setPivotSpeed(double speed);

    /**
     * Sets the position of the pivot motor.
     * @param position The position to set the pivot motor to.
     */
    public void setPivotPosition(double position);

    /**
     * Gets the encoder value of the pivot motor.
     * @return The encoder value of the pivot motor.
     */
    public double getPivotPosition();

    /**
     * Resets the encoder value of the pivot motor.
     */
    public void reset();
}