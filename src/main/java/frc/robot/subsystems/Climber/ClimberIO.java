package frc.robot.subsystems.Climber;

public interface ClimberIO {
    /**
     * Sets the speed of the climber motor.
     * @param speed The speed to set the climber motor to.
     */
    public void setClimberSpeed(double speed);

    /**
     * Sets the voltage of the climber motor.
     * @param speed The voltage to set the climber motor to.
     */
    public void setClimberVoltage(double voltage);

    /**
     * Sets the position of the climber motor.
     * @param position The position to set the climber motor to.
     */
    public void setClimberPosition(double position);

    /**
     * Gets the encoder value of the climber motor.
     * @return The encoder value of the climber motor.
     */
    public double getClimberPosition();

    /**
     * Resets the encoder value of the climber motor.
     */
    public void reset();
}
