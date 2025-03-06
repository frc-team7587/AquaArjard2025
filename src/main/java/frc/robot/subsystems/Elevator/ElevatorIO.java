package frc.robot.subsystems.Elevator;

public interface ElevatorIO {

    /** 
     * Moves the elevator motors up.
     * @param speed The speed to set the motors to.
     */
    public void elevatorUp(double speed);


    /** 
     * Moves the elevator motors down.
     * @param speed The speed to set the motors to.
     */
    public void elevatorDown(double speed);

    /** 
     * Sets the elevator motors to a specific position.
     * @param position The position to set the motors to.
     */
    public void setElevatorPosition(double position);

    /** 
     * Stops the elevator motors.
     */
    public void elevatorStop();

    /** 
     * Gets the encoder value of the elevator motors.
     * @return The encoder value of the elevator motors.
     */
    public double getElevatorPosition();

    /** 
     * Resets the encoder value of the elevator motors.
     */
    public void resetElevator();
    public double getLvoltage();
    public double getRvoltage();
    public double getLcurrent();
    public double getRcurrent();
    public double getLoutput();
    public double getRoutput();
    public double getLtemp();
    public double getRtemp();

}
