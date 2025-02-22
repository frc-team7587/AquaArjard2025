package frc.robot.subsystems.coralIntake;

public interface CoralIntakeIO {
    public void setIndexerSpeed(double speed);
    public void setPivotSpeed(double speed);
    public void setPivotPosition(double position);
    public double getPivotPosition();
    public boolean getBreakBeamSensorValue();
    public void reset();
    public void stop();
}
