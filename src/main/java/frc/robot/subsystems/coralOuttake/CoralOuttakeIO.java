package frc.robot.subsystems.coralOuttake;

public interface CoralOuttakeIO {
    public void setIndexerSpeed(double speed);
    public void setPivotSpeed(double speed);
    public void setPivotPosition(double position);
    public double getPivotPosition();
    public boolean getBreakBeamSensorValue();
    public void reset();
    public void stop();
}