package frc.robot.subsystems.algaeOuttake;

public interface AlgaeOuttakeIO {
    public void setOuttakeSpeed(double speed);
    public void setPivotSpeed(double speed);
    public void setPivotPosition(double position);
    public double getPivotPosition();
    public void reset();
    public void stop();
}
