package frc.robot.subsystems.algaeIntake;

public interface AlgaeIntakeIO {
    public void setIntakeSpeed(double speed);
    public void setPivotSpeed(double speed);
    public void setPivotPosition(double position);
    public double getPivotPosition();
    public void reset();
}
