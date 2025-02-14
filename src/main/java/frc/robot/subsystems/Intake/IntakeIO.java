package frc.robot.subsystems.Intake;

public interface IntakeIO {
    public void setIntakeSpeed(double speed);
    public void setPivotSpeed(double speed);
    public void setPivotPosition(double position);
    public double getPivotPosition();
    public void reset();
}