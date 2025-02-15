package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private final CoralIntakeIO coral;

    public CoralIntake(CoralIntakeIO coral) {
        this.coral = coral;
    }
    
    public Command intakeCoral() {
        return run(
            () -> coral.setIndexerSpeed(CoralIntakeConstants.kCoralIntakeSpeed));
    }

    public Command shootCoral() {
        return run(
            () -> coral.setIndexerSpeed(CoralIntakeConstants.kCoralShootSpeed));
    }

    public Command pivotCoralUp() {
        return run(
            () -> coral.setPivotSpeed(CoralIntakeConstants.kCoralPivotUpSpeed));
    }

    public Command pivotCoralDown() {
        return run(
            () -> coral.setPivotSpeed(CoralIntakeConstants.kCoralPivotDownSpeed));
    }

    public Command stopMotor() {
        return run(
            () -> coral.stop());
    }
}
