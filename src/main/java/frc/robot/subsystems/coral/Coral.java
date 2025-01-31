package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    private final CoralIO coral;

    public Coral(CoralIO coral) {
        this.coral = coral;
    }

    public Command intakeCoral() {
        return run(
            () -> coral.setIndexerSpeed(CoralConstants.kCoralIntakeSpeed));
    }

    public Command shootCoral() {
        return run(
            () -> coral.setIndexerSpeed(CoralConstants.kCoralShootSpeed));
    }

    public Command pivotCoralUp() {
        return run(
            () -> coral.setPivotSpeed(CoralConstants.kCoralPivotUpSpeed));
    }

    public Command pivotCoralDown() {
        return run(
            () -> coral.setPivotSpeed(CoralConstants.kCoralPivotDownSpeed));
    }

    public Command stopMotor() {
        return run(
            () -> coral.stop());
    }
}
