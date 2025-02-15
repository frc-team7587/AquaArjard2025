package frc.robot.subsystems.coralOutake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;

public class CoralOutake extends SubsystemBase {
    private final CoralOutakeIO coralOutake;

    public CoralOutake(CoralOutakeIO coralOutake) {
        this.coralOutake = coralOutake;
    }

    public Command intakeCoral() {
        return run(
            () -> coralOutake.setIndexerSpeed(CoralIntakeConstants.kCoralIntakeSpeed));
    }

    public Command shootCoral() {
        return run(
            () -> coralOutake.setIndexerSpeed(CoralIntakeConstants.kCoralShootSpeed));
    }

    public Command pivotCoralUp() {
        return run(
            () -> coralOutake.setPivotSpeed(CoralIntakeConstants.kCoralPivotUpSpeed));
    }

    public Command pivotCoralDown() {
        return run(
            () -> coralOutake.setPivotSpeed(CoralIntakeConstants.kCoralPivotDownSpeed));
    }

    public Command stopMotor() {
        return run(
            () -> coralOutake.stop());
    }
}
