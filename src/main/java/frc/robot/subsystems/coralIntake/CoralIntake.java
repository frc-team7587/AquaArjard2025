package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private final CoralIntakeIO coralIntake;

    public CoralIntake(CoralIntakeIO coralIntake) {
        this.coralIntake = coralIntake;
    }

    public Command intakeCoral() {
        return run(
            () -> coralIntake.setIndexerSpeed(CoralIntakeConstants.kCoralIntakeSpeed));
    }

    public Command pivotCoralUp() {
        return run(
            () -> coralIntake.setPivotSpeed(CoralIntakeConstants.kCoralPivotUpSpeed));
    }

    public Command pivotCoralDown() {
        return run(
            () -> coralIntake.setPivotSpeed(CoralIntakeConstants.kCoralPivotDownSpeed));
    }

    public Command stopMotor() {
        return run(
            () -> coralIntake.stop());
    }
}
