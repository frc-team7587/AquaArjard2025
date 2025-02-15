package frc.robot.subsystems.algaeOutake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeOutake extends SubsystemBase {
    private final AlgaeOutakeIO algaeOutake;

    public AlgaeOutake(AlgaeOutakeIO algaeOutake) {
        this.algaeOutake = algaeOutake;
    }

    public Command intakeAlgae() {
        return run(
            () -> algaeOutake.setMotorSpeed(AlgaeOutakeConstants.kAlgaeIntakeSpeed));
    }

    public Command shootAlgae() {
        return run(
            () -> algaeOutake.setMotorSpeed(AlgaeOutakeConstants.kAlgaeShootSpeed));
    }

    public Command stopMotor() {
        return run(
            () -> algaeOutake.stop());
    }
}
