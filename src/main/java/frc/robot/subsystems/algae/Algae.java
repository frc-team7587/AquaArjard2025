package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
    private final AlgaeIO algae;

    public Algae(AlgaeIO algae) {
        this.algae = algae;
    }

    public Command intakeAlgae() {
        return run(
            () -> algae.setMotorSpeed(AlgaeConstants.kAlgaeIntakeSpeed));
    }

    public Command shootAlgae() {
        return run(
            () -> algae.setMotorSpeed(AlgaeConstants.kAlgaeShootSpeed));
    }

    public Command stopMotor() {
        return run(
            () -> algae.stop());
    }
}
