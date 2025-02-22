package frc.robot.subsystems.algaeOuttake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;

public class AlgaeOuttake extends SubsystemBase {
    private final AlgaeOuttakeIO algaeOuttake;

    public AlgaeOuttake(AlgaeOuttakeIO algaeOuttake) {
        this.algaeOuttake = algaeOuttake;
    }

    public Command outtakeAlgae(){
        return startEnd(
            () -> algaeOuttake.setOuttakeSpeed(AlgaeIntakeConstants.kIntakeOutSpeed),
            () -> algaeOuttake.setOuttakeSpeed(0));
    }

    public Command shootAlgae() {
        return run(
            () -> algaeOuttake.setOuttakeSpeed(AlgaeOuttakeConstants.kAlgaeShootSpeed));
    }

    public Command stopMotor() {
        return run(
            () -> algaeOuttake.stop());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Algae Outtake Pivot Position", Math.round(algaeOuttake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}
