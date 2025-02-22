package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private final AlgaeIntakeIO algaeIntake;

    public AlgaeIntake(AlgaeIntakeIO algaeIntake) {
        this.algaeIntake = algaeIntake;
    }

    public Command sequenceAlgae(){
        return new ScheduleCommand(
            turntoNeutral().withTimeout(1),
            intakeAlgae().withTimeout(1),
            turntoUp().withTimeout(1)
        );
    }

    public Command intakeAlgae(){
        return startEnd(
            () -> algaeIntake.setIntakeSpeed(AlgaeIntakeConstants.kIntakeInSpeed),
            () -> algaeIntake.setIntakeSpeed(0));
    }

    public Command turntoUp(){
        return run(() -> algaeIntake.setPivotPosition(AlgaeIntakeConstants.kIntakeUpPosition));
    }

    public Command turntoNeutral(){
        return run(() -> algaeIntake.setPivotPosition(AlgaeIntakeConstants.kIntakeNeutralPosition));
    }

    public Command turntoDown(){
        return run(() -> algaeIntake.setPivotPosition(AlgaeIntakeConstants.kIntakeDownPosition));
    }

    public Command intakeIn(){
        return run(() -> algaeIntake.setIntakeSpeed(AlgaeIntakeConstants.kIntakeInSpeed));
    }

    public Command intakeOut(){
        return run(() -> algaeIntake.setIntakeSpeed(AlgaeIntakeConstants.kIntakeOutSpeed));
    }

    public void resetEncoder(){
        algaeIntake.reset();
    }

    public Command stopIntake(){
        return run(() -> algaeIntake.setIntakeSpeed(0));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Algae Intake Pivot Position", Math.round(algaeIntake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}
