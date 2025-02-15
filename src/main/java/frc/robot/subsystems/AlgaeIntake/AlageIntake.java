package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlageIntake extends SubsystemBase{
    private final AlgaeIntakeIO intake;

    public AlageIntake(AlgaeIntakeIO intake){
        this.intake = intake;
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
            () -> intake.setIntakeSpeed(AlgaeIntakeConstants.kIntakeInSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    public Command outtakeAlgae(){
        return startEnd(
            () -> intake.setIntakeSpeed(AlgaeIntakeConstants.kIntakeOutSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    public Command turntoUp(){
        return run(() -> intake.setPivotPosition(AlgaeIntakeConstants.kPivotMaxPosition));
    }
    public Command turntoNeutral(){
        return run(() -> intake.setPivotPosition(AlgaeIntakeConstants.kPivotNeutalPosition));
    }
    public Command turntoDown(){
        return run(() -> intake.setPivotPosition(AlgaeIntakeConstants.kPivotMinPosition));
    }
    public Command intakeIn(){
        return run(() -> intake.setIntakeSpeed(AlgaeIntakeConstants.kIntakeInSpeed));
    }
    public Command intakeOut(){
        return run(() -> intake.setIntakeSpeed(AlgaeIntakeConstants.kIntakeOutSpeed));
    }
    public void resetEncoder(){
        intake.reset();
    }
    public Command stopIntake(){
        return run(() -> intake.setIntakeSpeed(0));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Algae Intake Position", Math.round(intake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }

}