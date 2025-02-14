package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeIO intake;

    public Intake(IntakeIO intake){
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
            () -> intake.setIntakeSpeed(IntakeConstants.kIntakeInSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    public Command outtakeAlgae(){
        return startEnd(
            () -> intake.setIntakeSpeed(IntakeConstants.kIntakeOutSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    public Command turntoUp(){
        return run(() -> intake.setPivotPosition(IntakeConstants.kIntakeUpPosition));
    }
    public Command turntoNeutral(){
        return run(() -> intake.setPivotPosition(IntakeConstants.kIntakeNeutralPosition));
    }
    public Command turntoDown(){
        return run(() -> intake.setPivotPosition(IntakeConstants.kIntakeDownPosition));
    }
    public Command intakeIn(){
        return run(() -> intake.setIntakeSpeed(IntakeConstants.kIntakeInSpeed));
    }
    public Command intakeOut(){
        return run(() -> intake.setIntakeSpeed(IntakeConstants.kIntakeOutSpeed));
    }
    public void resetEncoder(){
        intake.reset();
    }
    public Command stopIntake(){
        return run(() -> intake.setIntakeSpeed(0));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Position", Math.round(intake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }

}