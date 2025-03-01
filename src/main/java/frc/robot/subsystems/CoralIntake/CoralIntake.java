package frc.robot.subsystems.CoralIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase{
    private final CoralIntakeIO intake;

    public CoralIntake(CoralIntakeIO intake){
        this.intake = intake;
    }



    public Command intakeCoral(){
        return startEnd(
            () -> intake.setIntakeSpeed(CoralIntakeConstants.kIntakeInSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    public Command outtakeCoral(){
        return startEnd(
            () -> intake.setIntakeSpeed(CoralIntakeConstants.kIntakeOutSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    public Command turntoUp(){
        return startEnd(
            () -> intake.setPivotSpeed(CoralIntakeConstants.kPivotSpeedUp),
            () -> intake.setIntakeSpeed(0)
        );

    }
    public Command turntoNeutral(){
        return run(() -> intake.setPivotPosition(CoralIntakeConstants.kPivotNeutalPosition));
    }
    public Command turntoDown(){
        return startEnd(
            () -> intake.setPivotSpeed(CoralIntakeConstants.kPivotSpeedDown),
            () -> intake.setIntakeSpeed(0)
        );
    }
    public Command intakeIn(){
        return run(() -> intake.setIntakeSpeed(CoralIntakeConstants.kIntakeInSpeed));
    }

    public Command intakeOut(){
        return run(() -> intake.setIntakeSpeed(CoralIntakeConstants.kIntakeOutSpeed));
    }
    public void resetEncoder(){
        intake.reset();
    }
    public Command stopIntake(){
        return run(() -> intake.setIntakeSpeed(0));
    }

    public Command setPivotPosition(double position){
        return run(() -> intake.setPivotPosition(position));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Coral Intake Position", Math.round(intake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }

}