package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private final CoralIntakeIO coralIntake;

    public CoralIntake(CoralIntakeIO coralIntake) {
        this.coralIntake = coralIntake;
    }

    public Command sequenceCoral(){
        return new ScheduleCommand(
            turntoNeutral().withTimeout(1),
            intakeCoral().withTimeout(1),
            turntoUp().withTimeout(1)
        );
    }

    public Command intakeCoral(){
        return startEnd(
            () -> coralIntake.setIndexerSpeed(CoralIntakeConstants.kCoralIntakeSpeed),
            () -> coralIntake.setIndexerSpeed(0));
    }

    public Command turntoUp(){
        return run(() -> coralIntake.setPivotPosition(CoralIntakeConstants.kIntakeUpPosition));
    }

    public Command turntoNeutral(){
        return run(() -> coralIntake.setPivotPosition(CoralIntakeConstants.kIntakeCoralPosition));
    }

    public Command turntoDown(){
        return run(() -> coralIntake.setPivotPosition(CoralIntakeConstants.kIntakeDownPosition));
    }

    public Command intakeIn(){
        return run(() -> coralIntake.setIndexerSpeed(CoralIntakeConstants.kCoralIntakeSpeed));
    }

    public void resetEncoder(){
        coralIntake.reset();
    }

    public Command stopIntake(){
        return run(() -> coralIntake.setIndexerSpeed(0));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Coral Intake Pivot Position", Math.round(coralIntake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}
