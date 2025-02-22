package frc.robot.subsystems.coralOuttake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralOuttake extends SubsystemBase {
    private final CoralOuttakeIO coralOuttake;

    public CoralOuttake(CoralOuttakeIO coralOuttake) {
        this.coralOuttake = coralOuttake;
    }

    public Command sequenceCoral(){
        return new ScheduleCommand(
            turntoNeutral().withTimeout(1),
            OuttakeCoral().withTimeout(1),
            turntoUp().withTimeout(1)
        );
    }

    public Command OuttakeCoral(){
        return startEnd(
            () -> coralOuttake.setIndexerSpeed(CoralOuttakeConstants.kCoralShootSpeed),
            () -> coralOuttake.setIndexerSpeed(0));
    }

    public Command turntoUp(){
        return run(() -> coralOuttake.setPivotPosition(CoralOuttakeConstants.kOuttakeUpPosition));
    }

    public Command turntoNeutral(){
        return run(() -> coralOuttake.setPivotPosition(CoralOuttakeConstants.kOuttakeCoralPosition));
    }

    public Command turntoDown(){
        return run(() -> coralOuttake.setPivotPosition(CoralOuttakeConstants.kOuttakeDownPosition));
    }

    public Command OuttakeIn(){
        return run(() -> coralOuttake.setIndexerSpeed(CoralOuttakeConstants.kCoralShootSpeed));
    }

    public void resetEncoder(){
        coralOuttake.reset();
    }

    public Command stopOuttake(){
        return run(() -> coralOuttake.setIndexerSpeed(0));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Coral Outtake Pivot Position", Math.round(coralOuttake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}
