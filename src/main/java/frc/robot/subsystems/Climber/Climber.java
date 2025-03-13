package frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private ClimberIO climber;

    public Climber(ClimberIO climber) {
        this.climber = climber;
    }

    public Command hangOnChain() {
        return run(
            () -> climber.setClimberPosition(ClimberConstants.kClimberHangPosition));
    }

    public Command getOffChain() {
        return run(
            () -> climber.setClimberPosition(ClimberConstants.kClimberGetOffPosition));
    }
    
    public Command climbUp() {
        return startEnd(
            () -> climber.setClimberSpeed(ClimberConstants.kClimberUpSpeed),
            () -> climber.setClimberSpeed(0));
    }

    public Command climbDown() {
        return startEnd(
            () -> climber.setClimberSpeed(ClimberConstants.kClimberDownSpeed),
            () -> climber.setClimberSpeed(0));
    }

    public Command turntoNeutral() {
        return run(() -> climber.setClimberPosition(ClimberConstants.kClimberNeutalPosition));
    }

    public void resetEncoder() {
        climber.reset();
    }
    public Command stopClimber() {
        return run(() -> climber.setClimberSpeed(0));
    }

    public Command setClimberPosition(double position) {
        return run(() -> climber.setClimberPosition(position));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Intake Position", Math.round(climber.getClimberPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}
