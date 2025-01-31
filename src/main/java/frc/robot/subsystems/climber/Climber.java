package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberIO climber;

    public Climber(ClimberIO climber) {
        this.climber = climber;
    }

    public Command climberDown() {
        return run(
            () -> climber.setMotorSpeed(ClimberConstants.kClimberDownSpeed));
    }

    public Command climberUp() {
        return run(
            () -> climber.setMotorSpeed(ClimberConstants.kClimberUpSpeed));
    }

    public Command stopElevator() {
        return run(
            () -> climber.stop());
    }
}
