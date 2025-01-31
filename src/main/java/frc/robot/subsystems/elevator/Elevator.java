package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevator;

    public Elevator(ElevatorIO elevator) {
        this.elevator = elevator;
    }

    public Command elevatorUp() {
        return run(
            () -> elevator.setMotorSpeed(ElevatorConstants.kElevatorUpSpeed));
    }

    public Command elevatorDown() {
        return run(
            () -> elevator.setMotorSpeed(ElevatorConstants.kElevatorUpSpeed));
    }

    public Command stopElevator() {
        return run(
            () -> elevator.stop());
    }
}