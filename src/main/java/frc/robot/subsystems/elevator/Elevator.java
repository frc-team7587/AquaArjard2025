package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevator;

    public Elevator(ElevatorIO elevator) {
        this.elevator = elevator;
    }

    public Command elevatorToL1OfTheReef() {
        return run(
            () -> elevator.setElevatorPosition(ElevatorConstants.kHeightOfL1InInches - ElevatorConstants.kHeightOfElevatorInInches));
    }

    public Command elevatorToL2OfTheReef() {
        return run(
            () -> elevator.setElevatorPosition(ElevatorConstants.kHeightOfL2InInches - ElevatorConstants.kHeightOfElevatorInInches));
    }

    public Command elevatorToL3OfTheReef() {
        return run(
            () -> elevator.setElevatorPosition(ElevatorConstants.kHeightOfL3InInches - ElevatorConstants.kHeightOfElevatorInInches));
    }

    public Command elevatorToL4OfTheReef() {
        return run(
            () -> elevator.setElevatorPosition(ElevatorConstants.kHeightOfL4InInches - ElevatorConstants.kHeightOfElevatorInInches));
    }

    public Command elevatorToBase() {
        return run(
            () -> elevator.setElevatorPosition(0));
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