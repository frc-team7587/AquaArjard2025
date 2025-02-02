package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO elevator;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;
  }

  // Moves the elevator up at the specified speed
  public Command elevatorUp() {
    return run(
      () -> elevator.elevatorUp(ElevatorConstants.kElevatorUpSpeed)
    );
  }

  // Moves the elevator down at the specified speed
  public Command elevatorDown() {
    return run(
      () -> elevator.elevatorDown(ElevatorConstants.kElevatorDownSpeed)
    );
  }

  // Sets the elevator to a specific position
  public Command setElevatorPosition(double position) {
    return run(
      () -> elevator.setElevatorPosition(position)
    );
  }

  // Stops the elevator movement
  public Command elevatorStop() {
    return run(
      () -> elevator.elevatorStop()
    );
  }
  
  // Resets the elevator position to zero
  public Command resetElevatorPosition() {
    return run(
      () -> elevator.resetElevatorPosition()
    );
  }
  // Gets the current position of the elevator to put on SmartDashboard
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", elevator.getElevatorPosition());
  }
}
