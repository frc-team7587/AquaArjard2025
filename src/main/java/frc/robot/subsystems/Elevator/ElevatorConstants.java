package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants;

public class ElevatorConstants {
    public static final double kElevatorUpSpeed = 0.5;
    public static final double kElevatorDownSpeed = -0.01;

    public static final double kMaxVelocity = 2.0;
    public static final double kMaxAcceleration = 2.0;

    /** Final position conversion factor based on drum radius in meters and gear ratio */
  public static double kPositionConversionFactor =
      4 * Constants.PI * 0.0508 * (1.0 / 6.0);
    /** Tolerance used when checking if the elevator is at the setpoint */
    public static final double kSetpointToleranceMeters = 0.05;

    public static final int kElevatorLeftMotorID = 10;
    public static final int kElevatorRightMotorID = 11;

    public static final double kElevatorMaxHeight = 29.0;
    public static final double kElevatorMinHeight = 0.0;

    public static final double kElevatorLevel0 = 0.0;
    public static final double kElevatorLevel1 = 4.0;
    public static final double kElevatorLevel2 = 8.0;
    public static final double kElevatorLevel3 = 0.0;
    public static final double kElevatorLevel4 = 0.0;

    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.05;
    public static final ElevatorFeedforward kFF = new ElevatorFeedforward(0.5,1.44,4.6,0.15);
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
}
