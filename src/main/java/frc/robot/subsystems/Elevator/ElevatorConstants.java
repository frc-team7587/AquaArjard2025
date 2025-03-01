package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorConstants {
    public static final double kElevatorUpSpeed = 0.5;
    public static final double kElevatorDownSpeed = -0.01;

    public static final double kMaxVelocity = 2.0;
    public static final double kMaxAcceleration = 2.0;

    public static final int kElevatorLeftMotorID = 10;
    public static final int kElevatorRightMotorID = 11;

    public static final double kElevatorMaxHeight = 0.0;
    public static final double kElevatorMinHeight = 0.0;

    public static final double kElevatorLevel0 = 0.0;
    public static final double kElevatorLevel1 = 4.3;
    public static final double kElevatorLevel2 = 13.3;
    public static final double kElevatorLevel3 = 29;
    public static final double kElevatorLevel4 = 17.0;

    // public static final double kP = 1.4;
    // public static final double kI = 0.02;
    // public static final double kD = 12;
     //public static final double kFF = new ElevatorFeedforward(1.175,1.625,4.6,0.15).calculate(0, 0);

    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = new ElevatorFeedforward(0.1,1.44,0.6,0.05).calculate(0);
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    public static final double kElevatorVoltage = 3; //og 0.9
    //output up = 1.1
    //output down = 1
}