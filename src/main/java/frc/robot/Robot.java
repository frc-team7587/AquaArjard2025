// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.LimelightHelpers;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final SwerveDrive m_swerve;

  XboxController  m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public static double period;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_swerve = new SwerveDrive();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    period = this.getPeriod();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .015;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= SwerveConstants.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = 0.5;
    double targetingForwardSpeed = LimelightHelpers.getTA("limelight") * kP;
    targetingForwardSpeed *= SwerveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  double limelight_distance(){
    double limelightMountAngleDegrees = 3.3;
    double limelightLensHeightInches = 9;
    double goalHeightInches = 13;
    double kP = 0.05;

    double targetOffSetAngle_Vertical = LimelightHelpers.getTY("limelight");

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffSetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelighToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    double targetingForwardSpeed = distanceFromLimelighToGoalInches * kP;
    targetingForwardSpeed *= SwerveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;



  }

  double[] limelight_align_and_range() {
    // Constants for proportional control
    double kP_Aim = 0.025;   // Aiming proportional gain (tx)
    double kP_Align = 0.02;  // Alignment proportional gain (ty)
    double kP_Range = 0.25;   // Distance proportional gain (ta)

    // Get Limelight data
    double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset
    double ty = LimelightHelpers.getTY("limelight");  // Vertical offset
    double ta = LimelightHelpers.getTA("limelight");  // Target area

    // Calculate angular velocity for aiming
    double aimingRot = -tx * kP_Aim * SwerveConstants.kMaxAngularSpeed;

    // Desired area (distance goal) - This should be tuned based on real-world measurements
    double desiredArea = 1.5;  // Example: adjust based on desired distance
    double distanceError = desiredArea - ta;

    // Forward/backward speed for maintaining distance
    double forwardSpeed = distanceError * kP_Range * SwerveConstants.kMaxSpeedMetersPerSecond;

    // Strafe speed for aligning parallel
    double strafeSpeed = -ty * kP_Align * SwerveConstants.kMaxSpeedMetersPerSecond;

    // Return an array of values
    return new double[]{forwardSpeed, strafeSpeed, aimingRot};
}

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.07))
            * SwerveConstants.kMaxSpeedMetersPerSecond;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.07))
            * SwerveConstants.kMaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.07))
            * SwerveConstants.kMaxAngularSpeed;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    if(m_controller.getAButton() && LimelightHelpers.getFiducialID("limelight") == 2) {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;
    } 
    // Override manual control when A button is pressed
    if (m_controller.getBButton()&& LimelightHelpers.getFiducialID("limelight") == 2 ) {
      double[] limelightOutputs = limelight_align_and_range();
      xSpeed = limelightOutputs[0]; // Forward/backward
      //ySpeed = limelightOutputs[1]; // Sideways alignment
      rot = limelightOutputs[2];    // Rotation
    }
    /*if(m_controller.getAButton())
    {
    double kPaim = 0.01;
    double kPdistance = 0.05;

    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kPaim;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kPdistance;
      double min_aim_command = 0.05;
      final var heading_error = -targetingAngularVelocity;
      final var distance_error = -targetingForwardSpeed;
      var steering_adjust = 0.0;

      if (targetingAngularVelocity > 1.0) {
        steering_adjust = kPaim * heading_error - min_aim_command;
      } else if (targetingAngularVelocity < -1.0)
      {
        steering_adjust = kPaim * heading_error + min_aim_command;
      }

      final var distance_adjust = kPdistance * distance_error;
      xSpeed += steering_adjust + distance_adjust;

    }*/


    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
   }
}