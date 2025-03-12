// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO;
import frc.robot.subsystems.CoralIntake.CoralIntakeSparkMax;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static double getPeriod;

  private final RobotContainer m_robotContainer;
  private final SwerveDrive m_drive = new SwerveDrive();
  private final XboxController m_driverController = new XboxController(0);
  
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Field2d field;
  

  public Robot() {
    m_robotContainer = new RobotContainer();
        field = new Field2d();
        SmartDashboard.putData("Field", field);

    

  }

   // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .007;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= (DriveConstants.kMaxAngularSpeed/3);

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    //double kP = .2; //follow mode
    double kP = 0.05;  //align mode
    double desiredArea = 7.1;
    double distanceError = desiredArea - LimelightHelpers.getTA("limelight");
    double targetingForwardSpeed = distanceError * kP * DriveConstants.kMaxSpeedMetersPerSecond;
    return targetingForwardSpeed;
  }

  double limmelight_strafe_proportional(boolean left)
  {
    // if(left){
    double kP = .05;
    double targetingSidewaysSpeed = LimelightHelpers.getTX("limelight") * kP * (DriveConstants.kMaxSpeedMetersPerSecond/3) * -1.0;
    return targetingSidewaysSpeed;
    // } else {
    //   double kP = .05;
    // double targetingSidewaysSpeed = LimelightHelpers.getTX("limelight")  * kP * (DriveConstants.kMaxSpeedMetersPerSecond/3) * -1.0;
    // return targetingSidewaysSpeed + (25 * kP * (DriveConstants.kMaxSpeedMetersPerSecond/3));
    // }
  };

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
    -MathUtil.applyDeadband((1 - 0.75 * m_driverController.getRightTriggerAxis()) * m_driverController.getLeftY(), OIConstants.kDriveDeadband);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
    -MathUtil.applyDeadband((1 - 0.75 * m_driverController.getRightTriggerAxis()) * m_driverController.getLeftX(), OIConstants.kDriveDeadband);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot = 
    -MathUtil.applyDeadband(0.5 * m_driverController.getRightX(), OIConstants.kDriveDeadband);

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    // if(m_driverController.getBButton())
    // {
    //     final var rot_limelight = limelight_aim_proportional();
    //     rot = rot_limelight;

    //     final var forward_limelight = limelight_range_proportional();
    //     xSpeed = forward_limelight;

    //     final var sideways_limelight = limmelight_strafe_proportional(false);
    //     ySpeed = sideways_limelight;

    //     //while using Limelight, turn off field-relative driving.
    //     fieldRelative = false;
    // }
    if(m_driverController.getXButton())
    {
        // final var rot_limelight = limelight_aim_proportional();
        // rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        final var sideways_limelight = limmelight_strafe_proportional(true);
        ySpeed = sideways_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }

    m_drive.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }

/* 
  double[] limelight_align_proportional(){
    
    // Retrieve Limelight offsets
    double tx = LimelightHelpers.getTX("limelight");
    double ta = LimelightHelpers.getTA("limelight");
    double[] tAngle = LimelightHelpers.getBotPose("limelight");


    // Define proportional control constants
    double kP_x = 0.02;
    double kP_a = 0.02;
    double kP_rot = 0.01;

    double desiredArea = 6.28;

    // Calculate translation speeds
    double xSpeed = (desiredArea - ta) * kP_a * (DriveConstants.kMaxSpeedMetersPerSecond / 2);
    double ySpeed = tx * kP_x * (DriveConstants.kMaxSpeedMetersPerSecond / 2) * -1.0;
    double rot = 


    // Set rotation to zero
    double rot = 0.0;

    // Disable field-relative driving
    boolean fieldRelative = false;

    // Return the calculated speeds
    return new double[] {xSpeed, ySpeed, rot};

  }
  private void limelightDrive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.02))
            * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.02))
            * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightX(), 0.02))
            * DriveConstants.kMaxAngularSpeed;
          
    xSpeed = limelight_align_proportional()[0];
    ySpeed = limelight_align_proportional()[1];
    rot = limelight_align_proportional()[2];

    //while using Limelight, turn off field-relative driving.
    fieldRelative = false;

    m_drive.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
  private void drive(boolean fieldRelative) {
    m_drive.setDefaultCommand(
         // The left stick controls translation of the robot.
         // Turning is controlled by the X axis of the right stick.
         new RunCommand(
             () -> m_drive.drive(
                 -MathUtil.applyDeadband((1 - 0.75 * m_driverController.getRightTriggerAxis()) * m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                 -MathUtil.applyDeadband((1 - 0.75 * m_driverController.getRightTriggerAxis()) * m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                 -MathUtil.applyDeadband(0.5 * m_driverController.getRightX(), OIConstants.kDriveDeadband),
                 true, Robot.getPeriod),
            m_drive
        )
    );
  }
*/
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  //   if (m_driverController.getAButton()) {
  //     limelightDrive(false);
  // } else {
  //     drive(true);
  // }

  drive(true);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(m_drive.getPose());
    
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        field.getObject("path").setPoses(poses);
    });
  }

  @Override
  public void robotInit(){
    // m_CoralIntake.setPivotPosition(4.5);

  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_drive.moveBackAuto();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    //m_drive.drive(-0.5,0,0, true, getPeriod);

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    getPeriod = getPeriod();
  }

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
}
