// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeSparkMax;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralIntake.CoralIntakeSparkMax;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorModule;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Vision.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj.IterativeRobotBase;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveDrive m_robotDrive = new SwerveDrive();
  private final Elevator m_elevator = new Elevator(new ElevatorModule());
  private final CoralIntake m_coralIntake = new CoralIntake(new CoralIntakeSparkMax());
  private final AlgaeIntake m_algaeIntake = new AlgaeIntake(new AlgaeIntakeSparkMax());


  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
    // m_robotDrive.setDefaultCommand(
    //      // The left stick controls translation of the robot.
    //      // Turning is controlled by the X axis of the right stick.
    //      new RunCommand(
    //          () -> m_robotDrive.drive(
    //              -MathUtil.applyDeadband((1 - 0.75 * m_driverController.getRightTriggerAxis()) * m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //              -MathUtil.applyDeadband((1 - 0.75 * m_driverController.getRightTriggerAxis()) * m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //              -MathUtil.applyDeadband(0.5 * m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //              true, Robot.getPeriod),
    //         m_robotDrive
    //     )
    // );
    m_driverController.a().toggleOnTrue(m_elevator.elevatorToLevel0());

    m_driverController.x().toggleOnTrue(m_elevator.elevatorToLevel1());

    m_driverController.b().toggleOnTrue(m_elevator.elevatorToLevel2());

    // m_driverController.x().toggleOnTrue(m_elevator.elevatorToLevel3().andThen(
    //   m_coralIntake.setPivotPosition(2.5)
    // ));
    m_driverController.y().toggleOnTrue(m_elevator.elevatorToLevel3());

    m_driverController.rightBumper().toggleOnTrue(m_coralIntake.setPivotPosition(2.4));
    m_driverController.leftBumper().toggleOnTrue(m_coralIntake.turntoNeutral());
    

  




    


















    //when b is pressed, coral intake pivots up
    //m_driverController.b().whileTrue(m_elevator.elevatorToLevel1());
    m_driverController.x().whileTrue(m_coralIntake.turntoNeutral());
   // m_driverController.rightTrigger().whileTrue(m_algaeIntake.intakeAlgae());
    m_driverController.leftTrigger().whileTrue(m_coralIntake.intakeCoral());
    m_driverController.rightTrigger().whileTrue(m_coralIntake.outtakeCoral());
    //when left dpad is pressed, algae ipivot goes down
    //m_driverController.leftBumper().whileTrue(m_algaeIntake.turntoZero());
    //when right dpad is pressed, algae pivot goes up
    //m_driverController.rightBumper().whileTrue(m_algaeIntake.turntoNeutral());
    
    // m_driverController.povUp().toggleOnTrue(m_elevator.elevatorToLevel3());
    // m_driverController.povDown().toggleOnTrue(m_elevator.resetElevatorPosition());
    // //m_driverController.povLeft().whileTrue(m_elevator.elevatorToLevel1());
    // m_driverController.povRight().toggleOnTrue(m_elevator.elevatorToLevel2());

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*/ Create config for trajectory


    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    */
    return null;
  }
  
  public void autonomousPeriodic() {
   // m_robotDrive.updateOdometry();
  }
  
  public void teleopInit() {
  }

  public void teleopPeriodic() {
    
    
  }

    
}