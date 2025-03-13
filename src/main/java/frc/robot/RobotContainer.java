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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeSparkMax;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberSparkMax;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralIntake.CoralIntakeSparkMax;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorModule;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.subsystems.Vision.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  //private final SwerveDrive m_robotDrive = new SwerveDrive();
  private final Elevator m_elevator = new Elevator(new ElevatorModule());
  private final CoralIntake m_coralIntake = new CoralIntake(new CoralIntakeSparkMax());
  private final AlgaeIntake m_algaeIntake = new AlgaeIntake(new AlgaeIntakeSparkMax());
  private final Climber m_climber = new Climber(new ClimberSparkMax());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // The operator's controller
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  // private final SendableChooser<Command> autoChooser;
  private final Field2d field;

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

  

    //Registering named commands to pathplanner
    NamedCommands.registerCommand("Level 0", m_elevator.elevatorToLevel0());
    NamedCommands.registerCommand("Level 1", m_elevator.elevatorToLevel1());
    NamedCommands.registerCommand("Level 2", m_elevator.elevatorToLevel2());
    NamedCommands.registerCommand("Level 3", m_elevator.elevatorToLevel3());

    NamedCommands.registerCommand("Shoot Coral", m_coralIntake.outtakeCoral());
    NamedCommands.registerCommand("Intake Coral", m_coralIntake.intakeCoral());

    NamedCommands.registerCommand("Turn Coral to Neutral", m_coralIntake.turntoNeutral());
    NamedCommands.registerCommand("Turn Coral to Up", m_coralIntake.turntoUp());
    NamedCommands.registerCommand("Turn Coral to Down", m_coralIntake.turntoDown());

    field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
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

    //sequantial command group for level 0 sco(ring, scores the corala and then brings elevator back to 0
    SequentialCommandGroup L0 = new SequentialCommandGroup(
      m_elevator.elevatorToLevel0().alongWith(m_coralIntake.turntoNeutral()).withTimeout(1)
      // m_coralIntake.outtakeCoral().withTimeout(1.5),
      // m_elevator.resetElevatorPosition()
    );

    //sequantial command group for level 1 scoring, scores the corala and then brings elevator back to 0
    SequentialCommandGroup L1 = new SequentialCommandGroup(
      m_elevator.elevatorToLevel1().alongWith(m_coralIntake.turntoNeutral()).withTimeout(1)
      // m_coralIntake.outtakeCoral().withTimeout(1.5),
      // m_elevator.resetElevatorPosition()
    );

    //sequantial command group for level 2 scoring, scores the corala and then brings elevator back to 0
    SequentialCommandGroup L2 = new SequentialCommandGroup(
      m_elevator.elevatorToLevel2().alongWith(m_coralIntake.turntoNeutral()).withTimeout(1)
      // m_coralIntake.outtakeCoral().withTimeout(1.5),
      // m_elevator.resetElevatorPosition()
    );

    //sequantial command group for level 3 scoring, scores the corala and then brings elevator back to 0
    SequentialCommandGroup L3 = new SequentialCommandGroup(
      m_elevator.elevatorToLevel3().alongWith(m_coralIntake.setPivotPosition(2)).withTimeout(1)
      // m_coralIntake.outtakeCoral().withTimeout(1.5),
      // m_elevator.resetElevatorPosition()
    );

  

    //OPERATOR CONTROLS

    //when bottom on Dpad is pressed, the level 0 sequence is run
    m_operatorController.povDown().onTrue(L0);

    //when left on Dpad is pressed, the level 1 sequence is run
    m_operatorController.povLeft().onTrue(L1);

    //when right on Dpad is pressed, the level 2 sequence is run
    m_operatorController.povRight().onTrue(L2);

    //when top on Dpad is pressed, the level 3 sequence is run
    m_operatorController.povUp().onTrue(L3);
    
    //when the Y button is held down, sets the robot into a position to intake the coral
    m_operatorController.y().onTrue(m_elevator.setElevatorPosition(4.1).alongWith(m_coralIntake.setPivotPosition(4.8).withTimeout(1)));

    //when the A button is held down, the elevator is set to level 0 and the coral intake is set to pivot position 0
    m_operatorController.a().onTrue(m_elevator.setElevatorPosition(ElevatorConstants.kElevatorLevel2+0.5).alongWith(m_coralIntake.turntoNeutral()).alongWith(m_algaeIntake.turntoNeutral()));

    //when the left bumper is held down, the algae intake motor spins to intake the algae
    m_operatorController.leftBumper().whileTrue(m_algaeIntake.intakeAlgae());

    //when the right bumper is held down, the algae intake motor spins to outtake the algae
    m_operatorController.rightBumper().whileTrue(m_algaeIntake.outtakeAlgae());

    //when the left trigger is held down, the coral intake motor spins to intake the coral
    m_operatorController.leftTrigger().whileTrue(m_coralIntake.intakeCoral());

    //when the right trigger is held down, the coral intake motor spins to outtake the coral
    m_operatorController.rightTrigger().whileTrue(m_coralIntake.outtakeCoral());

    //when the start button (button with the 3 lines) is held down, the coral pivot motor spins to move the algae pivot downwards
    m_operatorController.start().and(m_operatorController.b()).whileTrue(m_algaeIntake.turntoDown());

    //when the back button (button with the 3 lines) is held down, the coral pivot motor spins to move the algae pivot upwards
    m_operatorController.start().and(m_operatorController.x()).whileTrue(m_algaeIntake.turntoUp());

    //when the start button (button with the 3 lines) is held down, the coral pivot motor spins to move the coral pivot downwards
    m_operatorController.start().and(m_operatorController.a()).whileTrue(m_coralIntake.turntoDown());

    //when the back button (button with the 3 lines) is held down, the coral pivot motor spins to move the coral pivot upwards
    m_operatorController.start().and(m_operatorController.y()).whileTrue(m_coralIntake.turntoUp());

    //when the b button is pressed, the climber motor spins to make the robot climb on the chain
    m_operatorController.b().onTrue(m_climber.hangOnChain());

    //when the x button is pressed, the climber motor spins to make the robot get off the chain
    m_operatorController.x().onTrue(m_climber.getOffChain());
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

  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    try{
    PathPlannerPath path = PathPlannerPath.fromPathFile("leave 0");

    return AutoBuilder.followPath(path);
    } catch (Exception e){
      DriverStation.reportError("error", null);
      return Commands.none();
    }
    //sendable chooser
    

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

    

  }
  
  public void autonomousPeriodic() {
   // m_robotDrive.updateOdometry();
  }
  
  public void teleopInit() {
  }

  public void teleopPeriodic() {
    
    
  }

    
}