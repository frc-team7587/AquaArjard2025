// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightHelpers;

public class SwerveDrive extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
    SwerveConstants.kFrontLeftDrivingCanId,
    SwerveConstants.kFrontLeftTurningCanId,
    SwerveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    SwerveConstants.kFrontRightDrivingCanId,
    SwerveConstants.kFrontRightTurningCanId,
    SwerveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
    SwerveConstants.kRearLeftDrivingCanId,
    SwerveConstants.kRearLeftTurningCanId,
    SwerveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
    SwerveConstants.kRearRightDrivingCanId,
    SwerveConstants.kRearRightTurningCanId,
    SwerveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    SwerveConstants.kDriveKinematics,
      getRotation(),
      getModulePositions());

  private Field2d field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public SwerveDrive() {
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );

      // Set up custom logging to add the current path to a field 2d widget
      PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

      SmartDashboard.putData("Field", field);

      // Usage reporting for MAXSwerve template
      HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

      m_gyro.reset();
    } catch (IOException | ParseException e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(getRotation(),getModulePositions());

        SmartDashboard.putNumber("Gyro angle: ", m_gyro.getAngle()%360);
        
       
  }
  /**
     * Returns the rotation of the robot reported by the gyroscope.
     * @return The rotation of the robot.
     */
  public Rotation2d getRotation() {
    return m_gyro.getRotation2d();
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          SwerveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  /**
     * Returns the positions of the swerve modules.
     * @return The swerve module positions.
     */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getRotation(),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
      SwerveConstants.kDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.discretize(
              fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                  : new ChassisSpeeds(xSpeed, ySpeed, rot),
              periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds discetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] moduleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(discetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(moduleStates[0]);
    m_frontRight.setDesiredState(moduleStates[1]);
    m_rearLeft.setDesiredState(moduleStates[2]);
    m_rearRight.setDesiredState(moduleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return (getRotation().getDegrees()) % 360;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });


    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }
}