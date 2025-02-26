package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.CoralIntake.CoralIntakeConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
    public static final class ElevatorConfig {
        public static final SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        public static final SoftLimitConfig elevatorSoftLimit = new SoftLimitConfig();

        static {
            leftMotorConfig
                    //.smartCurrentLimit(0);
                    .idleMode(IdleMode.kBrake)
                    .voltageCompensation(12.0)
                    .inverted(false);
            leftMotorConfig.closedLoop.pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 0);
            leftMotorConfig.closedLoop.maxMotion
                    .maxVelocity(ElevatorConstants.kMaxVelocity)
                    .maxAcceleration(ElevatorConstants.kMaxAcceleration);
            rightMotorConfig
                    .apply(leftMotorConfig)
                    .follow(ElevatorConstants.kElevatorLeftMotorID, false);
        
        }  
    }
    public static final class AlgaeIntakeConfig{
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
        public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();
        public static final SoftLimitConfig pivotSoftLimit = new SoftLimitConfig();

        static{
        intakeConfig
                .idleMode(IdleMode.kBrake);
        pivotConfig
                .idleMode(IdleMode.kBrake);
        pivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(AlgaeIntakeConstants.kP, AlgaeIntakeConstants.kI, AlgaeIntakeConstants.kD)
                .outputRange(AlgaeIntakeConstants.kMinOutput, AlgaeIntakeConstants.kMaxOutput);
        /*pivotSoftLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit((float)AlgaeIntakeConstants.kPivotMaxPosition)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit((float)AlgaeIntakeConstants.kPivotMinPosition);
        */
        }
    }
    public static final class CoralIntakeConfig{
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
        public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();
        public static final SoftLimitConfig pivotSoftLimit = new SoftLimitConfig();

        static{
        pivotConfig
                .idleMode(IdleMode.kBrake);
        pivotConfig.closedLoop
                .pid(CoralIntakeConstants.kP, CoralIntakeConstants.kI, CoralIntakeConstants.kD)
                .velocityFF(CoralIntakeConstants.kFF)
                .outputRange(CoralIntakeConstants.kMinOutput, CoralIntakeConstants.kMaxOutput);
        pivotSoftLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit((float)CoralIntakeConstants.kPivotMaxPosition)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit((float)CoralIntakeConstants.kPivotMinPosition);
        intakeConfig
                .idleMode(IdleMode.kBrake);
        }
    }
}