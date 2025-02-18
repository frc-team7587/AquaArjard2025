package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Vision extends SubsystemBase {
    SwerveDrive m_swerve = new SwerveDrive();

    String limelightName;

    public Vision(String limelightName) {
        this.limelightName = limelightName;
    }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight. This will make the robot turn to face the target.
    public double limelight_aim_proportional() {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX(limelightName) * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= SwerveConstants.kMaxAngularSpeed;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    public double limelight_range_proportional() {    
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY(limelightName) * kP;
        targetingForwardSpeed *= SwerveConstants.kMaxSpeedMetersPerSecond;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    public double calculateDistanceToTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0; 

        // distance from the target to the floor
        double goalHeightInches = 60.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }
}