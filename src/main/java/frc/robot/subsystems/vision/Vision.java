package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.*;
import edu.wpi.first.math.util.Units;

public class Vision extends SubsystemBase {
    PhotonCamera camera;
    public Vision(){
        camera = new PhotonCamera("photonvision");

    }

    @Override
    public void periodic(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            SmartDashboard.putNumber("targetArea", target.getArea());
        }
    }

    public double getTargetArea(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            return target.getArea();
        }
        return -1;
    }
    
    public double getDistanceToTarget(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.kCameraHeight, VisionConstants.kTargetHeight,
            Units.degreesToRadians(VisionConstants.kCameraPitch),
            Units.degreesToRadians(target.getPitch()));
            return distance;
        }
        return -1;
    }

    
}