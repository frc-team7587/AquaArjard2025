package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;

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

    public boolean hasTargets(){
        var result = camera.getLatestResult();
        return result.hasTargets();
    }

    public double getYaw(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            return target.getYaw();
        }
        return -1;
    }

    public double getPitch(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            return target.getPitch();
        }
        return -1;
    }

    public double getFiducialId(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            return target.getFiducialId();
        }
        return -1;
    }

    public Transform3d getCameraToTarget(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            return target.getBestCameraToTarget();
        }
        return null;
    }

    public double getPoseAmbiguity(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            return target.getPoseAmbiguity();
        }
        return -1;
    }

    public Transform3d getAlternateCameraToTarget() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            return target.getAlternateCameraToTarget();
        }
        return null;
    }

    public double getTimestampSeconds() {
        var result = camera.getLatestResult();
        return result.getTimestampSeconds();
    }

    public int getNumberOfTargets() {
        var result = camera.getLatestResult();
        return result.getTargets().size();
    }

    public List<PhotonTrackedTarget> getAllTargets() {
        var result = camera.getLatestResult();
        return result.getTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        var result = camera.getLatestResult();
        return result.getBestTarget();
    }
}
    