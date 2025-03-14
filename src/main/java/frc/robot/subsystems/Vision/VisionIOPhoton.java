package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.Vision.VisionConstants.camNames;
import static frc.robot.subsystems.Vision.VisionConstants.camsRobotToCam;
import static frc.robot.subsystems.Vision.VisionConstants.kTagLayout;
import static frc.robot.subsystems.Vision.VisionConstants.numCameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera[] cameras = new PhotonCamera[numCameras];
  private final PhotonPoseEstimator[] cameraEstimators = new PhotonPoseEstimator[numCameras];
  private final PhotonPipelineResult[] cameraResults = new PhotonPipelineResult[numCameras];

  private Pose2d lastEstimate = new Pose2d();

  LoggedNetworkBoolean killSideCams =
      new LoggedNetworkBoolean("/SmartDashboard/Vision/KillSideCams", false);

  public VisionIOPhoton() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    for (int i = 0; i < numCameras; i++) {
      cameras[i] = new PhotonCamera(camNames[i]);
      cameraEstimators[i] =
          new PhotonPoseEstimator(
              kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camsRobotToCam[i]);
      cameraEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      cameraResults[i] = new PhotonPipelineResult();
    }

    SmartDashboard.putBoolean("KillSideCams", false);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate, Rotation2d heading) {
    lastEstimate = currentEstimate;

    PhotonPipelineResult[] results = getAprilTagResults();
    PhotonPoseEstimator[] photonEstimators = getAprilTagEstimators(currentEstimate, heading);

    inputs.estimate = new Pose2d[] {new Pose2d()};

    // add code to check if the closest target is in front or back
    inputs.timestamp = estimateLatestTimestamp(results);

    inputs.timestampArray = getTimestampArray(results);

    if (hasEstimate(results)) {
      // inputs.results = results;
      inputs.estimate = getEstimatesArray(results, photonEstimators);
      inputs.hasEstimate = true;

      inputs.cameraTargets = getCameraTargets(results);

      Pose3d[] tags = getTargetsPositions(results);
      Logger.recordOutput("Vision/Targets3D", tags);
      Logger.recordOutput("Vision/Targets", Pose3dToPose2d(tags));
      Logger.recordOutput("Vision/TagCounts", tagCounts(results));
    } else {
      inputs.timestamp = inputs.timestamp;
      inputs.hasEstimate = false;
    }

    // Log if the robot code can see these cameras
    for (int i = 0; i < numCameras; i++) {
      Logger.recordOutput("Vision/cam" + (i + 1) + "/Connected", cameras[i].isConnected());
    }
  }

  @Override
  public PhotonPipelineResult getLatestResult(int camIndex) {
    if (camIndex < 0 || camIndex >= numCameras) return new PhotonPipelineResult();

    var unreadResults = cameras[camIndex].getAllUnreadResults();
    double latestTimestamp = 0;

    if (unreadResults.size() == 0) {
      return cameraResults[camIndex];
    }

    for (var result : unreadResults) {
      if (result.getTimestampSeconds() > latestTimestamp) {
        latestTimestamp = result.getTimestampSeconds();
        cameraResults[camIndex] = result;
      }
    }

    return cameraResults[camIndex];
  }

  private PhotonPipelineResult[] getAprilTagResults() {
    if (killSideCams.get()) {
      PhotonPipelineResult cam1_result = getLatestResult(0);

      printStuff("cam1", cam1_result);

      return new PhotonPipelineResult[] {cam1_result};
    }

    PhotonPipelineResult[] results = new PhotonPipelineResult[numCameras];

    for (int i = 0; i < numCameras; i++) {
      results[i] = getLatestResult(i);
      printStuff("cam" + (i + 1), results[i]);
    }

    return results;
  }

  private void printStuff(String name, PhotonPipelineResult result) {
    Logger.recordOutput("Vision/" + name + "/results", result.getTargets().size());

    PhotonTrackedTarget target = result.getBestTarget();
    if (target != null) {
      Logger.recordOutput(
          "Vision/" + name + "/PoseAmbiguity", result.getBestTarget().getPoseAmbiguity());
      Logger.recordOutput("Vision/" + name + "/Yaw", result.getBestTarget().getYaw());
    }
  }

  private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate, Rotation2d heading) {
    if (killSideCams.get()) {
      cameraEstimators[0].setReferencePose(currentEstimate);
      cameraEstimators[0].addHeadingData(Timer.getFPGATimestamp(), heading);

      return new PhotonPoseEstimator[] {cameraEstimators[0]};
    }

    for (PhotonPoseEstimator estimator : cameraEstimators) {
      estimator.setReferencePose(currentEstimate);
      estimator.addHeadingData(Timer.getFPGATimestamp(), heading);
    }

    return cameraEstimators;
  }

  @Override
  public boolean goodResult(PhotonPipelineResult result) {
    return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD
    /*
     * && kTagLayout.
     * getTagPose(
     * result.
     * getBestTarget().
     * getFiducialId())
     * .get().toPose2d(
     * ).getTranslation
     * ()
     * .getDistance(
     * lastEstimate.
     * getTranslation()
     * ) < MAX_DISTANCE
     */ ;
  }
}
