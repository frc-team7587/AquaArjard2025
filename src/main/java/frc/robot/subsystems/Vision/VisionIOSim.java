package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.Vision.VisionConstants.camNames;
import static frc.robot.subsystems.Vision.VisionConstants.camsRobotToCam;
import static frc.robot.subsystems.Vision.VisionConstants.kTagLayout;
import static frc.robot.subsystems.Vision.VisionConstants.numCameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOSim implements VisionIO {
  private final PhotonCamera[] cameras = new PhotonCamera[numCameras];
  private final PhotonPoseEstimator[] camEstimators = new PhotonPoseEstimator[numCameras];
  private PhotonCameraSim[] camSims = new PhotonCameraSim[numCameras];
  private PhotonPipelineResult[] cameraResults = new PhotonPipelineResult[numCameras];

  private VisionSystemSim visionSim;

  private Pose2d lastEstimate = new Pose2d();

  public VisionIOSim() {
    for (int i = 0; i < numCameras; i++) {
      cameras[i] = new PhotonCamera(camNames[i]);
      camEstimators[i] =
          new PhotonPoseEstimator(
              kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camsRobotToCam[i]);
      camEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
      cameraResults[i] = new PhotonPipelineResult();
    }

    // Create the vision system simulation which handles cam1s and targets on the
    // field.
    visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this
    // simulated field.
    visionSim.addAprilTags(kTagLayout);
    // Create simulated cam1 properties. These can be set to mimic your actual
    // cam1.

    SimCameraProperties[] camProps = new SimCameraProperties[numCameras];

    for (int i = 0; i < 4; i++) {
      camProps[i] = new SimCameraProperties();
      camProps[i].setCalibError(0.35, 0.10);
      camProps[i].setAvgLatencyMs(50);
      camProps[i].setLatencyStdDevMs(15);
    }

    camProps[0].setCalibration(480, 320, Rotation2d.fromDegrees(70));
    camProps[0].setFPS(40);
    camProps[1].setCalibration(480, 320, Rotation2d.fromDegrees(70));
    camProps[1].setFPS(20);
    camProps[2].setCalibration(480, 320, Rotation2d.fromDegrees(70));
    camProps[2].setFPS(20);
    camProps[3].setCalibration(480, 320, Rotation2d.fromDegrees(70));
    camProps[3].setFPS(40);

    // Create a PhotonCameraSim which will update the linked PhotonCamera's values
    // with visible
    // targets.
    for (int i = 0; i < numCameras; i++) {
      camSims[i] = new PhotonCameraSim(cameras[i], camProps[i]);
      visionSim.addCamera(camSims[i], camsRobotToCam[i]);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate, Pose2d odometry) {
    lastEstimate = currentEstimate;
    visionSim.update(odometry);

    for (PhotonPoseEstimator estimator : camEstimators) {
      estimator.setReferencePose(currentEstimate);
      estimator.addHeadingData(Timer.getFPGATimestamp(), odometry.getRotation());
    }

    PhotonPipelineResult[] results = new PhotonPipelineResult[numCameras];

    for (int i = 0; i < numCameras; i++) {
      results[i] = getLatestResult(i);
    }

    inputs.estimate = new Pose2d[] {new Pose2d()};

    // add code to check if the closest target is in front or back
    inputs.timestamp = estimateLatestTimestamp(results);

    inputs.timestampArray = getTimestampArray(results);

    if (hasEstimate(results)) {
      inputs.estimate = getEstimatesArray(results, camEstimators);
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

    Logger.recordOutput("Vision/OrangeConnected", cameras[0].isConnected());
    Logger.recordOutput("Vision/RaspberryConnected", cameras[1].isConnected());
    Logger.recordOutput("Vision/Raspberry2Connected", cameras[2].isConnected());
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

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    return visionSim.getDebugField();
  }

  @Override
  public boolean goodResult(PhotonPipelineResult result) {
    return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD;
    // && kTagLayout
    //         .getTagPose(result.getBestTarget().getFiducialId())
    //         .get()
    //         .toPose2d()
    //         .getTranslation()
    //         .getDistance(lastEstimate.getTranslation())
    //     < MAX_DISTANCE;
  }
}
