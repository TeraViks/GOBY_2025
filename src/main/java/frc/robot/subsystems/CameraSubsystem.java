// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class CameraSubsystem extends SubsystemBase {
  public static final class CameraConfig {
    public final String m_cameraName;
    public final Transform3d m_robotToCameraTransform;
    public final boolean m_enable;

    public CameraConfig(String cameraName, Transform3d robotToCameraTransform, boolean enable) {
      m_cameraName = cameraName;
      m_robotToCameraTransform = robotToCameraTransform;
      m_enable = enable;
    }
  }

  private static final class PoseEstimator {
    public final PhotonCamera m_camera;
    public final PhotonPoseEstimator m_photonPoseEstimator;
    private Optional<EstimatedRobotPose> m_estimate;

    public PoseEstimator(CameraConfig cameraConfig) {
      assert(cameraConfig.m_enable);
      m_camera = new PhotonCamera(cameraConfig.m_cameraName);
      m_photonPoseEstimator = new PhotonPoseEstimator(
        FieldConstants.kAprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraConfig.m_robotToCameraTransform);
      m_photonPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    }

    public void periodic() {
      for (PhotonPipelineResult pipelineResult : m_camera.getAllUnreadResults()) {
        Optional<EstimatedRobotPose> estimate = m_photonPoseEstimator.update(pipelineResult);
        // Filter out spurious empty results, such that the most recent non-empty result is
        // used as the estimate. PhotonVision may return multiple pipeline results (some
        // empty) per periodic update, and a result that's a few milliseconds old is better
        // than no result at all.
        if (estimate.isPresent()) {
          m_estimate = estimate;
        }
      }
    }

    public Optional<EstimatedRobotPose> estimatePose() {
      return m_estimate;
    }
  }

  private final List<PoseEstimator> m_poseEstimators;

  public CameraSubsystem(List<CameraConfig> cameraConfigs) {
    m_poseEstimators = cameraConfigs
      .stream()
      .filter(cameraConfig -> cameraConfig.m_enable)
      .map(cameraConfig -> new PoseEstimator(cameraConfig))
      .collect(Collectors.toList());
  }

  public List<Optional<EstimatedRobotPose>> getFieldRelativePoseEstimators() {
    return m_poseEstimators
      .stream()
      .map(poseEstimator -> poseEstimator.estimatePose())
      .collect(Collectors.toList());
  }

  @Override
  public void periodic() {
    for (PoseEstimator poseEstimator : m_poseEstimators) {
      poseEstimator.periodic();
    }
  }
}