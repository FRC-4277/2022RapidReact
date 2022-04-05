// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class Vision extends SubsystemBase {
  private static final String PHOTON_CAMERA_NAME = "Driver Camera";
  private final PhotonCamera photonCamera = new PhotonCamera(PHOTON_CAMERA_NAME);
  private final HttpCamera httpCamera = new HttpCamera("Microsoft_LifeCam_HD-3000-output",
          "http://10.42.77.12:1182/stream.mjpg");
  private final ShuffleboardTab driverTab;

  /** Creates a new Vision. */
  public Vision(ShuffleboardTab driverTab) {
    this.driverTab = driverTab;

    // Set Photon to driver mode
    setDriverMode(true);

    // Add camera to main tab
    this.driverTab.add("Photon Vision", httpCamera)
    .withWidget(BuiltInWidgets.kCameraStream)
    .withPosition(0, 0)
    .withSize(5, 4);
  }

  public void setDriverMode(boolean driverMode) {
    photonCamera.setDriverMode(driverMode);
  }

  public void setPipeline() {
    DriverStation.Alliance alliance = DriverStation.getAlliance();
    if (alliance == DriverStation.Alliance.Red) {
      photonCamera.setPipelineIndex(0);
    } else {
      photonCamera.setPipelineIndex(1);
    }
  }

  public Optional<Double> getBallDegrees() {
    PhotonPipelineResult result = photonCamera.getLatestResult();
    if (result == null) {
      return Optional.empty();
    }
    PhotonTrackedTarget trackedTarget = result.getBestTarget();
    if (trackedTarget == null) {
      return Optional.empty();
    }
    double yaw = trackedTarget.getYaw();
    SmartDashboard.putNumber("Photon Yaw", yaw);
    return Optional.of(yaw);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
