// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static final String PHOTON_CAMERA_NAME = "Driver Camera";
  private PhotonCamera photonCamera = new PhotonCamera(PHOTON_CAMERA_NAME);
  //private HttpCamera httpCamera = new HttpCamera("Photon", "http://10.42.77.12:1181");
  private ShuffleboardTab driverTab;

  /** Creates a new Vision. */
  public Vision(ShuffleboardTab driverTab) {
    this.driverTab = driverTab;
    photonCamera.setDriverMode(true);
    //this.driverTab.add("Driver Camera", httpCamera)
    this.driverTab.addCamera("Driver Camera", "Driver Camera", "http://10.42.77.12:1181")
    .withPosition(0, 0)
    .withSize(6, 6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
