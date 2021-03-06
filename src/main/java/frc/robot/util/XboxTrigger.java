/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class XboxTrigger extends Trigger {
  private static final double DEFAULT_THRESHOLD = 0.05;
  private XboxController controller;
  private boolean left;
  private double threshold;

  public XboxTrigger(XboxController controller, boolean left) {
    this(controller, left, DEFAULT_THRESHOLD);
  }

  public XboxTrigger(XboxController controller, boolean left, double threshold) {
    this.controller = controller;
    this.left = left;
    this.threshold = threshold;
  }
  
  @Override
  public boolean get() {
      return (left ? controller.getLeftTriggerAxis() : controller.getRightTriggerAxis()) >= threshold;
  }
}