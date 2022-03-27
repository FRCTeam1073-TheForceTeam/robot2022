// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Shooter;

public class WaitForFallingEdge extends CommandBase {

  // public enum SensorIndex {
  //   TOF1,TOF2
  // }

  Shooter shooter;
  int index;

  boolean currValue = false;
  boolean prevValue = false;

  /** Creates a new WaitForFallingEdge.
   * @param index (1 or 2 only)
  */
  public WaitForFallingEdge(Shooter shooter, int index) {
    this.shooter = shooter;
    this.index = index;
  }

  @Override
  public void initialize() {
    currValue = getValue();
    prevValue = currValue;
  }

  public boolean getValue() {
    // if (index == 1) {
    //   return shooter.getRange1() < Shooter.Constants.kTOF1_closed;
    // }else if (index == 2) {
      return shooter.getRange2() < Shooter.Constants.kTOF2_closed;
    // } else {
    //   return false;
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currValue = getValue();
    if(!currValue && prevValue){return true;}
    prevValue = currValue;
    return false;
  }
}
