// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DashboardReadoutCommand extends CommandBase {

  private static int counter = 0;

  public static void resetCounter() {
    counter = 0;
    SmartDashboard.putString("AutoReadout/CurrentCommand", "(Reset)");
  }

  String readout;

  /** Creates a new DashboardReadoutCommand. */
  public DashboardReadoutCommand(String readout) {
    this.readout = readout;
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putString("AutoReadout/CurrentCommand", String.format("[Command #%d]: %s", counter, readout));
    counter++;
    return true;
  }
}
