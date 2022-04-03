// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class HoodIndexCommand extends CommandBase {
  Shooter shooter;

  boolean prevTOFIndexClosed;
  boolean currTOFIndexClosed;
  boolean shouldCancel;
  double hoodPosition;

  /** Creates a new HoodIndexCommand. */
  public HoodIndexCommand(Shooter shooter_) {
    shooter = shooter_;
    SmartDashboard.putNumber("HoodIndexing/End position", 0);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shouldCancel = false;
    currTOFIndexClosed = (shooter.getHoodIndexRange() < Shooter.Constants.kTOFIndex_closed);
    if (currTOFIndexClosed) {
      shooter.setHoodPositionWithVelocity(0.25, 0.1);
      prevTOFIndexClosed = currTOFIndexClosed;
    } else {
      shouldCancel = true;
    }
  }

  @Override
  public void execute() {}
  
  @Override
  public void end(boolean interrupted) {
    if (!interrupted && !shouldCancel) {
      SmartDashboard.putNumber("HoodIndexing/End position", hoodPosition);
      Shooter.setAdditionalHoodAngle(hoodPosition - Shooter.Constants.nominalHoodZeroPosition);
      shooter.zeroHood();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currTOFIndexClosed = (shooter.getHoodIndexRange() < Shooter.Constants.kTOFIndex_closed);
    hoodPosition = shooter.getHoodPosition();
    if (shouldCancel || (!currTOFIndexClosed && prevTOFIndexClosed)) {
      return true;
    }
    prevTOFIndexClosed = currTOFIndexClosed;
    return false;
  }
}
