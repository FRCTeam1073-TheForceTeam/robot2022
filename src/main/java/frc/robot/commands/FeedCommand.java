// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class FeedCommand extends CommandBase {

  Shooter shooter;

  double feederVelocity = 192;

  boolean startTOF1Closed = false;
  boolean startTOF2Closed = false;
  boolean currentTOF1Closed = false;
  boolean currentTOF2Closed = false;

  /** Creates a new FeedCommand. */
  public FeedCommand(Shooter shooter_) {
    shooter = shooter_;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTOF1Closed = (shooter.getRange1() < Shooter.Constants.kTOF1_closed);
    startTOF2Closed = (shooter.getRange2() < Shooter.Constants.kTOF2_closed);
    if (startTOF1Closed && !startTOF2Closed) {
      shooter.setFeederVelocity(feederVelocity);      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFeederVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentTOF1Closed = (shooter.getRange1() < Shooter.Constants.kTOF1_closed);
    currentTOF2Closed = (shooter.getRange2() < Shooter.Constants.kTOF2_closed);
    // If the shooter *didn't* start with the bottom sensor closed and the top sensor open, end instantly.
    // Otherwise, end only when the bottom sensor is open and the top sensor is closed.
    // TODO: Is !currentTOF1Closed necessary, or might it break things?
    return !(startTOF1Closed && !startTOF2Closed) || (!currentTOF1Closed && currentTOF2Closed);
  }
}
