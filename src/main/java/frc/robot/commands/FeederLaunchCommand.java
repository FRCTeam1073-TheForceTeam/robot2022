// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class FeederLaunchCommand extends CommandBase {
  Shooter shooter;
  Feeder feeder;

  double feederVelocity = 192;

  int initialClosedCount = 0;

  boolean prevTOF2Closed = false;
  boolean currentTOF2Closed = false;

  int numLoops = 0;

  /** Creates a new FeedCommand. */
  public FeederLaunchCommand(Feeder feeder_, Shooter shooter_) {
    feeder = feeder_;
    shooter = shooter_;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialClosedCount = 0;
    feeder.setFeederVelocity(feederVelocity);
    prevTOF2Closed = false;
    currentTOF2Closed = false;
    numLoops = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentTOF2Closed = (shooter.getRange2() < Shooter.Constants.kTOF2_closed);
    if (!currentTOF2Closed || numLoops != 0) {
      numLoops++;
    }
    return !currentTOF2Closed && (numLoops > 10);
    // if ((!currentTOF2Closed) && prevTOF2Closed) {
    //   return true;
    // }
    // prevTOF2Closed = currentTOF2Closed;
    // return false;
  }
}
