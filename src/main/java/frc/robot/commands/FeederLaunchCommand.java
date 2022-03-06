// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class FeederLaunchCommand extends CommandBase {
  Shooter shooter;

  double feederVelocity = 192;

  boolean startTOF2Closed = false;
  boolean currentTOF2Open = false;

  /** Creates a new FeedCommand. */
  public FeederLaunchCommand(Shooter shooter_) {
    shooter = shooter_;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTOF2Closed = (shooter.getRange2() < Shooter.Constants.kTOF2_closed);
    if (startTOF2Closed) {
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
    currentTOF2Open = (shooter.getRange2() > Shooter.Constants.kTOF2_open);
    return (!startTOF2Closed) || (currentTOF2Open);
  }
}
