// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitToLevel extends CommandBase {
  Shooter shooter;
  double startTime;
  double timeout;
  /** Creates a new WaitToLevel. */
  public WaitToLevel(Shooter shooter_, double timeout_) {
    shooter = shooter_;
    timeout = timeout_;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = ((double) System.currentTimeMillis()) * 1e-3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.setFlywheelVelocity(0);
      shooter.setHoodPosition(0);
      shooter.setFeederVelocity(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((((double) System.currentTimeMillis()) * 1e-3) - startTime >= timeout);
  }
}
