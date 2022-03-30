// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class FeederLaunchCommand extends CommandBase {
  Indexer indexer;
  Shooter shooter;
  Feeder feeder;

  double feederVelocity = 192;

  Timer timer;

  int numLoops = 0;
  boolean ballInIndexer = true;

  /** Creates a new FeedCommand. */
  public FeederLaunchCommand(Indexer indexer_, Feeder feeder_, Shooter shooter_) {
    indexer = indexer_;
    feeder = feeder_;
    shooter = shooter_;
    timer = new Timer();
    addRequirements(indexer, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    feeder.setFeederVelocity(192);
    if (shooter.getRange0() < Shooter.Constants.kTOF0_closed) {
      ballInIndexer = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ballInIndexer) {
      indexer.setPower(0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setFeederVelocity(0);
    if (ballInIndexer) {
      indexer.setPower(0.8);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.3);
  }
}
