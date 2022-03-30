// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class IndexCommand extends CommandBase {

  Indexer indexer;
  Shooter shooter;

  boolean risingEdge = false;
  boolean currentTOF0Closed = false;
  int numClosed = 0;
  boolean ballAlreadyInShooter = false;

  /** Creates a new CollectCargoCommand. */
  public IndexCommand(Indexer indexer_, Shooter shooter_) {
    indexer = indexer_;
    shooter = shooter_;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    numClosed = 0;
    currentTOF0Closed = false;
    risingEdge = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.setPower(0.45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ballAlreadyInShooter = (shooter.getRange2() < Shooter.Constants.kTOF2_closed);
    // if (ballAlreadyInShooter) {
      // currentTOF1Closed = (shooter.getRange0() < Shooter.Constants.kTOF1_closed_withTopBall);
    // } else {
    currentTOF0Closed = (shooter.getRange0() < Shooter.Constants.kTOF0_closed);
    // }
    if (currentTOF0Closed) {
      numClosed++;
    } else {
      numClosed = 0;
    }
    return (numClosed >= 4);
  }
}
