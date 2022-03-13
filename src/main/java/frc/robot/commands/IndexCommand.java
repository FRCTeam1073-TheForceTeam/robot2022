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
  boolean currentTOF1Closed = false;
  int numClosed = 0;

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
    currentTOF1Closed = false;
    risingEdge = false;
    for (int i = 0; i < 4; i++) {
      System.out.println(System.currentTimeMillis()+"IDX_START");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.setPower(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setPower(0);
    for (int i = 0; i < 10; i++) {
      System.out.println(System.currentTimeMillis()+"IDX_STOP");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentTOF1Closed = (shooter.getRange1() < Shooter.Constants.kTOF1_closed);
    if (currentTOF1Closed) {
      numClosed++;
    } else {
      numClosed = 0;
    }
    System.out.println("\t\tC"+numClosed);
    return (numClosed >= 4);
  }
}
