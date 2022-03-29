// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class LoadCommand extends CommandBase {

  Feeder feeder;
  Indexer indexer;
  Shooter shooter;

  private boolean prevTOF1closed = false;
  private boolean currTOF1closed = false;

  int cycles;

  boolean shouldCancel = false;

  /** Creates a new LoadCommand. */
  public LoadCommand(Indexer indexer, Feeder feeder, Shooter shooter) {
    this.indexer = indexer;
    this.feeder = feeder;
    this.shooter = shooter;
    addRequirements(indexer, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shouldCancel = shooter.getRange2() < Shooter.Constants.kTOF2_closed;
    currTOF1closed = (shooter.getRange1() < Shooter.Constants.kTOF1_closed);
    prevTOF1closed = currTOF1closed;
    indexer.setPower(0.8);
    feeder.setFeederVelocity(120);
    cycles = 0;
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
    feeder.setFeederVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shouldCancel) {return true;}
    cycles++;
    currTOF1closed = (shooter.getRange1() < Shooter.Constants.kTOF1_closed);
    if ((cycles>=10) && !currTOF1closed && prevTOF1closed) {return true;}
    prevTOF1closed = currTOF1closed;
    return false;
  }
}
