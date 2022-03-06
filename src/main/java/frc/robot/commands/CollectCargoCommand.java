// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class CollectCargoCommand extends CommandBase {

  Collector collector;
  Indexer indexer;
  Shooter shooter;
  double collectorVelocity;

  /** Creates a new CollectCargoCommand. */
  public CollectCargoCommand(Collector collector_, Indexer indexer_, Shooter shooter_) {
    collector = collector_;
    indexer = indexer_;
    shooter = shooter_;
    addRequirements(collector, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setLiftPosition(Collector.Constants.loweredCollectorPosition);
    collector.setIntakeVelocity(collectorVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.setPower(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setLiftPosition(Collector.Constants.raisedCollectorPosition);
    collector.setIntakeVelocity(0);
    indexer.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.getRange1() < Shooter.Constants.kTOF1_closed);
  }
}
