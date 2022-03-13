// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * Lowers the collector and runs wheels indefinitely. Raises and stops wheels on end.
 * This command does not end itself, so please put in a timeout or in a ParallelDeadLineGroup
 * or it won't stop!
 */
public class CollectCommand extends CommandBase {
  Collector collector;
  double collectorVelocity = 12;

  /** Creates a new CollectCommand. */
  public CollectCommand(Collector collector_) {
    collector = collector_;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.setLiftPosition(Collector.Constants.loweredCollectorPosition);
    collector.setLinearIntakeVelocity(collectorVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setLiftPosition(Collector.Constants.raisedCollectorPosition);
    collector.setLinearIntakeVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
