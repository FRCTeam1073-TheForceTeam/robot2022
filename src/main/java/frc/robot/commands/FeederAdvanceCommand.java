// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class FeederAdvanceCommand extends CommandBase {

  Feeder feeder;
  double startPosition;
  double targetRotation;
  double velocity;

  public FeederAdvanceCommand(Feeder feeder_, double targetRotation_) {
    this(feeder_, targetRotation_, 400);
  }

  /** Creates a new FeederAdvanceCommand. */
  public FeederAdvanceCommand(Feeder feeder_, double targetRotation_, double velocity_) {
    feeder = feeder_;
    targetRotation = targetRotation_;
    velocity = velocity_;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPosition = feeder.getFeederPosition();
    feeder.setFeederVelocity(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.zeroFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(feeder.getFeederPosition() - startPosition) > Math.abs(targetRotation);
  }
}
