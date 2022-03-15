// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.OI;

public class TeleopFeeder extends CommandBase {

  private double feederVelocity = 192;

  Feeder feeder;
  /** Creates a new TeleopFeeder. */
  public TeleopFeeder(Feeder feeder_) {
    feeder = feeder_;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.operatorController.getXButton()) {
      feeder.setFeederVelocity(feederVelocity);
    } else {
      feeder.setFeederVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
