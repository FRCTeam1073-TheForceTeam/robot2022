// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;

public class TeleopClimber extends CommandBase {
  Climber climber = new Climber();

  /** Creates a new TeleopClimber. */
  public TeleopClimber(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // climber.setSpoolVelocity(OI.operatorController.getLeftY());
    // climber.setExtensionVelocity(OI.operatorController.getRightY());

    climber.setSpoolPower(OI.operatorController.getLeftY() * .25);
    climber.setExtensionPower(OI.operatorController.getRightY() * .25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpoolVelocity(0);
    climber.setExtensionVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}