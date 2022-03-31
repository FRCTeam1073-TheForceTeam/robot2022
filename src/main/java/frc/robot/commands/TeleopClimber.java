// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopClimber extends CommandBase {
  Climber climber;

  private double spoolMultiplier = 11.0;
  private double extensionMultiplier = 1.5;

  /** Creates a new TeleopClimber. */
  public TeleopClimber(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // climber.setSpoolVelocity(OI.operatorController.getLeftY());
    // climber.setExtensionVelocity(OI.operatorController.getRightY());

    double spoolVel = OI.operatorController.getLeftY() * spoolMultiplier;
    double extensionVel = OI.operatorController.getRightY() * extensionMultiplier;
    if (climber.getSpoolPosition() >= 0) {
      spoolVel = Math.min(0, spoolVel);
    }
   else if (climber.getSpoolPosition() <= Climber.Constants.maxSpoolDistance) {
      spoolVel = Math.max(0, spoolVel);
    }
    climber.setSpoolVelocity(spoolVel);

    if (OI.operatorController.getRightTriggerAxis() > 0.5) {
      if (climber.getExtensionMode()) {
        climber.setExtensionBrake(false);
      }
      climber.setExtensionPower(0);
    } 
    else {
      if (!climber.getExtensionMode()) {
        climber.setExtensionBrake(true);
      }
      if (climber.getExtensionPosition() >= 0) {
        extensionVel = Math.min(0, extensionVel);
      }
      else if (climber.getExtensionPosition() <= -2.1){
        extensionVel = Math.max(0, extensionVel);
      }
      climber.setExtensionVelocity(extensionVel);
    }
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