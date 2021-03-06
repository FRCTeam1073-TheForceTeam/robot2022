// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.OI;

public class TeleopHubTracking extends CommandBase {
  private HubTracking hubTracker;
  /** Creates a new TeleopHubTracker. */
  public TeleopHubTracking(HubTracking hubTracker) {
    this.hubTracker = hubTracker;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hubTracker);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Update hubVisible
    // hubTracker.setLEDIntensity(OI.driverController.getLeftTriggerAxis());
    hubTracker.setLEDIntensity(hubTracker.ledPower, hubTracker.ledPower, hubTracker.ledPower);
    // hubTracker.setLEDIntensity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hubTracker.setLEDIntensity(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
