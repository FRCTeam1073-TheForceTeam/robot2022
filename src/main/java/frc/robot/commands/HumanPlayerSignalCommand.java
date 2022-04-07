// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Bling;

public class HumanPlayerSignalCommand extends CommandBase {

  boolean blinkCounter = false;
  Bling bling;

  /** Creates a new HumanPlayerSignalCommand. */
  public HumanPlayerSignalCommand(Bling bling_) {
    bling = bling_;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blinkCounter = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // blinkCounter = !blinkCounter;
    // if (blinkCounter) {
    bling.setSlot(5, 255, 53, 104);
    for (int i = 16; i <= 21; i++) {
      bling.setSlot(i, 255, 53, 104);      
    }
    // } else {
    //   bling.setSlot(6, 0, 0, 0);
    //   for (int i = 16; i <= 21; i++) {
    //     bling.setSlot(i, 0, 0, 0);      
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bling.setSlot(5, 0, 0, 0);
    for (int i = 16; i <= 22; i++) {
      bling.setSlot(i, 0, 0, 0);      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
