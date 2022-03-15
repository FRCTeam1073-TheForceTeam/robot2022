// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Shooter;

public class TeleopIndexer extends CommandBase {

  private Indexer indexer;
  private Shooter shooter;

  /** Creates a new TeleopIndexer. */
  public TeleopIndexer(Indexer indexer, Shooter shooter) {
    this.indexer = indexer;
    this.shooter = shooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setPower(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Indexer runs inward if the left bumper's held and outward if the right bumper's held.
    //If the left trigger is past 50%, it goes backwards regardless of other inputs.
    //Otherwise, it sets power to zero.
    if (OI.operatorController.getLeftTriggerAxis() > 0.5) {
      indexer.setPower(-0.8);
    }else if (OI.operatorController.getLeftBumper()) {
      indexer.setPower(0.8);
    }else{
      indexer.setPower(0);
    }

    
    if (indexer.isStalled()) {
      // Robot.getBling().setSlot(2, 255, 0, 0);
    } else {
      // Robot.getBling().setSlot(2, 0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}