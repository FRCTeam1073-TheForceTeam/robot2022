// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IMU;

public class ClimbAcrossRungs extends CommandBase {

  private Climber climber;

  /** Creates a new ClimbToMediumRung. */
  public ClimbAcrossRungs(Climber climber) {
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
    //Check to see if it's already hanging and hang is stable
    //Release spool halfway
    //Extend vertically
    //Release spool the rest of the way
    //Extend until you hit the next bar
    //Wind spool (pulling robot onto new bar and off of previous bar)
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
