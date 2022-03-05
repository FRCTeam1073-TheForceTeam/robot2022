// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterFeedCommand extends CommandBase {
  Shooter shooter;
  double startTime;
  double timeout;
  boolean prevSensor2;
  boolean currSensor2;
  boolean hasDetectedFallingEdge;

  /** Creates a new ShooterFeedCommand. */
  public ShooterFeedCommand(Shooter shooter_, double timeout_) {
    shooter = shooter_;
    timeout = timeout_;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currSensor2 = shooter.isBallInShooter();
    prevSensor2 = currSensor2;
    startTime = ((double) System.currentTimeMillis()) / 1000.0;
    shooter.setFeederVelocity(192);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFeederVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Check for falling edge on ToF sensor 2
    currSensor2 = shooter.isBallInShooter();
    boolean fallingEdge = (!currSensor2) && (prevSensor2);
    prevSensor2 = currSensor2;
    double currentTime = ((double) System.currentTimeMillis()) / 1000.0;
    System.out.println((currentTime - startTime));
    return ((currentTime - startTime) >= timeout);
  }
}
