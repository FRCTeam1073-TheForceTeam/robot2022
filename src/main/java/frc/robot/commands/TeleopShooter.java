// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class TeleopShooter extends CommandBase {
  
  private Shooter shooter;

  /** Creates a new TeleopShooter. */
  public TeleopShooter(Shooter shooter) {
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double flywheelVelocity;
    double loaderVelocity;
    double hoodPosition;

    flywheelVelocity = SmartDashboard.getNumber("Flywheel Motor Velocity", 0);
    if (flywheelVelocity < -1) {
      flywheelVelocity = -1;
    }
    if (flywheelVelocity > 1) {
      flywheelVelocity = 1;
    }

    loaderVelocity = SmartDashboard.getNumber("Loader Motor Velocity", 0);
    if (loaderVelocity < -1) {
      loaderVelocity = -1;
    }
    if (loaderVelocity > 1) {
      loaderVelocity = 1;
    }

    hoodPosition = SmartDashboard.getNumber("Set Hood Position", 0);
    if (hoodPosition < -1) {
      hoodPosition = -1;
    }
    if (hoodPosition >1) {
      hoodPosition = 1;
    }

    // System.out.println(flywheelVelocity);
    // shooter.setFlywheelVelocity(flywheelVelocity);

    // System.out.println(loaderVelocity);
    // shooter.setLoaderVelocity(loaderVelocity);

    System.out.println(hoodPosition);
    shooter.setHoodPosition(hoodPosition);
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