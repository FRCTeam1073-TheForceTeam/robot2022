// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopDrivetrain extends CommandBase {

  private Drivetrain drivetrain;
  private ChassisSpeeds chassisSpeeds;
  private final double forwardSpeed = 2;
  private final double rotationSpeed = 2;

  /** Creates a new TeleopDrivetrain. */
  public TeleopDrivetrain(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassisSpeeds = new ChassisSpeeds();
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = OI.driverController.getLeftY()*forwardSpeed;
    double rotation = OI.driverController.getRightX()*rotationSpeed;
    chassisSpeeds.vxMetersPerSecond = forward;
    chassisSpeeds.omegaRadiansPerSecond = rotation;
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds.vxMetersPerSecond = 0;
    chassisSpeeds.omegaRadiansPerSecond = 0;
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}