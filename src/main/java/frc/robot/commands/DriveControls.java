// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class DriveControls extends CommandBase {
  Drivetrain drivetrain; 
  /** Creates a new DriveControls. */
  public DriveControls(Drivetrain drivetrain_) {
    drivetrain = drivetrain_;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setPower(0, 0);
  }

  double zone = 0.05;
  public double deadzone(double a) {
    return (Math.abs(a) < zone) ? 0 : Math.signum(a) * Math.pow((Math.abs(a) - zone) / (1 - zone), 1.0);
  }

  ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.driverController.getRawButtonReleased(14)) {
      OI.zeroDriverController();
    }
    // SmartDashboard.putNumber("[Left X]", OI.getDriverLeftX());
    // SmartDashboard.putNumber("[Left Y]", OI.getDriverLeftY());
    // SmartDashboard.putNumber("[Right Y]", OI.getDriverRightY());
    // SmartDashboard.putNumber("[Right X]", OI.getDriverRightX());
    
    double forward = deadzone(OI.getDriverLeftY());
    double rotate = -deadzone(OI.getDriverRightX());
    chassisSpeeds.vxMetersPerSecond = forward * 3.80;
    chassisSpeeds.omegaRadiansPerSecond = rotate * 4.40;
    drivetrain.setChassisSpeeds(chassisSpeeds);

    // double fwd=0.5*OI.getDriverLeftY();
    // double rot=0.5*OI.getDriverRightX();
    // double fwd = SmartDashboard.getNumber("X", 0);
    // drivetrain.setPower(fwd + rot, fwd - rot);
    
    // SmartDashboard.putNumber("Y", rotate);
    // drivetrain.setPower((forward + rotate), (forward - rotate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
