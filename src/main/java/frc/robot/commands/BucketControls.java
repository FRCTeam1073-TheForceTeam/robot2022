// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Bucket;
import frc.robot.subsystems.OI;

public class BucketControls extends CommandBase {

  Bucket bucket;
  /** Creates a new BucketControls. */
  public BucketControls(Bucket bucket_) {
    bucket=bucket_;
    addRequirements(bucket);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bucket.setPower(0);
  }

  double zone = 0.05;
  public double deadzone(double a) {
    return (Math.abs(a) < zone) ? 0 : Math.signum(a) * Math.pow((Math.abs(a) - zone) / (1 - zone), 1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("POV", OI.operatorController.getPOV());
    int pov=OI.operatorController.getPOV();
    if((pov==315)||(pov==0)||(pov==45)){
      bucket.setPower(0.9);
    }else if((pov==135)||(pov==180)||(pov==225)){
      bucket.setPower(-0.9);
    }else{
      bucket.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
