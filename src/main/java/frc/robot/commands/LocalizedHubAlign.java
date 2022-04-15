// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Localizer;

public class LocalizedHubAlign extends CommandBase {

  Drivetrain drivetrain;
  Localizer localizer;
  IMU imu;
  double targetHeading;
  double currentAngle;
  ChassisSpeeds speeds;
  HubTracking hubTracking;
  int trackCounter;
  public static final int MAX_TRACKS=10;

  /** Creates a new LocalizedHubAlign. */
  public LocalizedHubAlign(Drivetrain drivetrain, Localizer localizer, HubTracking hubTracking) {
    this.drivetrain = drivetrain;
    this.imu = this.drivetrain.getIMU();
    this.localizer = localizer;
    this.hubTracking = hubTracking;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetHeading = localizer.getAngleToHub();
    currentAngle = imu.getAngleRadians();
    speeds = new ChassisSpeeds();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = imu.getAngleRadians();
    double diff = MathUtil.angleModulus(currentAngle - targetHeading);
    speeds.omegaRadiansPerSecond = curve(diff);
    drivetrain.setChassisSpeeds(speeds);
    if(hubTracking.isHubVisible()){
      trackCounter++;
    }else{
      trackCounter--;
    }
    trackCounter = Math.max(0, Math.min(MAX_TRACKS, trackCounter));
  }
  
  public double curve(double x){
    return -2.0 * Math.signum(x) * (0.2 + Math.abs(x));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Only stop it if we got interrupted or it got there without seeing the hub
    if (!(trackCounter >= MAX_TRACKS)) {
      speeds.omegaRadiansPerSecond = 0;
      drivetrain.setChassisSpeeds(speeds);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(MathUtil.angleModulus(targetHeading - currentAngle)) < 0.1) || (trackCounter >= MAX_TRACKS);
  }
}
