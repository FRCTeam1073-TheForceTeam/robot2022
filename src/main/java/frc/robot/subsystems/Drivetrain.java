// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase 
{
  /** Creates a new Drive. */
  public Drivetrain() 
  {
    
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) 
  {
    
  }

  // Fills in actual speeds
  public void getChassisSpeeds(ChassisSpeeds speeds) 
  {
    speeds.vxMetersPerSecond = 0.0;
    // vy is always 0
    speeds.vyMetersPerSecond = 0.0;
    speeds.omegaRadiansPerSecond = 0.0;
  }

  public Pose2d getPoseMeters() 
  {
    return new Pose2d();
  }

  public void resetOdometry(Pose2d newPose) 
  {

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() 
  {
    return new DifferentialDriveWheelSpeeds();
  }

  public void setBrakeMode(boolean braking) 
  {
    
  }
}
