// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HubTracking;
import frc.robot.subsystems.HubTracking.HubData;

public class AlignToHub extends CommandBase {

  HubData data;
  double hubAzimuth;
  ChassisSpeeds chassisSpeeds;
  public static double scaleFactor = 3.0;
  double azimuthTolerance = 0.04;
  int timeoutCounter = 0;
  public static double minVelocity = 0.58;
  // double minError = minVelocity / scaleFactor;

  private HubTracking hubTracking;
  private Drivetrain drivetrain;

  private static boolean blinkToggle = false;

  /** Creates a new AlignToHub. */
  public AlignToHub(Drivetrain drivetrain, HubTracking hubTracking) {
    this.drivetrain = drivetrain;
    this.hubTracking = hubTracking;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassisSpeeds = new ChassisSpeeds();
    drivetrain.setChassisSpeeds(chassisSpeeds);
    timeoutCounter = 0;
    data = new HubData();
    blinkToggle = !blinkToggle;
    SmartDashboard.putBoolean("AlignToHub on", blinkToggle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hubTracking.sampleHubData(data);
    Robot.getBling().setSlot(3, 200, 200, 200);
    if (hubTracking.isHubVisible()) {
      hubAzimuth = data.azimuth;
      timeoutCounter = 0;
      if (Math.abs(hubAzimuth) < (minVelocity / scaleFactor)) {
        System.out.println("NARROW");
        if (hubAzimuth < 0) {
          chassisSpeeds.omegaRadiansPerSecond = minVelocity;
        }
        else {
          chassisSpeeds.omegaRadiansPerSecond = -minVelocity;
        }
      }
      else {
        chassisSpeeds.omegaRadiansPerSecond = -hubAzimuth * scaleFactor;
      }
    }
    else {
      timeoutCounter++;
      chassisSpeeds.omegaRadiansPerSecond = 0;
    }
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds = new ChassisSpeeds();
    Robot.getBling().setSlot(3, 0, 0, 0);


    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(hubAzimuth) < azimuthTolerance) || (timeoutCounter > 20);
  }
}