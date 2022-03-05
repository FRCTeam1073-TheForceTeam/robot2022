// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FrontSonar;

public class HangStartPositioning extends CommandBase {

  private Drivetrain drivetrain;
  private FrontSonar frontsonar;

  private double rightRange;
  private double leftRange;
  private final double sensorSpacing = 0.57785; //distance between the sensors in meters

  private final double hangarLength = 1.6; //distance from back of hangar in meters. Need to add additional distance between the sensor and the hooks on the robot!

  private ChassisSpeeds chassisSpeeds;

  /** Creates a new HangStartPositioning. */
  public HangStartPositioning(Drivetrain drivetrain, FrontSonar frontsonar) {
    this.drivetrain = drivetrain;
    this.frontsonar = frontsonar;

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
    chassisSpeeds.omegaRadiansPerSecond = calculateRotation();
    chassisSpeeds.vxMetersPerSecond = calculateForward();
    // drivetrain.setChassisSpeeds(chassisSpeeds);
    // System.out.println("Rotation: " + chassisSpeeds.omegaRadiansPerSecond);
    // System.out.println("Forward: " + chassisSpeeds.vxMetersPerSecond);
    // System.out.println("Raw Range: " + rightRange);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double calculateRotation(){
    rightRange = frontsonar.getRangeRight(0);
    leftRange = frontsonar.getRangeLeft(0);
    return Math.asin((rightRange - leftRange) / sensorSpacing);
  }

  public double calculateForward(){
    return Math.min(rightRange, leftRange) - hangarLength;
  }
}