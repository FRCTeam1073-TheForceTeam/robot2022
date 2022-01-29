// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase 
{
  private WPI_TalonFX leftMotorLeader;
  private WPI_TalonFX rightMotorLeader;

  /** Creates a new Drive. */
  public Drivetrain() 
  {
    leftMotorLeader = new WPI_TalonFX(20);
    rightMotorLeader = new WPI_TalonFX(46);
    setUpDrivetrainMotors();
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putBoolean("[1] A button", OI.driverController.getAButton());
    SmartDashboard.putBoolean("[2] B button", OI.driverController.getBButton());
    SmartDashboard.putBoolean("[3] X button", OI.driverController.getXButton());
    SmartDashboard.putBoolean("[4] Y button", OI.driverController.getYButton());

    SmartDashboard.putBoolean("[5] Left bumper", OI.driverController.getLeftBumper());
    SmartDashboard.putBoolean("[6] Right bumper", OI.driverController.getRightBumper());
    SmartDashboard.putBoolean("[7] Start button", OI.driverController.getStartButton());
    SmartDashboard.putBoolean("[8] Back button", OI.driverController.getBackButton());

    setPower(OI.driverController.getLeftY()*0.5,OI.driverController.getRightY()*0.5);
  }

  public void setPower(double leftPower, double rightPower)
  {
    leftMotorLeader.set(ControlMode.PercentOutput, leftPower);
    rightMotorLeader.set(ControlMode.PercentOutput, rightPower);
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
    if (braking) {
      leftMotorLeader.setNeutralMode(NeutralMode.Brake);
      rightMotorLeader.setNeutralMode(NeutralMode.Brake);
    } else {
      leftMotorLeader.setNeutralMode(NeutralMode.Coast);
      rightMotorLeader.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setUpDrivetrainMotors(){
    leftMotorLeader.configFactoryDefault();
    leftMotorLeader.setSafetyEnabled(false);
    leftMotorLeader.setNeutralMode(NeutralMode.Brake);
    leftMotorLeader.setInverted(true);
    leftMotorLeader.configPeakOutputForward(1.0);
    leftMotorLeader.configPeakOutputReverse(-1.0);
    // leftMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
    leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftMotorLeader.setSensorPhase(true);
    leftMotorLeader.setSelectedSensorPosition(0);
    // Add PID constants

    rightMotorLeader.configFactoryDefault();
    rightMotorLeader.setSafetyEnabled(false);
    rightMotorLeader.setNeutralMode(NeutralMode.Brake);
    rightMotorLeader.setInverted(false);
    rightMotorLeader.configPeakOutputForward(1.0);
    rightMotorLeader.configPeakOutputReverse(-1.0);
    // rightMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
    rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMotorLeader.setSensorPhase(true);
    rightMotorLeader.setSelectedSensorPosition(0);
    // Add PID constants
  }
}