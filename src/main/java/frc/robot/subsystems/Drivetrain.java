// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX leftMotorLeader;
  private WPI_TalonFX leftMotorFollower;
  private WPI_TalonFX rightMotorLeader;
  private WPI_TalonFX rightMotorFollower;

  private final double kP = 0.0;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kF = 0.0;

  private final double ticksPerMeter = 1;
  private final double drivetrainWidth = 1;

  private Pose2d robotPose;

  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;

  private IMU imu;

  double heading = 0;

  /** Creates a new Drive. */
  public Drivetrain(IMU imu) {
    this.imu=imu;
    leftMotorLeader = new WPI_TalonFX(31);
    leftMotorFollower = new WPI_TalonFX(27);
    rightMotorLeader = new WPI_TalonFX(30);
    rightMotorFollower = new WPI_TalonFX(50);
    robotPose = new Pose2d();
    kinematics = new DifferentialDriveKinematics(drivetrainWidth);
    heading = imu.getAngleRadians();
    odometry = new DifferentialDriveOdometry(new Rotation2d(heading));
    setupDrivetrainMotors();
  }

  @Override
  public void periodic() {
    heading = imu.getAngleRadians();
    robotPose = odometry.update(new Rotation2d(imu.getAngleRadians()),
        leftMotorLeader.getSelectedSensorPosition() / ticksPerMeter,
        rightMotorLeader.getSelectedSensorPosition() / ticksPerMeter);
  }

  public void setPower(double leftPower, double rightPower)
  {
    leftMotorLeader.set(ControlMode.PercentOutput, leftPower);
    rightMotorLeader.set(ControlMode.PercentOutput, rightPower);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    leftMotorLeader.set(ControlMode.Velocity, wheelSpeeds.leftMetersPerSecond * ticksPerMeter * 0.1);
    rightMotorLeader.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond * ticksPerMeter * 0.1);
  }

  // Fills in actual speeds
  public void getChassisSpeeds(ChassisSpeeds speeds) {
    speeds = kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public Pose2d getPoseMeters() {
    return robotPose;
  }

  public void resetOdometry(Pose2d newPose) {
    robotPose = new Pose2d(newPose.getTranslation(), newPose.getRotation());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMotorLeader.getSelectedSensorVelocity(),
      rightMotorLeader.getSelectedSensorVelocity()
    );
  }

  public void setBrakeMode(boolean braking) {
    if (braking) {
      leftMotorLeader.setNeutralMode(NeutralMode.Brake);
      rightMotorLeader.setNeutralMode(NeutralMode.Brake);
    } else {
      leftMotorLeader.setNeutralMode(NeutralMode.Coast);
      rightMotorLeader.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setupDrivetrainMotors() {
    leftMotorLeader.configFactoryDefault();
    rightMotorLeader.configFactoryDefault();
    leftMotorFollower.configFactoryDefault();
    rightMotorFollower.configFactoryDefault();

    leftMotorLeader.setSafetyEnabled(false);
    rightMotorLeader.setSafetyEnabled(false);
    leftMotorFollower.setSafetyEnabled(false);
    rightMotorFollower.setSafetyEnabled(false);

    leftMotorLeader.setNeutralMode(NeutralMode.Brake);
    rightMotorLeader.setNeutralMode(NeutralMode.Brake);
    leftMotorFollower.setNeutralMode(NeutralMode.Brake);
    rightMotorFollower.setNeutralMode(NeutralMode.Brake);

    leftMotorLeader.setInverted(true);
    leftMotorFollower.setInverted(true);

    rightMotorLeader.setInverted(false);
    rightMotorFollower.setInverted(false);

    leftMotorFollower.follow(leftMotorLeader);
    rightMotorFollower.follow(rightMotorLeader);

    leftMotorLeader.configPeakOutputForward(1.0);
    leftMotorLeader.configPeakOutputReverse(-1.0);

    rightMotorLeader.configPeakOutputForward(1.0);
 
    leftMotorLeader.configPeakOutputReverse(-1.0);
    rightMotorLeader.configPeakOutputReverse(-1.0);

    // leftMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
    // rightMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 28, 33, 0.25));
    leftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftMotorLeader.setSensorPhase(true);
    rightMotorLeader.setSensorPhase(true);

    leftMotorLeader.setSelectedSensorPosition(0);
    rightMotorLeader.setSelectedSensorPosition(0);

    // Add PID constants
    leftMotorLeader.config_kP(0, kP);
    leftMotorLeader.config_kI(0, kI);
    leftMotorLeader.config_kD(0, kD);
    leftMotorLeader.config_kF(0, kF);
    leftMotorLeader.configMaxIntegralAccumulator(0, 400);

    rightMotorLeader.config_kP(0, kP);
    rightMotorLeader.config_kI(0, kI);
    rightMotorLeader.config_kD(0, kD);
    rightMotorLeader.config_kF(0, kF);
    rightMotorLeader.configMaxIntegralAccumulator(0, 400);
 
    leftMotorLeader.setIntegralAccumulator(0);
    rightMotorLeader.setIntegralAccumulator(0);
  }
}