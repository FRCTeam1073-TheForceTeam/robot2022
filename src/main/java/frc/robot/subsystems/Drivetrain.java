// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // DEBUG:
  private final boolean debug = true;
  
  private WPI_TalonFX leftMotorLeader;
  private WPI_TalonFX leftMotorFollower;
  private WPI_TalonFX rightMotorLeader;
  private WPI_TalonFX rightMotorFollower;

  private double kP = 0.15 * 0.75;
  private double kI = 0.002 * 0;
  private double kD = 0.0;
  private double kF = 0.052;
  //0.5: 10100, 0.75:
  //Old: 0.5 : 9650, 0.75 : 14550


  private final double currentLimit = 28;
  private final double currentThreshold = 30; 
  private final double currentThresholdTime = 0.25;

  private final double wheelRadius = Units.inchesToMeters(2.0);
  private final double gearRatio = (12.0 / 44.0) * (24.0 / 50.0);
  //(ticks/output meters) = (ticks/motor rotation)/(gearRatio output rotations/motor rotation)/(2pi*wheelRadius meters/output rotation)
  private final double ticksPerMeter = 2048.0 / gearRatio / (2.0 * Math.PI * wheelRadius);
  private final double drivetrainWidth = Units.inchesToMeters(24.660);

  private Pose2d robotPose;

  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveWheelSpeeds wheelSpeeds;

  private IMU imu;

  double heading = 0;

  NetworkTable drivetrainTable;
  NetworkTableEntry pEntry;
  NetworkTableEntry iEntry;
  NetworkTableEntry dEntry;
  NetworkTableEntry fEntry;
  NetworkTableEntry updateButton;

  Field2d field;

  SlewRateLimiter leftMotorLimiter;
  SlewRateLimiter rightMotorLimiter;
  private DifferentialDriveWheelSpeeds limitedTargetWheelSpeeds;

  private double rateLimit = 3.0;
  private DifferentialDriveWheelSpeeds targetWheelSpeeds;
  private int counter = 0;

  private final boolean isPowerMode = false;

  public static final boolean initMotorsInCoastMode = true;

  /** Creates a new Drive. */
  public Drivetrain(IMU imu) {
    drivetrainTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
    pEntry = drivetrainTable.getEntry("kP");
    iEntry = drivetrainTable.getEntry("kI");
    dEntry = drivetrainTable.getEntry("kD");
    fEntry = drivetrainTable.getEntry("kF");

    updateButton = drivetrainTable.getEntry("update constants");

    updateButton.setBoolean(true);
    pEntry.setDouble(kP);
    iEntry.setDouble(kI);
    dEntry.setDouble(kD);
    fEntry.setDouble(kF);

    this.imu = imu;
    leftMotorLeader = new WPI_TalonFX(30);
    leftMotorFollower = new WPI_TalonFX(50);
    rightMotorLeader = new WPI_TalonFX(31);
    rightMotorFollower = new WPI_TalonFX(27);
    robotPose = new Pose2d();
    kinematics = new DifferentialDriveKinematics(drivetrainWidth);
    heading = imu.getAngleRadians();
    odometry = new DifferentialDriveOdometry(new Rotation2d(heading));
    wheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

    leftMotorLimiter = new SlewRateLimiter(rateLimit);
    rightMotorLimiter = new SlewRateLimiter(rateLimit);

    targetWheelSpeeds = new DifferentialDriveWheelSpeeds();
    limitedTargetWheelSpeeds = new DifferentialDriveWheelSpeeds();

    setupDrivetrainMotors();
    if (initMotorsInCoastMode) {
      System.out.println("[Drivetrain] WARNING: Drivetrain motors set to COAST MODE");
      setBrakeMode(false);
    }
    field = new Field2d();
  }

  public void setRateLimit(double rateLimit_) {
    rateLimit = rateLimit_;
    leftMotorLimiter = new SlewRateLimiter(rateLimit);
    rightMotorLimiter = new SlewRateLimiter(rateLimit);
    leftMotorLimiter.reset(limitedTargetWheelSpeeds.leftMetersPerSecond);
    rightMotorLimiter.reset(limitedTargetWheelSpeeds.rightMetersPerSecond);
  }
  
  public IMU getIMU(){
    return imu;
  }
  
  boolean a = false;

  @Override
  public void periodic() {
    heading = imu.getAngleRadians();
    double leftPos = leftMotorLeader.getSelectedSensorPosition() / ticksPerMeter;
    double rightPos = rightMotorLeader.getSelectedSensorPosition() / ticksPerMeter;
    robotPose = odometry.update(
        new Rotation2d(imu.getAngleRadians()),
        leftPos,
        rightPos);
    wheelSpeeds.leftMetersPerSecond = leftMotorLeader.getSelectedSensorVelocity() / ticksPerMeter * 10.0;
    wheelSpeeds.rightMetersPerSecond = rightMotorLeader.getSelectedSensorVelocity() / ticksPerMeter * 10.0;

    limitedTargetWheelSpeeds.leftMetersPerSecond = leftMotorLimiter.calculate(targetWheelSpeeds.leftMetersPerSecond);
    limitedTargetWheelSpeeds.rightMetersPerSecond = rightMotorLimiter.calculate(targetWheelSpeeds.rightMetersPerSecond);

    if (!isPowerMode) {
      leftMotorLeader.set(ControlMode.Velocity, (targetWheelSpeeds.leftMetersPerSecond) * ticksPerMeter * 0.1);
      rightMotorLeader.set(ControlMode.Velocity, (targetWheelSpeeds.rightMetersPerSecond) * ticksPerMeter * 0.1);
    }

    drivetrainTable.getEntry("Left distance")
        .setDouble(Units.metersToFeet(leftPos));
    drivetrainTable.getEntry("Right distance")
        .setDouble(Units.metersToFeet(rightPos));

    SmartDashboard.putNumber("DrivetrainTemps/LEFT Max Temp (degs C)",
      Math.max(leftMotorLeader.getTemperature(), leftMotorFollower.getTemperature())
    );
    SmartDashboard.putNumber("DrivetrainTemps/RIGHT Max Temp (degs C)",
      Math.max(rightMotorLeader.getTemperature(), rightMotorFollower.getTemperature())
    );

    if (updateButton.getBoolean(false)) {
      kP = pEntry.getDouble(0);
      kI = iEntry.getDouble(0);
      kD = dEntry.getDouble(0);
      kF = fEntry.getDouble(0);

      leftMotorLeader.config_kP(0, kP);
      leftMotorLeader.config_kI(0, kI);
      leftMotorLeader.config_kD(0, kD);
      leftMotorLeader.config_kF(0, kF);
      leftMotorLeader.setIntegralAccumulator(0);

      rightMotorLeader.config_kP(0, kP);
      rightMotorLeader.config_kI(0, kI);
      rightMotorLeader.config_kD(0, kD);
      rightMotorLeader.config_kF(0, kF);
      rightMotorLeader.setIntegralAccumulator(0);

      updateButton.setBoolean(false);
    }

    field.setRobotPose(robotPose);
    if (debug){
      SmartDashboard.putData(field);
      SmartDashboard.putNumber("[Drivetrain] Robot pose/X position (meters)", robotPose.getTranslation().getX());
      SmartDashboard.putNumber("[Drivetrain] Robot pose/Y position (meters)", robotPose.getTranslation().getY());
      SmartDashboard.putNumber("[Drivetrain] Robot pose/Angle (radians)", robotPose.getRotation().getRadians());
    }  
  }

  public void setPower(double leftPower, double rightPower)
  {
    if (isPowerMode) {
      leftMotorLeader.set(ControlMode.PercentOutput, leftPower);
      rightMotorLeader.set(ControlMode.PercentOutput, rightPower);
    }
  }

  public ChassisSpeeds targetChassisSpeeds=new ChassisSpeeds();

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    targetChassisSpeeds = speeds;
    targetWheelSpeeds = kinematics.toWheelSpeeds(speeds);
  }

  // Fills in actual speeds
  public void getChassisSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds temp = kinematics.toChassisSpeeds(wheelSpeeds);
    speeds.vxMetersPerSecond = temp.vxMetersPerSecond;
    speeds.vyMetersPerSecond = temp.vyMetersPerSecond;
    speeds.omegaRadiansPerSecond = temp.omegaRadiansPerSecond;
  }

  // // Fills in actual speeds
  // public ChassisSpeeds tGetChassisSpeeds() {
  //   return kinematics.toChassisSpeeds(wheelSpeeds);
  // }

  public Pose2d getPoseMeters() {
    return robotPose;
  }

  public void resetOdometry(Pose2d newPose) {
    robotPose = new Pose2d(newPose.getTranslation(), newPose.getRotation());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void setBrakeMode(boolean braking) {
    if (braking) {
      leftMotorLeader.setNeutralMode(NeutralMode.Brake);
      rightMotorLeader.setNeutralMode(NeutralMode.Brake);
      leftMotorFollower.setNeutralMode(NeutralMode.Brake);
      rightMotorFollower.setNeutralMode(NeutralMode.Brake);
    } else {
      leftMotorLeader.setNeutralMode(NeutralMode.Coast);
      rightMotorLeader.setNeutralMode(NeutralMode.Coast);
      leftMotorFollower.setNeutralMode(NeutralMode.Coast);
      rightMotorFollower.setNeutralMode(NeutralMode.Coast);
    }
  }

  public double getLeftWheelPosition(){
    return leftMotorLeader.getSelectedSensorPosition() / ticksPerMeter;
  }

  public double getRightWheelPosition(){
    return rightMotorLeader.getSelectedSensorPosition() / ticksPerMeter;
  }
  
  public double getRawVelocity() {
    return (leftMotorLeader.getSelectedSensorVelocity() + rightMotorLeader.getSelectedSensorVelocity()) * 0.5;
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

    leftMotorLeader.setInverted(false);
    leftMotorFollower.setInverted(false);

    rightMotorLeader.setInverted(true);
    rightMotorFollower.setInverted(true);

    leftMotorFollower.follow(leftMotorLeader);
    rightMotorFollower.follow(rightMotorLeader);

    leftMotorLeader.configPeakOutputForward(1.0);
    leftMotorLeader.configPeakOutputReverse(-1.0);

    rightMotorLeader.configPeakOutputForward(1.0);
    rightMotorLeader.configPeakOutputReverse(-1.0);

    leftMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentThreshold, currentThresholdTime));
    rightMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentThreshold, currentThresholdTime));
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
    leftMotorLeader.configMaxIntegralAccumulator(0, 8000);

    rightMotorLeader.config_kP(0, kP);
    rightMotorLeader.config_kI(0, kI);
    rightMotorLeader.config_kD(0, kD);
    rightMotorLeader.config_kF(0, kF);
    rightMotorLeader.configMaxIntegralAccumulator(0, 8000);
 
    leftMotorLeader.setIntegralAccumulator(0);
    rightMotorLeader.setIntegralAccumulator(0);

    leftMotorFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    leftMotorFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    rightMotorFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    rightMotorFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
  }
  public static class Constants{
    public static double autoRateLimit = 3.0;
    public static double teleopRateLimit = 8.0;
  }
}
