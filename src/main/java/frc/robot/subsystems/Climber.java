// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {

  private WPI_TalonFX spoolMotorRight;
  private WPI_TalonFX extensionMotorRight;
  private WPI_TalonFX spoolMotorLeft;
  private WPI_TalonFX extensionMotorLeft;

  // private CANCoder spoolCANCoderRight;
  // private CANCoder spoolCANCoderLeft;

  // TODO: get actual values
  private final double spoolGearRatio = 56.25;
  private final double extensionGearRatio = 50.0;

  private final double spoolTicksPerRadian = 2048.0 * spoolGearRatio / (2.0 * Math.PI);
  private final double extensionTicksPerRadian = 2048.0 * extensionGearRatio / (2.0 * Math.PI);

  private final DigitalInput DI1 = new DigitalInput(2);
  private final DigitalInput DI2 = new DigitalInput(3);
  private final DigitalInput DI3 = new DigitalInput(4);
  private final DigitalInput DI4 = new DigitalInput(5);

  // private final Bling bling = new Bling();
  
  SlewRateLimiter spoolFilter = new SlewRateLimiter(80.0);
  SlewRateLimiter extensionFilter = new SlewRateLimiter(80.0);

  double targetSpoolVelocity = 0;
  double targetExtensionVelocity = 0;

  double currentSpoolVelocity = 0;
  double currentExtensionVelocity = 0;

  private double spool_kP = 0.1;
  private double spool_kI = 0.001;
  private double spool_kD = 0;
  private double spool_kF = 0.05;

  private double extension_kP = 0.2;
  private double extension_kI = 0.003;
  private double extension_kD = 0;
  private double extension_kF = 0.05;

  /** Creates a new Climber. */
  public Climber() {
    spoolMotorRight = new WPI_TalonFX(44);
    extensionMotorRight = new WPI_TalonFX(17);
    
    spoolMotorLeft = new WPI_TalonFX(32);
    extensionMotorLeft = new WPI_TalonFX(29);

    resetSpoolMotor(spoolMotorRight);
    resetExtensionMotor(extensionMotorRight);

    resetSpoolMotor(spoolMotorLeft);
    resetExtensionMotor(extensionMotorLeft);

    spoolMotorLeft.setInverted(false);
    spoolMotorRight.setInverted(true);

    extensionMotorLeft.setInverted(true);
    extensionMotorRight.setInverted(false);

    spoolMotorLeft.follow(spoolMotorRight);
    extensionMotorLeft.follow(extensionMotorRight);

    spoolMotorLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 22, 28, 0.25));
    extensionMotorLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 22, 28, 0.25));

    spoolMotorRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 22, 28, 0.25));
    extensionMotorRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 22, 28, 0.25));

    // spooler encoder
    // spoolCANCoderRight = new CANCoder(0);

    SmartDashboard.putNumber("climber-spool_kP", spool_kP);
    SmartDashboard.putNumber("climber-spool_kI", spool_kI);
    SmartDashboard.putNumber("climber-spool_kD", spool_kD);
    SmartDashboard.putNumber("climber-spool_kF", spool_kF);

    SmartDashboard.putNumber("climber-extension_kP", extension_kP);
    SmartDashboard.putNumber("climber-extension_kI", extension_kI);
    SmartDashboard.putNumber("climber-extension_kD", extension_kD);
    SmartDashboard.putNumber("climber-extension_kF", extension_kF);
    SmartDashboard.putBoolean("Update", false);

    setPIDs(spoolMotorRight, spool_kP, spool_kI, spool_kD, spool_kF, 1000);
    setPIDs(extensionMotorRight, extension_kP, extension_kI, extension_kD, extension_kF, 1000);

  }

  @Override
  public void periodic() {
    if (DI1.get()) {
      Robot.getBling().setSlot(1, 255, 0, 0);
    } else {
      Robot.getBling().setSlot(1, 0, 0, 0);
    }

    if (DI2.get()) {
      Robot.getBling().setSlot(2, 0, 255, 0);
    } else {
      Robot.getBling().setSlot(2, 0, 0, 0);
    }

    if (DI3.get()) {
      Robot.getBling().setSlot(3, 0, 0, 255);
    } else {
      Robot.getBling().setSlot(3, 0, 0, 0);
    }

    if (DI4.get()) {
      Robot.getBling().setSlot(4, 255, 255, 255);
    } else {
      Robot.getBling().setSlot(4, 0, 0, 0);
    }

    // This method will be called once per scheduler run
    double limitedSpoolVelocity = spoolFilter.calculate(targetSpoolVelocity);
    double limitedExtensionVelocity = extensionFilter.calculate(targetExtensionVelocity);

    double rawSpoolVel = limitedSpoolVelocity * spoolTicksPerRadian * 0.1;
    double rawExtensionVel = limitedExtensionVelocity * extensionTicksPerRadian * 0.1;
    
    // spoolMotorRight.set(ControlMode.Velocity, rawSpoolVel);
    // extensionMotorRight.set(ControlMode.Velocity, rawExtensionVel);

    spoolMotorRight.set(ControlMode.Velocity, rawSpoolVel);
    extensionMotorRight.set(ControlMode.Velocity, rawExtensionVel);

    currentSpoolVelocity = spoolMotorRight.getSelectedSensorVelocity() / spoolTicksPerRadian * 10.0;
    currentExtensionVelocity = extensionMotorRight.getSelectedSensorVelocity() / extensionTicksPerRadian * 10.0;

    //debug
    SmartDashboard.putNumber("target spool velocity", targetSpoolVelocity);
    SmartDashboard.putNumber("target extension velocity", targetExtensionVelocity);
    SmartDashboard.putNumber("actual spool velocity", currentSpoolVelocity);
    SmartDashboard.putNumber("actual extension velocity", currentExtensionVelocity);
    // SmartDashboard.putNumber("raw spool velocity", rawSpoolVel);
    SmartDashboard.putNumber("raw extension velocity", rawExtensionVel);
    SmartDashboard.putNumber("spool output percent", spoolMotorRight.getMotorOutputPercent());

    if (SmartDashboard.getBoolean("Update", false)) 
    {
      spool_kP = SmartDashboard.getNumber("climber-spool_kP", 0);
      spool_kI = SmartDashboard.getNumber("climber-spool_kI", 0);
      spool_kD = SmartDashboard.getNumber("climber-spool_kD", 0);
      spool_kF = SmartDashboard.getNumber("climber-spool_kF", 0);

      extension_kP = SmartDashboard.getNumber("climber-extension_kP", 0);
      extension_kI = SmartDashboard.getNumber("climber-extension_kI", 0);
      extension_kD = SmartDashboard.getNumber("climber-extension_kD", 0);
      extension_kF = SmartDashboard.getNumber("climber-extension_kF", 0);

      // TODO: someone plz tell me what the 1000 does plz
      setPIDs(spoolMotorRight, spool_kP, spool_kI, spool_kD, spool_kF, 1000);
      setPIDs(extensionMotorRight, extension_kP, extension_kI, extension_kD, extension_kF, 1000);

      SmartDashboard.putBoolean("Update", false);
    }
  }

  public void setSpoolVelocity(double velocity){
    targetSpoolVelocity = velocity;
  }

  public void setExtensionVelocity(double angularVelocity){
    targetExtensionVelocity = angularVelocity;
  }

  public void setSpoolPower(double power) {
    spoolMotorRight.set(ControlMode.PercentOutput, power);
  }

  public void setExtensionPower(double power) {
    extensionMotorRight.set(ControlMode.PercentOutput, power);
  }

  public void setPIDs(WPI_TalonFX motor, double p_val, double i_val, double d_val, double f_val, double max_integrator) {
    motor.config_kP(0,p_val);
    motor.config_kI(0,i_val);
    motor.config_kD(0,d_val);
    motor.config_kF(0,f_val);
    motor.setIntegralAccumulator(0);
    motor.configMaxIntegralAccumulator(0, max_integrator);
  }

  /**
   * Sets a *single* climber motor to a given angular velocity.
   * This disables *all other motors*, and is meant to be used for indexing the climber.
   * @param motor
   * @param velocity
   */
  // public void setMotorVelocity(ClimberMotor motor, double velocity){
  // }

  public boolean hasNotHung(){
    return true;
    //if you've never attempted to hang (without resetting)
  }

  public boolean hangIsStable(){
    return false;
    //if it's not swinging and winch is fully retracted
  }

  public double getSpoolPosition(){
    return spoolMotorRight.getSelectedSensorPosition() / spoolTicksPerRadian;
  }

  public double getExtensionPosition(){
    return extensionMotorRight.getSelectedSensorPosition() / extensionTicksPerRadian;
  }



  // public double getMotorPosition(ClimberMotor motor){
  //   return 0;
  // }

  //no brake mode?
  private void resetSpoolMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setSafetyEnabled(false);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25));
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSelectedSensorPosition(0);
  }

  private void resetExtensionMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setSafetyEnabled(false);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25));
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSelectedSensorPosition(0);
  }
}