// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private WPI_TalonFX spoolMotorRight;
  // private WPI_TalonFX spoolMotorLeft;
  private WPI_TalonFX extensionMotorRight;
  // private WPI_TalonFX extensionMotorLeft;

  // private CANCoder spoolCANCoderRight;
  // private CANCoder spoolCANCoderLeft;

  /** Creates a new Climber. */
  public Climber() {
    spoolMotorRight = new WPI_TalonFX(20);
    extensionMotorRight = new WPI_TalonFX(46);
    //set the left motors to follow + be inverted from the right motors

    resetSpoolMotor(spoolMotorRight);
    resetExtensionMotor(extensionMotorRight);
    //then set up followers (reset may need to be organized differently)

    //spooler encoder
    // spoolCANCoderRight = new CANCoder(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpoolVelocity(double velocity){

  }

  public void setExtensionVelocity(double angularVelocity){

  }

  /**
   * Sets a *single* climber motor to a given angular velocity.
   * This disables *all other motors*, and is meant to be used for indexing the climber.
   * @param motor
   * @param velocity
   */
  // public void setMotorVelocity(ClimberMotor motor, double velocity){
  // }

  public boolean isHanging(){
    return false;
  }

  public double getSpoolPosition(){
    return 0;
  }

  public double getExtensionPosition(){
    return 0;
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