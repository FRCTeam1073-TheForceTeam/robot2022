// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase 
{
  /**
     * TODO: Set correct CAN ID for motors
  */
  WPI_TalonFX liftMotor;
  /**
     * TODO: Set correct CAN ID for motors
  */
  WPI_TalonFX collectMotor;

  private double lift_kP;
  private double lift_kI;
  private double lift_kD;
  private double lift_kF;

  private double collect_kP;  
  private double collect_kI;
  private double collect_kD;
  private double collect_kF;  

  private void resetMotors()
  {
    liftMotor.configFactoryDefault();
    collectMotor.configFactoryDefault();

    liftMotor.setSafetyEnabled(false);
    collectMotor.setSafetyEnabled(false);

    liftMotor.setNeutralMode(NeutralMode.Brake);
    collectMotor.setNeutralMode(NeutralMode.Brake);

    liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25));
    collectMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25));

    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    collectMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    liftMotor.setSelectedSensorPosition(0);
    collectMotor.setSelectedSensorPosition(0);

    liftMotor.config_kP(0, lift_kP);
    liftMotor.config_kI(0, lift_kI);
    liftMotor.config_kD(0, lift_kD);
    liftMotor.config_kF(0, lift_kF);

    collectMotor.config_kP(0, collect_kP);
    collectMotor.config_kI(0, collect_kI);
    collectMotor.config_kD(0, collect_kD);
    collectMotor.config_kF(0, collect_kF);
  }

  /** Creates a new Collector. */
  public Collector() 
  {
    liftMotor = new WPI_TalonFX(6); // set CAN ID
    collectMotor  = new WPI_TalonFX(7); // set CAN ID

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public void setTargetPosition(double pos) 
  {

  }

  public double getActualPosition() 
  {
    return 0;
  }

  // velocity is the speed of the ball in the intake in meters/second
  public void setIntakeVelocity(double velocity) 
  {

  }

  public double getIntakeVelocity() {
    return 0.0;
  }

  public boolean isIntakeStalled() {
    return false;
  }
}
