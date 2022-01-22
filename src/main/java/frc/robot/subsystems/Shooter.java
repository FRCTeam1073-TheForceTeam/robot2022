// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX  prototypeMotor;
  /** Creates a new Shooter. */
  public Shooter() {
    prototypeMotor = new WPI_TalonFX(20);
    prototypeMotor.configFactoryDefault();
    prototypeMotor.setSafetyEnabled(false);
    prototypeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 25, 25, 0.2));
    prototypeMotor.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putNumber("Prototype Motor Velocity", 0);
  }

  public void setPrototypePower(double power) {
    prototypeMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double output;
    output = SmartDashboard.getNumber("Prototype Motor Velocity", 0);
    if (output < -1) {
      output = -1;
    }
    if (output > 1) {
      output = 1;
    }
    System.out.println(output);
    setPrototypePower(output);
  }

  /*  TODO: Is this just flywheel control
      Is there an adjustable hood
      Is there a turret
      How many motors
  */
}