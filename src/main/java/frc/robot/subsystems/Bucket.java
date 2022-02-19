// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Bucket extends SubsystemBase {

  WPI_TalonSRX bucket;

  /** Creates a new Bucket. */
  public Bucket() {
    bucket=new WPI_TalonSRX(16);
    bucket.configFactoryDefault();
    bucket.setInverted(true);
    bucket.setNeutralMode(NeutralMode.Brake);
    bucket.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(
        true, 15, 20, 0.25
      )
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power){
    bucket.set(ControlMode.PercentOutput, power);
  }
}
