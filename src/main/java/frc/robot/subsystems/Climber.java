// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  enum ClimberMotor{
    LIFT_LEFT,
    LIFT_RIGHT,
    GRAPPLE_LEFT,
    GRAPPLE_RIGHT
  }

  //How many climber arms?

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLiftVelocity(double velocity) {

  }

  public void setGrappleVelocity(double angularVelocity) {

  }

  /**
   * Sets a *single* climber motor to a given angular velocity.
   * This disables *all other motors*, and is meant to be used for indexing the climber.
   * @param motor
   * @param velocity
   */
  public void setMotorVelocity(ClimberMotor motor, double velocity){
  }

  public double getLiftPosition() {
    return 0;
  }

  public double getGrapplePosition() {
    return 0;
  }

  public double getMotorPosition(ClimberMotor motor){
    return 0;
  }
}