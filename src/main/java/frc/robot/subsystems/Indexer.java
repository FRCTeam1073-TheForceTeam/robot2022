// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class Indexer extends SubsystemBase {
  private double motorVelocity;
  private NetworkTableInstance nti;
  private NetworkTable indexerCargoTable;
  private NetworkTableEntry isCurrentCargoThere;
  private NetworkTableEntry isNextCargoThere;
  private NetworkTableEntry isCurrentCargoRed;
  private NetworkTableEntry isNextCargoRed;

  /** Creates a new Indexer. */
  public Indexer(NetworkTableInstance ntinst) {
    motorVelocity = 0.0;
    nti = ntinst;
    indexerCargoTable = nti.getTable("INDEXER CARGO");
    isCurrentCargoThere = indexerCargoTable.getEntry("Is Current Cargo There");
    isCurrentCargoThere.setBoolean(false);
    isNextCargoThere = indexerCargoTable.getEntry("Is Next Cargo There");
    isNextCargoThere.setBoolean(false);
    isCurrentCargoRed = indexerCargoTable.getEntry("Is Current Cargo Red");
    isCurrentCargoRed.setBoolean(false);
    isNextCargoRed = indexerCargoTable.getEntry("Is Next Cargo Red");
    isNextCargoRed.setBoolean(false);
  }

  public Indexer() {
  }

  @Override
  public void periodic() {
    if (motorVelocity > 0.0) {
      Robot.getBling().setSlot(1, 60, 168, 50);
    } else {
      Robot.getBling().setSlot(1, 168, 50, 50);
    }
  }

  public void setWheelVelocity(double velocity) {
    motorVelocity = velocity;
  }

  public double getWheelVelocity() {
    return 0.0;
  }

  public boolean isCurrentCargoThere(){
    return false;
  }

  public boolean isNextCargoThere(){
    return false;
  }

  public boolean isCurrentCargoRed(){
    return false;
  }

  public boolean isNextCargoRed(){
    return false;
  }

  public int getNumCargo(){
    return 0;
  }

  /*
   * TODO: Does this launch the cargo,
   * is it position based or velocity based,
   * how many motors
   */
}