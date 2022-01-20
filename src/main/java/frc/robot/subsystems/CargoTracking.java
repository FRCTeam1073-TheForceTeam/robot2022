// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoTracking extends SubsystemBase {
  public static class CargoData {
    public int cx = 0;
    public int cy = 0;
    public int vx = 0;
    public int vy = 0;
    public int area = 0;
    public long timestamp = 0;
    public double range = 0.0;
    public double angle = 0.0;

    public CargoData() {
      clear();
    }

    public void clear() {

    }
  }

  /** Creates a new CargoTracking. */
  public CargoTracking() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public int numberCargoVisible() {
    return 0;
  }

  // Needs to throw an exception when you ask for nonexistent data
  public void getCargoData(int index, CargoData cargo) {
    cargo.clear();
  }
}
