// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubTracking extends SubsystemBase {
  public static class HubData {
    public int cx = 0;
    public int cy = 0;
    public int vx = 0;
    public int vy = 0;
    public int area = 0;
    public long timestamp = 0;
    public double range = 0.0;
    public double azimuth = 0.0;
    public double elevation = 0.0;

    public HubData() {
      clear();
    }

    public void clear() {

    }
  }

  private boolean hubVisible;

  /** Creates a new HubTracking. */
  public HubTracking() {
    hubVisible = false;
  }

  @Override
  public void periodic() {
    //Update hubVisible
  }

  public void sampleHubData(HubData data){

  }

  public boolean isHubVisible(){
    return hubVisible;
  }
}