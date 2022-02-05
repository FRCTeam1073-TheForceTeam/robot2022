// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

public class HubTracking extends SubsystemBase {
  // Resources:
  // CANifier - https://docs.ctre-phoenix.com/en/stable/ch12_BringUpCANifier.html
  // setLEDOutput() - https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1_c_a_nifier.html
  private final CANifier canifier;
  private final LEDChannel channel;


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

    // Initialize the canifier variables:
    canifier = new CANifier(8);
    channel = LEDChannel.LEDChannelA;

  }

  @Override
  public void periodic() {
  }

  public void sampleHubData(HubData data){

  }

  public boolean isHubVisible(){
    return hubVisible;
  }

  public void setLEDIntensity(double percent){
    // System.out.println("Set Itensity: ");
    canifier.setLEDOutput(percent, channel); 
  }
}