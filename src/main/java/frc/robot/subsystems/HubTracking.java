// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.InterpolatorTable;
import frc.robot.components.InterpolatorTable.InterpolatorTableEntry;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.CANifier.LEDChannel;

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
  // Resources:
  // CANifier - https://docs.ctre-phoenix.com/en/stable/ch12_BringUpCANifier.html
  // setLEDOutput() - https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1_c_a_nifier.html
  private final CANifier canifier;
  private final LEDChannel lowerChannel;
  private final LEDChannel upperChannel;
  private NetworkTableEntry outputX;
  private NetworkTableEntry outputY;
  private NetworkTableEntry outputArea;
  private NetworkTableInstance ntinst;
  private HubData hubData;
  private boolean hubVisible;
  private InterpolatorTable rangeInterpolator;
  private InterpolatorTable elevationInterpolator;
  private boolean hubVisibility;

  /** Creates a new HubTracking. */
  public HubTracking() {
    hubVisible = false;

    // Initialize the canifier variables:
    canifier = new CANifier(8);
    canifier.configFactoryDefault();
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 100);
    lowerChannel = LEDChannel.LEDChannelA;
    upperChannel = LEDChannel.LEDChannelB;
    ntinst = NetworkTableInstance.getDefault();

    outputX = ntinst.getTable("HUB").getEntry("Hub X");
    outputY = ntinst.getTable("HUB").getEntry("Hub Y");
    outputArea = ntinst.getTable("HUB").getEntry("Hub Area");
    
    hubData = new HubData();
    rangeInterpolator = new InterpolatorTable(
      new InterpolatorTableEntry(9, 1.0),
      new InterpolatorTableEntry(119, 1.5),
      new InterpolatorTableEntry(194, 2.0),
      new InterpolatorTableEntry(256, 2.5),
      new InterpolatorTableEntry(302, 3.0),
      new InterpolatorTableEntry(340, 3.5),
      new InterpolatorTableEntry(375, 4.0),
      new InterpolatorTableEntry(398, 4.5),
      new InterpolatorTableEntry(426, 5.0),
      new InterpolatorTableEntry(447, 5.5)
    );
    elevationInterpolator = new InterpolatorTable(
      new InterpolatorTableEntry(9, 1.12),
      new InterpolatorTableEntry(119, 0.94),
      new InterpolatorTableEntry(194, 0.8),
      new InterpolatorTableEntry(256, 0.69),
      new InterpolatorTableEntry(302, 0.6),
      new InterpolatorTableEntry(340, 0.53),
      new InterpolatorTableEntry(375, 0.48),
      new InterpolatorTableEntry(398, 0.43),
      new InterpolatorTableEntry(426, 0.39),
      new InterpolatorTableEntry(447, 0.35)
    );
  }

  @Override
  public void periodic() {
    hubData.cx = outputX.getNumber(0).intValue();
    hubData.cy = outputY.getNumber(0).intValue();
    hubData.area = outputArea.getNumber(0).intValue();
    hubData.azimuth = (hubData.cx - 320)/320.0;
    hubData.range = rangeInterpolator.getValue(hubData.cy);
    hubData.elevation = elevationInterpolator.getValue(hubData.cy);
    hubData.timestamp = System.currentTimeMillis();
    SmartDashboard.putNumber("Hub Tracker/cx", hubData.cx);
    SmartDashboard.putNumber("Hub Tracker/cx", hubData.cy);
    SmartDashboard.putNumber("Hub Tracker/area", hubData.area);
    SmartDashboard.putNumber("Hub Tracker/azimuth", hubData.azimuth);
    SmartDashboard.putNumber("Hub Tracker/range", hubData.range);
    SmartDashboard.putNumber("Hub Tracker/elevation", hubData.elevation);
    SmartDashboard.putNumber("Hub Tracker/timestamp", hubData.timestamp);
    SmartDashboard.putBoolean("Hub Visibility", isHubVisible());
  }

  public void sampleHubData(HubData data){
    data.cx = hubData.cx;
    data.cy = hubData.cy;
    data.area = hubData.area;
    data.range = hubData.range;
    data.azimuth = hubData.azimuth;
    data.elevation = hubData.elevation;
    data.timestamp = hubData.timestamp;
  }

  public boolean isHubVisible()
  {
    return hubData.area > 0;
  }

  public void setLEDIntensity(double percent){
    //System.out.println("Set Intensity: "+percent);
    canifier.setLEDOutput(percent, lowerChannel);
    canifier.setLEDOutput(percent, upperChannel);
  }
}