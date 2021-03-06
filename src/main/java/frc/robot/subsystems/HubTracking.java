// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
  private NetworkTableEntry outputX;
  private NetworkTableEntry outputY;
  private NetworkTableEntry outputArea;
  private NetworkTableEntry visionTime;
  private NetworkTableInstance ntinst;
  private HubData hubData;
  private boolean hubVisible;
  private InterpolatorTable rangeInterpolator;
  private InterpolatorTable elevationInterpolator;
  private boolean hubVisibility;
  private int sequencing;

  public double ledPower = 2.0;
  public double additionalRange = 0.0;
  Bling bling;

  /** Creates a new HubTracking. */
  public HubTracking() {
    hubVisible = false;

    // Initialize the canifier variables:
    canifier = new CANifier(8);
    canifier.configFactoryDefault();
    canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 100);
    ntinst = NetworkTableInstance.getDefault();

    outputX = ntinst.getTable("HUB").getEntry("Hub X");
    outputY = ntinst.getTable("HUB").getEntry("Hub Y");
    outputArea = ntinst.getTable("HUB").getEntry("Hub Area");
    visionTime = ntinst.getTable("HUB").getEntry("Vision Time");
    sequencing = 0;

    hubData = new HubData();
    rangeInterpolator = new InterpolatorTable(
      new InterpolatorTableEntry(37, 1.0),
      new InterpolatorTableEntry(133, 1.5),
      new InterpolatorTableEntry(208, 2.0),//NEW
      new InterpolatorTableEntry(263, 2.5),//NEW
      new InterpolatorTableEntry(309, 3.0),//NEW
      new InterpolatorTableEntry(349, 3.5),//NEW
      new InterpolatorTableEntry(381, 4.0),//NEW
      new InterpolatorTableEntry(409, 4.5),//NEW
      new InterpolatorTableEntry(433, 5.0) //NEW
      // new InterpolatorTableEntry(412, 4.5),
      // new InterpolatorTableEntry(432, 5.0),
      // new InterpolatorTableEntry(450, 5.5),
      // new InterpolatorTableEntry(466, 6.0)

      // new InterpolatorTableEntry(18, 1.0),
      // new InterpolatorTableEntry(112, 1.5),
      // new InterpolatorTableEntry(188, 2.0),
      // new InterpolatorTableEntry(247, 2.5),
      // new InterpolatorTableEntry(295, 3.0),
      // new InterpolatorTableEntry(328, 3.5),
      // new InterpolatorTableEntry(360, 4.0),
      // new InterpolatorTableEntry(391, 4.5),
      // new InterpolatorTableEntry(412, 5.0),
      // new InterpolatorTableEntry(432, 5.5),
      // new InterpolatorTableEntry(450, 6.0)
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
    ledPower = SmartDashboard.getNumber("Hub Tracker/ledPower", ledPower);
    SmartDashboard.putNumber("Hub Tracker/ledPower", ledPower);
    SmartDashboard.putNumber("Hub Tracker/Additional Range", additionalRange);
  }

  @Override
  public void periodic() {
    int newSequencing = visionTime.getNumber(0).intValue();
    if (newSequencing != sequencing){
      hubData.cx = outputX.getNumber(0).intValue();
      hubData.cy = outputY.getNumber(0).intValue();
      hubData.area = outputArea.getNumber(0).intValue();
      hubData.azimuth = (hubData.cx - 320)/320.0;
      hubData.range = rangeInterpolator.getValue(hubData.cy);
      hubData.elevation = elevationInterpolator.getValue(hubData.cy);
      hubData.timestamp = System.currentTimeMillis();    
      sequencing = newSequencing;
    }
    else if (System.currentTimeMillis() - hubData.timestamp > 1000) {
      hubData.cx = 0;
      hubData.cy = 0;
      hubData.area = 0;
      hubData.azimuth = 0;
      hubData.range = 0;
      hubData.elevation = 0;
    }
    if (bling == null) {
      bling = Robot.getBling();
    }
    SmartDashboard.putNumber("Hub Tracker/cx", hubData.cx);
    SmartDashboard.putNumber("Hub Tracker/cx", hubData.cy);
    SmartDashboard.putNumber("Hub Tracker/area", hubData.area);
    SmartDashboard.putNumber("Hub Tracker/azimuth", hubData.azimuth);
    SmartDashboard.putNumber("Hub Tracker/range", hubData.range);
    SmartDashboard.putNumber("Hub Tracker/elevation", hubData.elevation);
    SmartDashboard.putNumber("Hub Tracker/timestamp", hubData.timestamp);
    SmartDashboard.putBoolean("Hub Visibility", isHubVisible());
    ledPower = SmartDashboard.getNumber("Hub Tracker/ledPower", 1.0);
    additionalRange = SmartDashboard.getNumber("Hub Tracker/Additional Range", 0.0);

    if (isHubVisible()) {
      if (Math.abs(hubData.azimuth) < 0.15) {
        bling.setSlot(5, 200, 200, 0);
        bling.setSlot(6, 200, 200, 0);
        bling.setSlot(7, 200, 200, 0);
        bling.setSlot(8, 200, 200, 0);
        bling.setSlot(14, 200, 200, 0);
      } else {
        bling.setSlot(5, 100, 0, 100);
        bling.setSlot(6, 100, 0, 100);
        bling.setSlot(7, 100, 0, 100);
        bling.setSlot(8, 100, 0, 100);
        bling.setSlot(14, 100, 0, 100);
      }
    } else {
      bling.setSlot(5, 0, 0, 0);
      bling.setSlot(6, 0, 0, 0);
      bling.setSlot(7, 0, 0, 0);
      bling.setSlot(8, 0, 0, 0);
      bling.setSlot(14, 0, 0, 0);
    }
  }

  public void sampleHubData(HubData data) {
    data.cx = hubData.cx;
    data.cy = hubData.cy;
    data.area = hubData.area;
    data.range = hubData.range + additionalRange;
    data.azimuth = hubData.azimuth;
    data.elevation = hubData.elevation;
    data.timestamp = hubData.timestamp;
  }

  public boolean isHubVisible()
  {
    return hubData.area > 0;
  }

  public void setLEDIntensity(double redPercent, double greenPercent, double bluePercent){
    redPercent /= 2;
    greenPercent /= 2;
    bluePercent /= 2;
    canifier.setLEDOutput(redPercent, CANifier.LEDChannel.LEDChannelA);
    canifier.setLEDOutput(greenPercent, CANifier.LEDChannel.LEDChannelB);
    canifier.setLEDOutput(bluePercent, CANifier.LEDChannel.LEDChannelC);
  }
}