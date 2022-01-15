package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Allow the commands running in the robot to express themselves visually.
 */
public class Bling extends SubsystemBase {
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  public int ledR = 0;
  public int ledG = 0;
  public int ledB = 0;

  public Bling() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(8);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void initialize() {

  }

  @Override
  public void periodic() {
    //setColorRGBAll(ledR, ledG, ledB);
    m_led.setData(m_ledBuffer);
    SmartDashboard.putNumberArray("Color", new double[]{ledR,ledG,ledB});
  }

  public void setColorDouble(double r, double g, double b) {
    setColor(
      (int) (255 * MathUtil.clamp(r, 0, 1)),
      (int) (255 * MathUtil.clamp(g, 0, 1)),
      (int) (255 * MathUtil.clamp(b, 0, 1))
    );
  }

  public void setLEDDouble(int i, double r, double g, double b) {
    setRGB(
      i,
      (int) (255 * MathUtil.clamp(r, 0, 1)),
      (int) (255 * MathUtil.clamp(g, 0, 1)),
      (int) (255 * MathUtil.clamp(b, 0, 1))
    );
  }

  public void setColor(int r, int g, int b) {
    ledR = r;
    ledG = g;
    ledB = b;
  }

  public void setColor(String color) {
    if (color.equals("red")) {
      setRGBAll(85, 0, 0);
    } else if (color.equals("orange")) {
      setRGBAll(85, 55, 0);
    } else if (color.equals("yellow")) {
      setRGBAll(85, 85, 0);
    } else if (color.equals("green")) {
      setRGBAll(0, 85, 0);
    } else if (color.equals("blue")) {
      setRGBAll(0, 0, 85);
    } else if (color.equals("purple")) {
      setRGBAll(42, 0, 42);
    } else if (color.equals("black")) {
      setRGBAll(0, 0, 0);
    } else if (color.equals("white")) {
      setRGBAll(85, 85, 85);
    }
  }

  public void setRGB(int i,int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
  }

  public void setRGBAll(int r, int g, int b) {
    ledR = r;
    ledG = g;
    ledB = b;
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }
}