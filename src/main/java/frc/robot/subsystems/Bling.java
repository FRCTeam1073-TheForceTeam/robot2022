package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Allow the commands running in the robot to express themselves visually.
 */
public class Bling extends SubsystemBase {
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  Collector collector;

  private int counter_rainbow = 0;
  private int move_rainbow = 0;

  private double brightness = 0.5;

  public int slotLength;

  int burst_done;
  int burstCount;

  int time;
  int time_burst;
  int time_blinkyLEDs;
  int leds_from_middle;
  int move;
  int dash_num;
  int dash_time;
  String gameData;
  int gameDataBlinkCount;
  int gameR;
  int gameG;
  int gameB;

  public int[] rgbArr = { 0, 0, 0 };

  public boolean cleared = false;

  public Bling() {
    m_led = new AddressableLED(1);
    m_ledBuffer = new AddressableLEDBuffer(66);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    slotLength = m_ledBuffer.getLength()/8;
  }

  public void initialize() {
    time_burst = 0;
    burst_done = 0;
    time = 0;
    time_blinkyLEDs = 0;
    leds_from_middle = 0;
    move = 0;
    dash_num = 0;
    dash_time = 0;
    gameDataBlinkCount = 0;
    gameR = 0;
    gameG = 0;
    gameB = 0;
  }

  @Override
  public void periodic() {
    
    if (!cleared) {
      LEDRainbow(0, m_ledBuffer.getLength() / 2, 10);

      // batteryBling(0, 1, 8.0, 12.5);

      // setColorRGBAll(255, 0, 0);

      // setSlot(5, 255, 0, 0);

      reverseRange(0, m_ledBuffer.getLength() / 2, m_ledBuffer.getLength() / 2);

      m_led.setData(m_ledBuffer);

    } else {
      clearLEDs();
    }
  }



  public AddressableLEDBuffer getM_LEDBuffer() {
    return m_ledBuffer;
  }



  public void cleared() {
    cleared = true;
  }



  public void uncleared() {
    cleared = false;
  }



  public void clearLEDs() {
    setColorRGBAll(0, 0, 0);
  }



  public void setLEDData() {
    m_led.setData(m_ledBuffer);
  }



  public void setArray(String color) {
    if (color.equals("red")) {
      rgbArr[0] = 85;
      rgbArr[1] = 0;
      rgbArr[2] = 0;

    } else if (color.equals("orange")) {
      rgbArr[0] = 85;
      rgbArr[1] = 55;
      rgbArr[2] = 0;

    } else if (color.equals("yellow")) {
      rgbArr[0] = 85;
      rgbArr[1] = 85;
      rgbArr[2] = 0;

    } else if (color.equals("green")) {
      rgbArr[0] = 0;
      rgbArr[1] = 85;
      rgbArr[2] = 0;

    } else if (color.equals("blue")) {
      rgbArr[0] = 0;
      rgbArr[1] = 0;
      rgbArr[2] = 85;

    } else if (color.equals("purple")) {
      rgbArr[0] = 42;
      rgbArr[1] = 0;
      rgbArr[2] = 42;

    } else if (color.equals("black")) {
      rgbArr[0] = 0;
      rgbArr[1] = 0;
      rgbArr[2] = 0;

    } else if (color.equals("white")) {
      rgbArr[0] = 85;
      rgbArr[1] = 85;
      rgbArr[2] = 85;
    }
  }



  public void setLED(int i, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
    // m_led.setData(m_ledBuffer);
  }



  // This sets two leds with the same color
  public void setLEDs2(int i, int i2, int r, int g, int b) {
    m_ledBuffer.setRGB(i, r, g, b);
    m_ledBuffer.setRGB(i2, r, g, b);
    m_led.setData(m_ledBuffer);
  }



  // setColorRGBAll sets the LEDs all to one color
  public void setColorRGBAll(int r, int g, int b) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }



  // setColorHSVAll() sets all of the LEDs to one color using HSV
  public void setColorHSVAll(int h, int s, int v) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }
    m_led.setData(m_ledBuffer);
  }



  // alternateRGB sets a range of LEDs where the even are one color and the odd are another
  public void alternateRGB(int min, int number, int r1, int g1, int b1, int r2, int g2, int b2) {
    int max = min + number;
    for (int i = min; i < (max - 1); i += 4) {
      m_ledBuffer.setRGB(i, r1, g1, b1);
      m_ledBuffer.setRGB(i + 1, r1, g1, b1);
    }

    for (int j = min + 2; j < (max - 1); j += 4) {
      m_ledBuffer.setRGB(j, r2, g2, b2);
      m_ledBuffer.setRGB(j + 1, r2, g2, b2);
    }
    m_led.setData(m_ledBuffer);
  }



  // alternateHSV() has the same functionality as alternateRGB() except with HSV
  // (hue, saturation + value)
  public void alternateHSV(int min, int number, int h1, int s1, int v1, int h2, int s2, int v2) {
    int max = min + number;
    for (int i = min; i < (max); i = i + 2) {
      m_ledBuffer.setHSV(i, h1, s1, v1);
    }

    for (int j = min + 1; j < (max); j = j + 2) {
      m_ledBuffer.setHSV(j, h2, s2, v2);
    }
    m_led.setData(m_ledBuffer);
  }



  // rangeRGB() sets a range of LEDs to one color
  public void rangeRGB(int min, int number, int r, int g, int b) {
    if (number != 1) {
      int max = min + number;
      for (int i = min; i < (max); i++) {
        m_ledBuffer.setRGB(i, r, g, b);
      }
    } else {
      m_ledBuffer.setRGB(min, r, g, b);
    }
  }



  // rangeHSV() same as rangeRGB() except using HSV values
  public void rangeHSV(int min, int number, int h, int s, int v) {
    int max = min + number;

    for (int i = min; i < (max); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }

    m_led.setData(m_ledBuffer);
  }



  public void setSlot(int slotNum, int r, int g, int b) {
    if (slotNum <= 7 && slotNum > 0) {
      if (slotLength == 1) {
        m_ledBuffer.setRGB(slotNum, r, g, b);
      } else {
        rangeRGB((slotNum * slotLength), slotLength, r, g, b);
      }
      
    }
  }



  // batteryBling() sets the LED color and number depending on the battery voltage
  public void batteryBling(int minLEDsVolts, int numberLEDsVolts, double min_volts, double max_volts) {
    for (int i = minLEDsVolts; i < (minLEDsVolts + numberLEDsVolts); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    
    double volts = RobotController.getBatteryVoltage();

    if (volts > max_volts) {
      volts = max_volts;
    }

    // First, it calculates the percentage of leds that will turn on.
    // amount above the minimum voltage / range of volts
    // the -1 and +1 account for the one that is always on.
    int num = (int) (Math.round(((volts - min_volts) / (max_volts - min_volts)) * (numberLEDsVolts - 1)) + 1);

    // If less than 1/3 of the leds are lit up, the light is red.
    // If between 1/3 and 2/3 of the leds are lit up, the light is yellow.
    // If more than 2/3 of the leds are lit up, the light is green.
    if (num > (2 * (numberLEDsVolts / 3))) {
      rangeRGB(minLEDsVolts, num, 0, 128, 255);
    } else if (num > (numberLEDsVolts / 3)) {
      rangeRGB(minLEDsVolts, num, 255, 255, 0);
    } else {
      rangeRGB(minLEDsVolts, num, 255, 0, 0);
    }
  }



  public void LEDRainbow(int startLEDs, int numLEDs, int targetTime) {
    if (counter_rainbow >= targetTime) {
      move_rainbow++;
      move_rainbow %= 12;
      counter_rainbow = 0;

      for (int i = startLEDs; i < (startLEDs + numLEDs); i++) {
        int ledColor = (i + move_rainbow) % 12;
        if (ledColor == 0 || ledColor == 1) {
          // Sets first LED, then sets every 6 after it "red"
          m_ledBuffer.setRGB(i, (int) (255 * brightness), 0, 0);
          
        } else if (ledColor == 2 || ledColor == 3) {
          // Sets second LED, then sets every 6 after it "orange"
          m_ledBuffer.setRGB(i, (int) (230 * brightness), (int) (55 * brightness), 0);
          
        } else if (ledColor == 4 || ledColor == 5) {
          // Sets third LED, then sets every 6 after it "yellow"
          m_ledBuffer.setRGB(i, (int) (252 * brightness), (int) (227 * brightness), 0);
          
        } else if (ledColor == 6 || ledColor == 7) {
          // Sets fourth LED, then sets every 6 after it "green"
          m_ledBuffer.setRGB(i, 0, (int) (255 * brightness), 0);
          
        } else if (ledColor == 8 || ledColor == 9) {
          // Sets fifth LED, then sets every 6 after it "blue"
          m_ledBuffer.setRGB(i, 0, 0, (int) (255 * brightness));
          
        } else if (ledColor == 10 || ledColor == 11) {
          // Sets sixth LED, then sets every 6 after it "purple"
          m_ledBuffer.setRGB(i, (int) (128 * brightness), 0, (int) (128 * brightness));
        }
      }

    } else {
      counter_rainbow++;
    }
  }



  public void reverseRange(int startRange, int numRange, int setRangeStart) {
    for (int i = 0; i < numRange; i++) {
      m_ledBuffer.setLED(setRangeStart + numRange - i - 1, m_ledBuffer.getLED8Bit(i));
    }
  }



  // burst() lights LEDs from the middle out
  public void burst(int length, int r, int g, int b, boolean init) {
    // Calculates the middle led(s) of the led string
    int middle1 = (int) (Math.floor((length / 2)));
    int middle2 = (int) (Math.ceil((length / 2)));

    if ((leds_from_middle + middle2) < (length - 1) && time_burst < 15) {
      // If there are still more LEDs to change and it is not yet time to change
      // Wait until the 2000th time
      time_burst = time_burst + 1;
    } else if ((leds_from_middle + middle2) < (length - 1)) {
      // If it is time to change and there are still more LEDs to change
      // Reset the time
      time_burst = 0;
      // Moves the LEDs out from the center by one light
      leds_from_middle = leds_from_middle + 1;
      // Sets the LEDs
      setColorRGBAll(0, 0, 0);
      setLEDs2(middle1 - leds_from_middle, middle2 + leds_from_middle, r, g, b);
    } else {
      // Resets the time and says that the burst is finished
      time_burst = 0;
      setColorRGBAll(0, 0, 0);
      if (init) {
        burst_done = 1;
      }
    }
  }



  // blinkyLightsTwoColors() switches the lights between two colors for all LEDs
  public void blinkyLightsTwoColors(int h, int s, int v, int r, int g, int b) {
    if (time < 50) {
      // Sets the LEDs to the first color
      setColorHSVAll(h, s, v);
      time = time + 1;
    } else if (time < 100) {
      // Sets the LEDs to the second color
      setColorRGBAll(r, g, b);
      time = time + 1;
    } else {
      // Resets the time
      time = 0;
    }
  }



  public void dashing(int min, int num, int r, int g, int b) {
    if (dash_time >= 50) {
      if (dash_num != 0) {
        setLED(min + ((dash_num - 1) % num), 0, 0, 0);
      }
      setLED(min + (dash_num % num), r, g, b);
      dash_num += 1;
    }
    dash_time += 1;
  }



  // blinkyLights() flashes lights on and off in one color for a range of LEDs
  public void blinkyLights(int minLEDsBlink, int numberLEDsBlink, int r, int g, int b) {
    if (time_blinkyLEDs < 30) {
      // Sets the LEDs to the specified color
      rangeRGB(minLEDsBlink, numberLEDsBlink, r, g, b);
      time_blinkyLEDs = time_blinkyLEDs + 1;
    } else if (time_blinkyLEDs < 60) {
      // Turns the LEDs off
      time_blinkyLEDs = time_blinkyLEDs + 1;
      rangeRGB(minLEDsBlink, numberLEDsBlink, 0, 0, 0);
    } else {
      // Resets the time counter
      time_blinkyLEDs = 0;
      gameDataBlinkCount = gameDataBlinkCount + 1;
    }
  }



  // movingLEDs() lights a single LED that moves up the range and then restarts
  public void movingLEDs(int minLEDsMove, int numberLEDsMove) {
    if (time < 50) {
      // Waits until the 50th time
      time = time + 1;
    } else {
      if (move < numberLEDsMove - 1) {
        // Changes the LED that is lit
        move = move + 1;
      } else {
        move = 0;
      }
      // Sets the LED that is lit
      int set = minLEDsMove + move;
      rangeRGB(minLEDsMove, numberLEDsMove, 0, 0, 0);
      setLED(set, 255, 0, 0);
    }
  }
}