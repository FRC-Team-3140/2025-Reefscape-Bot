// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SignalTower extends SubsystemBase {
  private static SignalTower instance = null;

  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.LEDCount);
  private AddressableLED led = null;

  private int rainbowFirstPixelHue = 0;

  public final int ledRainbowSpan = Constants.LED.RainbowSpan;

  private Mode mode = Mode.Rainbow;
  private int solidR = 0;
  private int solidG = 0;
  private int solidB = 0;

  public enum Mode {
    Rainbow,
    Solid
  }

  public static SignalTower getInstance() {
    if (instance == null) {
      instance = new SignalTower();
    }

    return instance;
  }

  /** Creates a new SignalTower. */
  private SignalTower() {
    led = new AddressableLED(Constants.LED.Port);
    led.setLength(buffer.getLength());
    led.start();
  }

  @Override
  public void periodic() {
    switch (mode) {
      case Rainbow:
        rainbow();
        break;

      case Solid:
        setColor(solidR, solidG, solidB);
        break;

      default:
        break;
    }
  }

  private void rainbow() {
    int hue;
    for (int i = 0; i < buffer.getLength(); i++) {
      hue = (rainbowFirstPixelHue + i * ledRainbowSpan) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    led.setData(buffer);
  }

  private void setColor(int R, int G, int B) {
    if (Math.max(Math.max(R, B), G) > 255) {
      System.err
          .println(this.getClass().getSimpleName() + " called setColor with an invalid value! Won't set color...");
      return;
    }

    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, R, G, B);
    }

    led.setData(buffer);
  }

  public void setRainbow() {
    mode = Mode.Rainbow;
  }

  public void setSolid(int R, int G, int B) {
    mode = Mode.Solid;

    solidR = R;
    solidG = G;
    solidB = B;
  }
}
