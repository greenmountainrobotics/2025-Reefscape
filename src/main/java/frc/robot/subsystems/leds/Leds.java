package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LedConstants;
import frc.robot.util.Alliance;

public class Leds extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuffer;

  public static class State {
    public static boolean AprilTagsConnected = false; // works
    public static boolean AutoEnabled = false; // works
    public static boolean DrivingToPose = false; // works

    public static boolean Enabled = false; // works
  }

  public Leds(AddressableLED leds) {
    this.leds = leds;
    ledBuffer = new AddressableLEDBuffer(LedConstants.LedsLength);

    leds.setLength(ledBuffer.getLength());

    leds.setData(ledBuffer);
    leds.start();
  }

  private void autoSetColors() {
    Color allianceColor = Alliance.isRed() ? Color.kRed : Color.kBlue;

    /*
    apriltags not connected -> yellow
    robot started -> green
    auto -> green pulse
    running to position -> alliance color pulse
    teleop -> alliance color static
    */

    if (!State.AprilTagsConnected) {
      flashColor(Color.kYellow, Color.kBlack, 2);
    } else if (State.AutoEnabled) {
      pulseColor(Color.kGreen, Color.kBlack, 2);
    } else if (State.DrivingToPose) {
      pulseColor(allianceColor, Color.kBlack, 0.5);
    } else if (State.Enabled) {
      showSolidColor(allianceColor);
    } else {
      showSolidColor(Color.kGreen);
    }
  }

  private void showSolidColor(Color color) {
    for (int i = 0; i < LedConstants.LedsLength; i++) {
      ledBuffer.setLED(i, color);
    }
  }

  private void flashColor(Color color1, Color color2, double pulseTimeSeconds) {
    double ratio = (Timer.getFPGATimestamp() % pulseTimeSeconds) / pulseTimeSeconds;
    showSolidColor(
        new Color(
            (color1.red * (1 - ratio)) + (color2.red * ratio),
            (color1.green * (1 - ratio)) + (color2.green * ratio),
            (color1.blue * (1 - ratio)) + (color2.blue * ratio)));
  }

  private void pulseColor(Color color1, Color color2, double pulseTimeSeconds) {
    double ratio =
        (Math.sin(Math.PI * 2 * ((Timer.getFPGATimestamp() % pulseTimeSeconds) / pulseTimeSeconds))
                + 1)
            / 2;

    showSolidColor(
        new Color(
            (color1.red * (1 - ratio)) + (color2.red * ratio),
            (color1.green * (1 - ratio)) + (color2.green * ratio),
            (color1.blue * (1 - ratio)) + (color2.blue * ratio)));
  }

  // starts from outside and closes in
  private void loadingBar(Color color, double percentage) {
    for (int i : LedConstants.LedStrips) {}
  }

  @Override
  public void periodic() {
    autoSetColors();
    leds.setData(ledBuffer);
  }
}
