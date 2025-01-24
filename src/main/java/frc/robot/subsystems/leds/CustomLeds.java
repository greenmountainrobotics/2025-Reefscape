package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class CustomLeds extends AddressableLED {

  public CustomLeds(int id) {
    super(id);
  }

  @Override
  public void setData(AddressableLEDBuffer buffer) {
    AddressableLEDBuffer newBuffer = new AddressableLEDBuffer(buffer.getLength());

    for (int i = 0; i < buffer.getLength(); i++) {
      Color led = buffer.getLED(i);
      newBuffer.setLED(i, new Color(led.red, led.blue, led.green));
    }

    super.setData(newBuffer);
  }
}
