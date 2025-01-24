package frc.robot.constants;

import java.util.Arrays;

public class LedConstants {
  public static final int[] LedStrips = {(21 / 3), (21 / 3), (33 / 3), (33 / 3)};
  public static final int LedsLength = Arrays.stream(LedStrips).sum();
}
