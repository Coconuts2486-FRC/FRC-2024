package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;

// import com.ctre.phoenix.ErrorCode;
// import com.ctre.phoenix.led.CANdleFaults;
// import com.ctre.phoenix.led.RainbowAnimation;

/**
 * This class represents the lights on a robot.
 */
public class Lights {

  public static CANdle candle = new CANdle(1);
  public static CANdleConfiguration config = new CANdleConfiguration();

  /**
   * Initializes the lights on the robot.
   */
  public static void CANdleInit() {
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness

    candle.configAllSettings(config);

    candle.setLEDs(255, 255, 255); // set the CANdle LEDs to white
  }

  /**
   * Runs a rainbow animation on the lights.
   * create a rainbow animation:
   *    max brightness
   *    half speed
   *    64 LEDs
   */
  public static void CANdleRunRainbow() {
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
    candle.animate(rainbowAnim);
  }

  /**
   * Runs a rainbow animation on the lights.
   * create a rainbow animation:
   *    max brightness
   *    half speed
   *    64 LEDs
   */
  public static void runCANdle() {
    if (Map.intakeStop.get()) {
      candle.setLEDs(106, 13, 173);
    } else {
      candle.setLEDs(255, 165, 0);
    }
  }

  ErrorCode error = candle.getLastError(); // gets the last error generated by the CANdle

  CANdleFaults faults = new CANdleFaults();

  ErrorCode faultsError = candle.getFaults(faults); // fills faults with the current CANdle faults; returns the last error generated
}
