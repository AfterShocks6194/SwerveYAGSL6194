package frc.robot;

public class RobotMap {

  // order of subsystems (and adjacent classes) shall be:
  // controllers, drivetrain, arm, intake, collector, charger (if it exists),
  // vision, leds



  public static final class mapArm {

    public static final int SHOULDER_CAN = 28;
    public static final int ELBOW_CAN = 29;

    public static final int SHOULDER_ABSOLUTE_ENCODER_DIO = 1;
    public static final int ELBOW_ABSOLUTE_ENCODER_DIO = 2;
  }




  public static final class mapLEDs {
    public static final int BLINKIN_PWM = 3;
  }
}
