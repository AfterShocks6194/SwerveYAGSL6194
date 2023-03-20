// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double CHASSIS_MASS = ROBOT_MASS;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  // use this to not slam into an april tag. Based on half our length.
  public static final double xOffset = .48;
  public static final int BLINKIN_PWM = 3;
  public static final boolean OUTPUT_DEBUG_VALUES = true;

  public static class OperatorConstants {
    // public static final int kDriverControllerPort = 0;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.00;
    public static final double LEFT_Y_DEADBAND = 0.00;

  }

  public static class ArmConstants {

    // public static final int shoulderMotorID = 15;
    // public static final int intakeMotorID = 16;
    // public static final int wristMotorID = 17;
    // public static final double pos0 = 0;
    // public static final double pos1 = 0;
    // public static final double pos2 = 0;
    // public static final double bottomPickup = 0;
    // public static final double topLimit = 0;
    // public static final double bottomLimit = 0;
    // public static final double constrictedBottomLimit = 0;

  }

  // public static class VisionConstants {

  // /**
  // * Physical location of the camera on the robot, relative to the center of the
  // robot.
  // */
  // public static final Transform3d CAMERA_TO_ROBOT =
  // new Transform3d(new Translation3d(Units.inchesToMeters(14), 0.0,
  // Units.inchesToMeters(8)), new Rotation3d());
  // public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

  // public static final double FIELD_LENGTH_METERS = 16.54175;
  // public static final double FIELD_WIDTH_METERS = 8.0137;
  // }

  public static final class Auton {

    public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0.0);
    public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0.0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.0);

    // public static final double MAX_SPEED = 4;
    // public static final double MAX_ACCELERATION = 2;
  }

  public static final class constArm {
    public static final boolean SHOULDER_MOTOR_INVERT = true;
    public static final boolean ELBOW_MOTOR_INVERT = false;

    public static final double SHOULDER_GEAR_RATIO = 200.0;
    public static final double ELBOW_GEAR_RATIO = 200.0;

    public static final boolean SHOULDER_ABSOLUTE_ENCODER_INVERT = false;
    public static final boolean ELBOW_ABSOLUTE_ENCODER_INVERT = false;

    public static final NeutralMode SHOULDER_MOTOR_BREAK = NeutralMode.Brake;
    public static final NeutralMode ELBOW_MOTOR_BREAK = NeutralMode.Brake;

    public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.470116);
    public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.823558);

    public static final double PRAC_SHOULDER_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.346228);
    public static final double PRAC_ELBOW_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.127852);

    public static final double SHOULDER_LENGTH = Units.inchesToMeters(30.0);
    public static final double ELBOW_LENGTH = Units.inchesToMeters(34.0);

    public static final double SHOULDER_FORWARD_LIMIT = Units.degreesToRadians(90.0);
    public static final double SHOULDER_REVERSE_LIMIT = Units.degreesToRadians(-135.0);

    public static final double ELBOW_FORWARD_LIMIT = Units.degreesToRadians(70);
    public static final double ELBOW_REVERSE_LIMIT = Units.degreesToRadians(-100);

    public enum ArmState {
      NONE(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
      STOWED(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(66)),
      FLOOR_INTAKE(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-15)),
      SHELF_INTAKE(Rotation2d.fromDegrees(-20), Rotation2d.fromDegrees(10)),
      HIGH_CONE_SCORE(Rotation2d.fromDegrees(-11), Rotation2d.fromDegrees(25)),
      HIGH_CONE_SCORE_LOWERED(Rotation2d.fromDegrees(-11), Rotation2d.fromDegrees(5)),
      MID_CONE_SCORE(Rotation2d.fromDegrees(-59), Rotation2d.fromDegrees(40)),
      MID_CONE_SCORE_LOWERED(Rotation2d.fromDegrees(-59), Rotation2d.fromDegrees(20)),
      HIGH_CUBE_SCORE_PLACE(Rotation2d.fromDegrees(37.5), Rotation2d.fromDegrees(-30.5)),
      HIGH_CUBE_SCORE_SHOOT(Rotation2d.fromDegrees(-99), Rotation2d.fromDegrees(32)),
      MID_CUBE_SCORE(Rotation2d.fromDegrees(-95.5), Rotation2d.fromDegrees(22)),
      HYBRID_SCORE(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(0));

      public Rotation2d shoulderAngle;
      public Rotation2d elbowAngle;

      private ArmState(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
        this.shoulderAngle = shoulderAngle;
        this.elbowAngle = elbowAngle;
      }
    }
  }

  public static final class Drivebase {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
}
