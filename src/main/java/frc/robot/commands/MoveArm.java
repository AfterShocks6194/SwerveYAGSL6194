// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {

  Arm subArm;

  double shoulderAdjuster;
  double elbowAdjuseter;

  Rotation2d shoulderAngle;
  Rotation2d elbowAngle;

  public MoveArm(Arm subArm, double d, double e) {
    this.subArm = subArm;
    this.shoulderAdjuster = d;
    this.elbowAdjuseter = e;

    addRequirements(this.subArm);
  }

  @Override
  public void initialize() {
    shoulderAngle = subArm.getShoulderPosition();
    elbowAngle = subArm.getElbowPosition();

    subArm.setGoalState(ArmState.NONE);
  }

  @Override
  public void execute() {

    if (!subArm.isGoalState(ArmState.NONE)) {
      shoulderAngle = subArm.getGoalState().shoulderAngle;
      elbowAngle = subArm.getGoalState().elbowAngle;
    }

    shoulderAngle = shoulderAngle
        .plus(Rotation2d.fromDegrees(shoulderAdjuster() * prefArm.shoulderAdjustRange.getValue()));
    elbowAngle = elbowAngle
        .plus(Rotation2d.fromDegrees(elbowAdjuseter() * prefArm.elbowAdjustRange.getValue()));

    subArm.setJointPositions(shoulderAngle, elbowAngle);
  }

  private double elbowAdjuseter() {
    return elbowAdjuseter;
  }

  private double shoulderAdjuster() {
    return shoulderAdjuster;
  }

  @Override
  public void end(boolean interrupted) {
    subArm.neutralJointOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
