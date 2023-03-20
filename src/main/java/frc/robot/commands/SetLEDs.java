// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;

public class SetLEDs extends CommandBase {
  LEDs subLEDs;
  PatternType desiredPattern;
  Arm subArm;

  public SetLEDs(LEDs subLEDs, Arm subArm) {
    this.subLEDs = subLEDs;
    this.subArm = subArm;

    addRequirements(subLEDs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    desiredPattern = PatternType.ColorWavesPartyPalette;
    subLEDs.setLEDPattern(desiredPattern);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
