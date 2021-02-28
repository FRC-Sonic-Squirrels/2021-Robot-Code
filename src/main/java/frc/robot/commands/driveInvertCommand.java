/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSubsystem;

public class driveInvertCommand extends CommandBase {

  driveSubsystem m_drive;

  public driveInvertCommand(driveSubsystem drive) {
    addRequirements(drive);
    m_drive = drive;
  }

  @Override
  public void initialize() {
    if (m_drive.getDriveInvert() == true) {
      m_drive.setDriveInvert(false);
    }

    else {
      m_drive.setDriveInvert(true);
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setCoastMode();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
