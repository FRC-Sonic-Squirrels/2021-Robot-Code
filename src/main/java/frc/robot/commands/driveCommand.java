/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.driveSubsystem;

public class driveCommand extends CommandBase {

  driveSubsystem m_drive;
  XboxController driveController = RobotContainer.m_driveController;
  private double speedMultiplier = 0.7;
  private double rotationMultiplier = 0.75;

  public driveCommand(driveSubsystem drive) {
    addRequirements(drive);
    m_drive = drive;
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Drive Multiplier", speedMultiplier);
  }

  @Override
  public void execute() {

    double speed = 0;
    double rotation = 0;

    double m = SmartDashboard.getNumber("Drive Multiplier", 0);
    if (m != speedMultiplier) {
      speedMultiplier = Math.min(Math.abs(m), 1.0);
    }

    if (m_drive.getForzaModeEnabled()) {
      // right trigger forward, left trigger for reverse
      speed = driveController.getTriggerAxis(Hand.kRight) - driveController.getTriggerAxis(Hand.kLeft);
      rotation = -driveController.getX(Hand.kLeft);
    }
    else {
      speed = driveController.getY(Hand.kLeft);
      rotation = -driveController.getX(Hand.kRight);
    }

    if (m_drive.getSquaredInputs()) {
      // square the joystick inputs, give much more control at lower speed and rotation inputs
      speed = speed * Math.abs(speed);
      // rotation = rotation * Math.abs(rotation);
    }

    if (driveController.getBumper(Hand.kLeft) == false) {
      // drive slower, press button in engage turbo mode

      speed = speed * speedMultiplier;
    }
    rotation = rotation * rotationMultiplier;

    if (m_drive.getDriveInvert() == true) {
      // invert driving direction
      speed = -speed;
    }

    m_drive.arcadeDrive(speed, rotation);

    SmartDashboard.putBoolean("Drive Inverted", m_drive.getDriveInvert());
    SmartDashboard.putBoolean("Drive Forza Mode", m_drive.getForzaModeEnabled());
    SmartDashboard.putBoolean("Drive Square Inputs", m_drive.getSquaredInputs());
    SmartDashboard.putNumber("Drive Multiplier", speedMultiplier);
  }

  @Override
  public void end(boolean interrupted) {
    //m_drive.setCoastMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
