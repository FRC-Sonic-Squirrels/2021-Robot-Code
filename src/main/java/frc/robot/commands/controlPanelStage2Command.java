/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.controlPanelSubsystem;

public class controlPanelStage2Command extends CommandBase {
  /**
   * Creates a new controlPanelStage2Command.
   */
  controlPanelSubsystem m_controlPanelSubsystem;
  String gameData;
  String currentColor = m_controlPanelSubsystem.getColor();
  int count = 0;

  public controlPanelStage2Command(controlPanelSubsystem cp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controlPanelSubsystem = cp;
    addRequirements(m_controlPanelSubsystem);

    m_controlPanelSubsystem.resetColorCount();

    // use a public get method to make private variables visible to other classes
    count = m_controlPanelSubsystem.getColorCount();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    int getMoveToGamePosition = m_controlPanelSubsystem.getMoveToGamePosition();
    if (getMoveToGamePosition < 0) {
      m_controlPanelSubsystem.setSpeed(-0.2);
      //move clockwise
    }
    if (getMoveToGamePosition >= 1) {
      m_controlPanelSubsystem.setSpeed(0.2);
      //move counterclockwise
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    char stage2ColorChar = gameData.charAt(0);
    char currentColorChar = currentColor.charAt(0);

    if (stage2ColorChar == 'Y' && currentColorChar == 'G') {
      return true;
    }
    if (stage2ColorChar == 'G' && currentColorChar == 'Y') {
      return true;
    }
    if (stage2ColorChar == 'B' && currentColorChar == 'R') {
      return true;
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'B') {
      return true;
    }
    return false;
  }
}
