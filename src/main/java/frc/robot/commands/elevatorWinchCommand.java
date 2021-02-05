/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevatorSubsystem;

public class elevatorWinchCommand extends CommandBase {

  elevatorSubsystem m_elevator;
  private XboxController driveController = RobotContainer.m_driveController;

  public elevatorWinchCommand(elevatorSubsystem elevator) {
    addRequirements(elevator);
    m_elevator = elevator;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {   
      if (driveController.getTriggerAxis(Hand.kLeft) >= 0.1) {
        m_elevator.brakeOff();
        m_elevator.setWinchPercentOutput(driveController.getTriggerAxis(Hand.kLeft));
        System.out.println(driveController.getTriggerAxis(Hand.kLeft));    
      } 
      
      else if (driveController.getTriggerAxis(Hand.kRight) >= 0.1) {
        m_elevator.brakeOff();
        m_elevator.setWinchPercentOutput(-driveController.getTriggerAxis(Hand.kRight));
        System.out.println(driveController.getTriggerAxis(Hand.kRight));
      }  
      
      else {
        m_elevator.stop();
      }
    }


  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
