// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.driveConstants.kDriveKinematics;
import java.util.List;

import javax.swing.plaf.InsetsUIResource;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import com.fearxzombie.limelight;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.Constants.driveConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.driveInvertCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //subsystems
  public final driveSubsystem m_drive = new driveSubsystem();

  public static final limelight m_limelight = new limelight("limelight-one");

  public static XboxController m_driveController = new XboxController(driveConstants.driveController);
  public static XboxController m_operatorController = new XboxController(driveConstants.operatorController);
  public static boolean limelightOnTarget = false;


  public RobotContainer() {
    configureButtonBindings();
    m_drive.setDefaultCommand(new driveCommand(m_drive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      //TODO: Oriente Driver Controls to be more like *Forza Controls* (Right Trigger = Throttle, left = Break/reverse)

      // Driver Controller Buttons 
    /*
    final JoystickButton driverAButton = new JoystickButton(m_driveController, Button.kA.value);
    final JoystickButton driverBButton = new JoystickButton(m_driveController, Button.kB.value);
    final JoystickButton driverStartButton = new JoystickButton(m_driveController, Button.kStart.value);
    final JoystickButton driverBackButton = new JoystickButton(m_driveController, Button.kBack.value);
    final JoystickButton driverLeftBumper = new JoystickButton(m_driveController, Button.kBumperLeft.value);
    */

    final JoystickButton driverRightBumper = new JoystickButton(m_driveController, Button.kBumperRight.value);
    
    
    // Operator Controller Buttons
    
    final JoystickButton opAButton = new JoystickButton(m_operatorController, Button.kA.value);
    final JoystickButton opBButton = new JoystickButton(m_operatorController, Button.kB.value);
    final JoystickButton opXButton = new JoystickButton(m_operatorController, Button.kX.value);
    final JoystickButton opYButton = new JoystickButton(m_operatorController, Button.kY.value);
    /*
    final JoystickButton opStartButton = new JoystickButton(m_operatorController, Button.kStart.value);
    final JoystickButton opBackButton = new JoystickButton(m_operatorController, Button.kBack.value);
    final JoystickButton opLeftBumper = new JoystickButton(m_operatorController, Button.kBumperLeft.value);
    final JoystickButton opRightBumper = new JoystickButton(m_operatorController, Button.kBumperRight.value);
    final POVButton opDPadUp = new POVButton(m_operatorController, 0);
    final POVButton opDPadDown = new POVButton(m_operatorController, 180);
    */
    
    // Driver Controls
      // Right Bumper - invert drive controls
      // Left Bumper - Slow down robot by 1/2
      
      //TODO: See why whenPressed is throwing an error
      //driverRightBumper.whenPressed(new driveInvertCommand(m_drive));
    
    // Operator Controls
      // Left Joystick - manual turret control
      // Left Trigger - manually move the indexer backwards
      // Right Trigger - manually move the indexer forwards
      // A Button - hold to deploy intake
      // B Button - stage balls for shooting
      // X Button - restage balls
      // Y Button - hold to eject balls out the back of the indexer
      // Right Bumper - shoot with hood up
      // Left Bumper - shoot with hood down
      // D Pad Up - manually increase ball count
      // D Pad Down - manually decrease ball count
      // Start Button - zero the turret
      // Back Button - spool up the shooter
      

      /*
      opAButton.whileHeld(new intakeDeployCommand(m_intake));
      opBButton.whenPressed(new indexerStageForShootingCommand(m_indexer));
      opXButton.whenPressed(new indexerRestageCommand(m_indexer));
      opYButton.whileHeld(new indexerReverseEjectCommand(m_indexer));
      opRightBumper.whileHeld(new hoodUpAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
      opLeftBumper.whileHeld(new hoodDownAutoShootCommand(m_indexer, m_turret, m_shooter, m_limelight));
      opDPadUp.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() + 1));
      opDPadDown.whenPressed(() -> m_indexer.setBallCount(m_indexer.getBallCount() - 1));
      opBackButton.toggleWhenPressed(new shooterSpoolCommand(m_shooter));

      */

  }

}
