// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.marquee.MarqueeMessage;
import frc.robot.subsystems.marquee.MarqueeMessageBuilder;
import frc.robot.subsystems.marquee.MarqueeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.ArrayList;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /**
   * Messages to display on the marquee.
   */
  private static final ArrayList<MarqueeMessage> kMessagesToDisplay;

  static {
    kMessagesToDisplay = new ArrayList<>();
    kMessagesToDisplay.add(
      new MarqueeMessageBuilder(
          "Metuchen Momentum", 100000)
        .setForegroundGreen(63)
        .setForegroundRed(63)
        .setDelay1(30)
        .build());
    kMessagesToDisplay.add(
      new MarqueeMessageBuilder(
          "Off the wall!", 1000000)
        .setBackgroundRed(31)
        .setForegroundGreen(63)
        .setDelay1(30)
        .build());
    kMessagesToDisplay.add(
      new MarqueeMessageBuilder(
          "Green Alliance", 100000)
        .setForegroundGreen(127)
        .setDelay1(30)
        .build());
  }

  /**
   * Subsystem that manages the marquee.
   */
  private final MarqueeSubsystem m_MarqueeSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Redirect console output to the log
    DataLogManager.start();
    DataLogManager.logConsoleOutput(true);


    System.out.println("Creating the marquee subsystem.");
    m_MarqueeSubsystem = MarqueeSubsystem.usbConnection(
        kMessagesToDisplay, 20);
    System.out.println("Marquee subsystem created");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return null;
  }
}