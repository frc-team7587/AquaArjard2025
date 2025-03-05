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
import java.time.Instant;
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
          "Metuchen Momentum", 20000)
        .setForegroundGreen(63)
        .setForegroundRed(63)
        .setDelay1(40)
        .build());
    kMessagesToDisplay.add(
      new MarqueeMessageBuilder(
          "Off the wall!", 20000)
        .setBackgroundRed(31)
        .setForegroundGreen(63)
        .setDelay1(40)
        .build());
    kMessagesToDisplay.add(
      new MarqueeMessageBuilder(
          "Green Alliance", 20000)
        .setForegroundGreen(127)
        .setDelay1(40)
        .build());
      kMessagesToDisplay.add(
        new MarqueeMessageBuilder(
            "Thank you, sponsors ...", 20000)
          .setForegroundRed(42)
          .setForegroundGreen(42)
          .setForegroundBlue(42)
          .setDelay1(40)
          .build());
      kMessagesToDisplay.add(
        new MarqueeMessageBuilder(
          "Boyd, Dewey, Cheetham, and How, Attournies At Law", 20000)
        .setForegroundBlue(127)
        .setDelay1(40)
        .build());
      kMessagesToDisplay.add(
        new MarqueeMessageBuilder(
          "Pickup Andropov Car Service", 20000)
        .setForegroundRed(127)
        .setDelay1(40)
        .build());
      kMessagesToDisplay.add(
        new MarqueeMessageBuilder(
          "Rolls, Canardly Fine Classic Automobiles", 20000)
        .setForegroundBlue(127)
        .setDelay1(40)
        .build());
      kMessagesToDisplay.add(
        new MarqueeMessageBuilder(
          "Orson Buggy Logistics", 20000)
        .setForegroundRed(63)
        .setForegroundBlue(63)
        .setDelay1(40)
        .build());
      kMessagesToDisplay.add(
        new MarqueeMessageBuilder(
          "Dustin Dubree Construction", 20000)
        .setForegroundRed(63)
        .setForegroundGreen(63)
        .setDelay1(40)
        .build());
      kMessagesToDisplay.add(
        new MarqueeMessageBuilder(
          "Horseshoe Road Inn, Guest Accommodations", 20000)
        .setForegroundGreen(63)
        .setForegroundBlue(63)
        .setDelay1(40)
        .build());
      kMessagesToDisplay.add(
        new MarqueeMessageBuilder(
          "Puns (Dis)Courtesy of Cartalk.Com", 20000)
        .setForegroundBlue(127)
        .setDelay1(40)
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
    DataLogManager.log(new StringBuilder("Robot is starting. at ")
        .append(Instant.now())
        .append('.')
        .toString());

    // System.out.print("Robot is starting. at ");
    // System.out.print(Instant.now());
    // System.out.println('.');


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