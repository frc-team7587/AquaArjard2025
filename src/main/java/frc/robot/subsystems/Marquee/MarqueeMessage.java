package frc.robot.subsystems.Marquee;

import frc.robot.subsystems.Marquee.MarqueeLib.DisplayMessage;

/**
 * Holds a <@link DisplayMessage> and its display time in milliseconds.
 */

public record MarqueeMessage(DisplayMessage displayMessage, int durationMs) {}