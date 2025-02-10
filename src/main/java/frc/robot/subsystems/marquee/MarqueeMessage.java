package frc.robot.subsystems.marquee;

import org.metuchenmomentum.marquee.DisplayMessage;

/**
 * Holds a <@link DisplayMessage> and its display time in milliseconds.
 */

public record MarqueeMessage(DisplayMessage displayMessage, int durationMs) {}