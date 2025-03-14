
package frc.robot.subsystems.Marquee.MarqueeLib;

/**
 * Specifies the actions that the display controller can perform. This
 * file must be kept in sync with DisplayCommand.h in the marquee
 * code.
 */
public enum DisplayCommand {
    ERROR,  // Dusplayed on invalid command or other problem
    FILL_WITH_COLOR,  // Flood the display with a single color
    FLASH_TWO_COLORS,  // TODO -- not implemented
    RIPPLING_RAINBOW,  // Rainbow pattern scrolling from right to left
    SINGLE_PIXEL_NATURAL_ORDER,  // A single pixlel in a zigzag pattern
    SINGLE_PIXEL_LEFT_TO_RIGHT,  // Single pixel left to right, bottom to top
    STATIC_TEXT,  // Non-moving text, limited to five characters
    SCROLLING_TEXT,  // Text that displays, then crawls to the left
    TEXT_CRAWL,  // Full text crawl, right to left
    NUMBER_OF_COMMANDS,   // MUST BE LAST!
}
