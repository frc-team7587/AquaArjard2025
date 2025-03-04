package org.metuchenmomentum.marquee;

import java.nio.charset.StandardCharsets;

/**
 * A message to send to the display controller. The message is an
 * array of bytes containing a pipe-delimited command string having
 * the following fields:
 * 
 * <pre>
 * 
 * | Field No. | Contents                                           |
 * | --------- | -------------------------------------------------- |
 * |         0 | The text to display, if any                        |
 * |         1 | The display command                                |
 * |         3 | Delay 1                                            |
 * |         4 | Delay 2                                            |
 * |         5 | Foreground red intensity                           |
 * |         6 | Foreground green intensity                         |
 * |         7 | Foreground blue intensity                          |
 * |         8 | Background red intensity                           |
 * |         9 | Background green intesity                          |
 * |        10 | Background blue intensity                          |
 * 
 * </pre>
 * 
 * Commands MUST end with a new-line (\n'). For Windows
 * compatibility, a carriage return ('\r') MAY appear before
 * the terminating new-line.
 * 
 * Display commands are specified by the DisplayCommand enum defined
 * in DisplayCommand.java.
 * 
 * Intensity values must be non-negative and less than or equal to
 * 255.
 * 
 * Delays must be non-negative.
 * 
 * Note that delay semantics vary among commands, and may be ignored
 * entirely.
 * 
 * @see DisplayCommand
 */
public final class DisplayMessage {

    private static String ERROR_STRING = 
        new DisplayMessage()
        .setDisplayCommand(DisplayCommand.ERROR)
        .toString();

    private String displayText;
    private DisplayCommand displayCommand;
    private int foregroundBlue;
    private int foregroundGreen;
    private int foregroundRed;
    private int backgroundBlue;
    private int backgroundGreen;
    private int backgroundRed;
    private int delay1;
    private int delay2;
    private State state;

    private enum State {
        VALID,
        INVALID,
    }
  
   private int checkColor(int value, String setting) {
    if (value < 0 || value > 255) {
        state = State.INVALID;
        System.out.println(
            new StringBuilder(
                "Color values must be between 0 and 255 but the value for ")
                .append(setting)
                .append(" is: ")
                .append(value)
                .append('.')
                .toString());
    }
    return value;
  }

  /**
   * Constructs an instance that fills the panel with black, that is,
   * turns the panel completely off. Users should invoke setters
   * to configure the message.
   */
   public DisplayMessage() {
    displayText = "";
    displayCommand = DisplayCommand.FILL_WITH_COLOR;
    foregroundBlue = 0;
    foregroundGreen = 0;
    foregroundRed = 0;
    backgroundBlue = 0;
    backgroundGreen = 0;
    backgroundRed = 0;
    delay1 = 0;
    delay2 = 0;

    state = State.VALID;
   }
 
   public DisplayMessage setText(String text) {
    displayText = text == null ? "" : text;
    return this;
   }

   public DisplayMessage setDisplayCommand(DisplayCommand command) {
    displayCommand = command;
    return this;
   }

   public DisplayMessage setForegroundBlue(int value) {
    foregroundBlue = checkColor(value, "foreground blue");
    return this;
   }

   public DisplayMessage setForegroundGreen(int value) {
    foregroundGreen = checkColor(value, "foreground green");
    return this;
   }

   public DisplayMessage setForegroundRed(int value) {
    foregroundRed = checkColor(value, "foreground red");
    return this;
   }

   public DisplayMessage setBackgroundBlue(int value) {
    backgroundBlue = checkColor(value, "background blue");
    return this;
   }

   public DisplayMessage setBackgroundGreen(int value) {
    backgroundGreen = checkColor(value, "background green");
    return this;
   }

   public DisplayMessage setBackgroundRed(int value) {
    backgroundRed = checkColor(value, "background red");
    return this;
   }

   public DisplayMessage setDelay1(int delay) {
    if (delay < 0) {
        System.out.println(
            "Delays must be non-negative but delay 1 is " 
            + delay
            + '.');
    state = State.INVALID;
    }
    delay1 = delay;
    return this;
   }

   public DisplayMessage setDelay2(int delay) {
    if (delay < 0) {
        state = State.INVALID;
        System.out.println(
            "Delays must be non-negative but delay 2 is " 
            + delay
            + '.');
    }
    delay2 = delay;
    return this;
   }

   @Override
   public String toString() {
    String result = null;
    switch (state) {
        case VALID:
            result = new StringBuilder(
                displayText.isEmpty() ? " " : displayText)
                .append('|')
                .append(displayCommand.ordinal())
                .append('|')
                .append(delay1)
                .append('|')
                .append(delay2)
                .append('|')
                .append(foregroundRed)
                .append('|')
                .append(foregroundGreen)
                .append('|')
                .append(foregroundBlue)
                .append('|')
                .append(backgroundRed)
                .append('|')
                .append(backgroundGreen)
                .append('|')
                .append(backgroundBlue)
                .append('\n')
                .toString();

            if (result.length() > 128) {
                result = ERROR_STRING;
            }
            break;
        case INVALID:
            result = ERROR_STRING;
            break;
    }
    return result;
   }

   public byte[] toByteArray() {
    byte[] result = new byte[128];
    byte[] serializedCommand = 
        toString().getBytes(StandardCharsets.US_ASCII);
    System.arraycopy(
        serializedCommand,
        0,
        result,
        0,
        serializedCommand.length);

    return result;
   }
}
