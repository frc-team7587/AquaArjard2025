package frc.robot.subsystems.marquee;

import java.util.ArrayList;
import java.util.List;

import org.metuchenmomentum.marquee.DisplayCommand;
import org.metuchenmomentum.marquee.DisplayConnection;
import org.metuchenmomentum.marquee.DisplayConnectionFactory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Marquee management subsystem that repeatedly 
 * displays a message sequence. Users can also
 * break the predetermined display sequence to display
 * game-related notifications.
 * 
 * Note: the design assumes that event handlers (i.e. 
 * {@link #periodic}) and explicitly invoked methods like
 * {@link #displayNotification} run on the same thread.
 * If they run on different threads, synchronize both
 * methods.
 */
public final class MarqueeSubsystem extends SubsystemBase {
    
    private static final MarqueeMessage FILL_WITH_BLACK = new MarqueeMessageBuilder(null, 2000)
    .setDisplayCommand(DisplayCommand.FILL_WITH_COLOR)
    .build();

    private static final List<MarqueeMessage> POWER_ON_MESSAGES;

    /**
     * Messages to display on startup. Note that the RoboRio needs time
     * to connect with the ESP32, so we send a bunch of dummy commands
     * that fill the screen with black before we send anything meaningful.
     */
    static {
        POWER_ON_MESSAGES = new ArrayList<>();
        POWER_ON_MESSAGES.add(FILL_WITH_BLACK);
        // POWER_ON_MESSAGES.add(FILL_WITH_BLACK);
        // POWER_ON_MESSAGES.add(FILL_WITH_BLACK);
        // POWER_ON_MESSAGES.add(FILL_WITH_BLACK);
        // POWER_ON_MESSAGES.add(FILL_WITH_BLACK);
        // POWER_ON_MESSAGES.add(FILL_WITH_BLACK);
        POWER_ON_MESSAGES.add(FILL_WITH_BLACK);
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder(null, 5000)
            .setDisplayCommand(DisplayCommand.RIPPLING_RAINBOW)
            .build());
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("Red", 1000)
            .setBackgroundRed(63)
            .setDisplayCommand(DisplayCommand.STATIC_TEXT)
            .build());
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("Green", 1000)
            .setBackgroundGreen(63)
            .setDisplayCommand(DisplayCommand.STATIC_TEXT)
            .build());
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("Blue", 1000)
            .setBackgroundBlue(63)
            .setDisplayCommand(DisplayCommand.STATIC_TEXT)
            .build());
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("White", 1000)
            .setBackgroundRed(21)
            .setBackgroundGreen(21)
            .setBackgroundBlue(21)
            .setDisplayCommand(DisplayCommand.STATIC_TEXT)
            .build());

        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("Red", 1000)
            .setForegroundRed(63)
            .setDisplayCommand(DisplayCommand.STATIC_TEXT)
            .build());
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("Green", 1000)
            .setForegroundGreen(63)
            .setDisplayCommand(DisplayCommand.STATIC_TEXT)
            .build());
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("Blue", 1000)
            .setForegroundBlue(63)
            .setDisplayCommand(DisplayCommand.STATIC_TEXT)
            .build());
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("White", 1000)
            .setForegroundRed(21)
            .setForegroundGreen(21)
            .setForegroundBlue(21)
            .setDisplayCommand(DisplayCommand.STATIC_TEXT)
            .build());
        
        POWER_ON_MESSAGES.add(new MarqueeMessageBuilder("Starting ...", 5000)
            .setForegroundRed(21)
            .setForegroundGreen(21)
            .setForegroundBlue(21)
            .setDelay1(40)
            .build());
    }
    

    private final DisplayConnection mDisplayConnection;
    private final List<MarqueeMessage> mMessages;
    private final int mMillisecondsPerTick;

    private List<MarqueeMessage> mActiveMessages;
    private int mTimeDisplayedMS;
    private int mWhenToAdvanceMS;
    private int mMessageIndex;
    private int mIterationCount;

    /**
     * Convenience factory method: creates a {@link MarqueeSubsystem}
     * communicates with the marquee via a serial USB serial
     * connection.
     * 
     * @param messages sequence of messages to display
     * @param millisecondsPerTick interval between {@link #periodic()}
     *        invocations.
     * @return the newly minted {@link MarqueeSubsystem}
     */
    public static MarqueeSubsystem usbConnection(
        List<MarqueeMessage> messages,
        int millisecondsPerTick) {
            System.out.println("Connecting to the ESP32 via ttyUSBn.");
            DisplayConnection marqueeConnection = DisplayConnectionFactory.usbConnection();
            System.out.println(
                "Created marquee connection type: " + marqueeConnection.getClass().getCanonicalName());
            return new MarqueeSubsystem(
               marqueeConnection,
                messages,
                millisecondsPerTick);
        }

    /**
     * Constructor
     * 
     * @param displayConnection connection to the marquee
     * @param messages sequence of messages to display
     * @param millisecondsPerTick interval between {@link #periodic()}
     *        invocations.
     */
    public MarqueeSubsystem(
        DisplayConnection displayConnection,
        List<MarqueeMessage> messages,
        int millisecondsPerTick) {
            mDisplayConnection = displayConnection;
            mMillisecondsPerTick = millisecondsPerTick;
            mMessages = messages;
            mActiveMessages = POWER_ON_MESSAGES;
            mTimeDisplayedMS = 0;
            mWhenToAdvanceMS = 0;
            mMessageIndex = 0;
            mIterationCount = 0;
    }

    /**
     * Displays the next message in the list if the current message
     * has expirec. If the message list has been exhausted, reset to its
     * first entry.
     */
    @Override
    public void periodic() {
        mTimeDisplayedMS += mMillisecondsPerTick;
        ++mIterationCount;
        if (mWhenToAdvanceMS <= mTimeDisplayedMS) {
            // System.out.println("Display expired at ");
            // System.out.print(mTimeDisplayedMS);
            // System.out.print(" milliseconds after ");
            // System.out.print(mIterationCount);
            // System.out.println(" iterations.");
            mTimeDisplayedMS = 0;
            mIterationCount = 0;
            if (mActiveMessages.size() <= mMessageIndex) {
                mMessageIndex = 0;
                mActiveMessages = mMessages;
            }
            MarqueeMessage showMe = mActiveMessages.get(mMessageIndex);
            // System.out.print("Showing message number ");
            // System.out.print(mMessageIndex);
            // System.out.print(", \"");
            // System.out.print(showMe.displayMessage().toString());
            // System.out.println("\"");
            showMessage(showMe);
            mWhenToAdvanceMS = showMe.durationMs();
            // System.out.print("Display duration: ");
            // System.out.println(mWhenToAdvanceMS);
            ++mMessageIndex;
        }
    }

    /**
     * Preempt the currently displayed message with the provided
     * notification. The display will advance normally when the
     * provided message display expires.
     * 
     * Invoke this method to display a game-related notification,
     * e.g. "OUCH OUCH OUCH!" when the robot is bumped.
     * 
     * @param notification the notification to display.
     */
    public void displayNotification(MarqueeMessage notification) {
        showMessage(notification);
    }

    /**
     * Sends the provided message to the Marquee for display, and
     * set its expiration time.
     * 
     * @param message the message to display.
     */
    private void showMessage(MarqueeMessage message) {
        mTimeDisplayedMS = 0;
        mWhenToAdvanceMS = message.durationMs();
        mDisplayConnection.send(message.displayMessage());
    }
}
