package frc.robot.subsystems.marquee;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.metuchenmomentum.marquee.DisplayConnection;
import org.metuchenmomentum.marquee.DisplayConnectionFactory;

import java.util.List;

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
    
    private final DisplayConnection mDisplayConnection;
    private final List<MarqueeMessage> mMessages;
    private final int mMillisecondsPerTick;

    private int mTimeDisplayedMS;
    private int mWhenToAdvanceMS;
    private int mMessageIndex;

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
            return new MarqueeSubsystem(
                DisplayConnectionFactory.usbConnection(),
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
        mTimeDisplayedMS = 0;
        mWhenToAdvanceMS = 0;
        mMessageIndex = 0;
    }

    /**
     * Displays the next message in the list if the current message
     * has expirec. If the message list has been exhausted, reset to its
     * first entry.
     */
    @Override
    public void periodic() {
        mTimeDisplayedMS += mMillisecondsPerTick;
        if (mWhenToAdvanceMS <= mTimeDisplayedMS) {
            if (mMessages.size() <= mMessageIndex) {
                mMessageIndex = 0;
            }
            showMessage(mMessages.get(mMessageIndex));
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
        mTimeDisplayedMS = message.durationMs();
        mDisplayConnection.send(message.displayMessage());
    }
}
