package frc.robot.subsystems.marquee;

import org.metuchenmomentum.marquee.DisplayCommand;
import org.metuchenmomentum.marquee.DisplayMessage;

public class MarqueeMessageBuilder {

    private int mBackgroundRed;
    private int mBackgroundGreen;
    private int mBackgroundBlue;

    private int mForegroundRed;
    private int mForegroundGreen;
    private int mForegroundBlue;

    private final DisplayCommand mDisplayCommand;
    private final String mText;
    private final int mDisplayTimeMS;

    private int mDelay1;
    private int mDelay2;

    public MarqueeMessageBuilder(String text, int displayTimeMS) {
        mDisplayCommand = DisplayCommand.TEXT_CRAWL;
        mText = text;
        mDisplayTimeMS = displayTimeMS;

        mBackgroundRed = 0;
        mBackgroundGreen = 0;
        mBackgroundBlue = 0;

        mForegroundRed = 0;
        mForegroundGreen = 0;
        mForegroundBlue = 0;

        mDelay1 = 0;
        mDelay2 = 0;
    }

    public MarqueeMessage build() {
        DisplayMessage displayMessage = new DisplayMessage()
            .setBackgroundBlue(mBackgroundBlue)
            .setBackgroundGreen(mBackgroundGreen)
            .setBackgroundRed(mBackgroundRed)

            .setForegroundBlue(mForegroundBlue)
            .setForegroundGreen(mForegroundGreen)
            .setForegroundRed(mForegroundRed)

            .setDisplayCommand(mDisplayCommand)
            .setText(mText)
            .setDelay1(mDelay1)
            .setDelay2(mDelay2);
        var marqueeMessage = new MarqueeMessage(displayMessage, mDisplayTimeMS);
        System.out.print("Created message: ");
        System.out.println(marqueeMessage.toString());
        return marqueeMessage;
    }

    public MarqueeMessageBuilder setBackgroundRed(int backgroundRed) {
        this.mBackgroundRed = backgroundRed;
        return this;
    }

    public MarqueeMessageBuilder setBackgroundGreen(int backgroundGreen) {
        this.mBackgroundGreen = backgroundGreen;
        return this;
    }

    public MarqueeMessageBuilder setBackgroundBlue(int backgroundBlue) {
        this.mBackgroundBlue = backgroundBlue;
        return this;
    }

    public MarqueeMessageBuilder setForegroundRed(int foregroundRed) {
        this.mForegroundRed = foregroundRed;
        return this;
    }

    public MarqueeMessageBuilder setForegroundGreen(int foregroundGreen) {
        this.mForegroundGreen = foregroundGreen;
        return this;
    }

    public MarqueeMessageBuilder setForegroundBlue(int foregroundBlue) {
        this.mForegroundBlue = foregroundBlue;
        return this;
    }

    public MarqueeMessageBuilder setDelay1(int delay1) {
        this.mDelay1 = delay1;
        return this;
    }

    public MarqueeMessageBuilder setDelay2(int delay2) {
        this.mDelay2 = delay2;
        return this;
    }
}