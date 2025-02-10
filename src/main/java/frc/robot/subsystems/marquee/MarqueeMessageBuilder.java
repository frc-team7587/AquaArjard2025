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
        return new MarqueeMessage(displayMessage, mDisplayTimeMS);
    }

    public MarqueeMessageBuilder setBackgroundRed(int mBackgroundRed) {
        this.mBackgroundRed = mBackgroundRed;
        return this;
    }

    public MarqueeMessageBuilder setBackgroundGreen(int mBackgroundGreen) {
        this.mBackgroundGreen = mBackgroundGreen;
        return this;
    }

    public MarqueeMessageBuilder setBackgroundBlue(int mBackgroundBlue) {
        this.mBackgroundBlue = mBackgroundBlue;
        return this;
    }

    public MarqueeMessageBuilder setForegroundRed(int mForegroundRed) {
        this.mForegroundRed = mForegroundRed;
        return this;
    }

    public MarqueeMessageBuilder setForegroundGreen(int mForegroundGreen) {
        this.mForegroundGreen = mForegroundGreen;
        return this;
    }

    public MarqueeMessageBuilder setForegroundBlue(int mForegroundBlue) {
        this.mForegroundBlue = mForegroundBlue;
        return this;
    }

    public MarqueeMessageBuilder setDelay1(int mDelay1) {
        this.mDelay1 = mDelay1;
        return this;
    }

    public MarqueeMessageBuilder setDelay2(int mDelay2) {
        this.mDelay2 = mDelay2;
        return this;
    }
}