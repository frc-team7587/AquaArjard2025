package frc.robot.subsystems.Marquee.MarqueeLib;

/**
 * Message sending API.
 */
public interface DisplayConnection {
    /**
     * Sends the specified {@code message} to the ESP32
     * 
     * @param message message to send. The message's {@code toString()}
     *     method must return a string whose length is less than 128
     *     bytes when serialized to a byte array.
     * 
     * @return the number of bytes sent. Should be between 0
     *     and 128, inclusive. Implementations that send fixed
     *     size blocks must send 128 bytes.
     */
    int send(DisplayMessage message);
}
