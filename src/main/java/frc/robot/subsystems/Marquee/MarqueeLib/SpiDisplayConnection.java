package frc.robot.subsystems.Marquee.MarqueeLib;

import edu.wpi.first.wpilibj.SPI;

/**
 * Sends commands to a marquee display via SPI.
 */
public class SpiDisplayConnection implements DisplayConnection {
    private final SPI displayConnection;

    /**
     * {@link DisplayConnection} implementation that sends message
     * over an
     * <a href='https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/SPI.html'>SPI</a>
     * link
     * 
     * @param port SPI port, the RoboRio SPI pin to connect to the ESP32's Slave Select
     * GPIO pin. 
     */
    public SpiDisplayConnection(SPI.Port port) {
        displayConnection = new SPI(port);
        displayConnection.setMode(SPI.Mode.kMode0);
        displayConnection.setClockRate(10000);
    }

    @Override
    public int send(DisplayMessage message) {
        byte[] messageToSend = message.toByteArray();
        int messageSize = messageToSend.length;
        if (messageSize != 128) {
            throw new IllegalStateException(
                "Message send length should be 128, is " + messageSize);
        }
        // Note: even though the message size parameter in
        // SPI.write() is an int, you can send AT MOST
        // 127 bytes, so we have to hack around that mumble mumble ...
        return displayConnection.write(messageToSend, 127);
    }
}
