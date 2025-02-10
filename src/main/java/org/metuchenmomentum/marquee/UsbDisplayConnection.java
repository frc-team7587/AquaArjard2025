package org.metuchenmomentum.marquee;

import edu.wpi.first.wpilibj.SerialPort;
import java.nio.file.Path;

/**
 * A {@link DeviceConnection} that sends marquee commands over a USB
 * serial connection.
 */
public class UsbDisplayConnection implements DisplayConnection {

    private SerialPort serialPort;

    private UsbDisplayConnection(SerialPort serialPort) {
        this.serialPort = serialPort;
    }
    
    /**
     * Sends the specified {@code message} to the ESP32 over a serial
     * USB connection.
     * 
     * @param message message to send
     * 
     * @return the number of bytes sent, which should be in the range
     *   [0 .. 127]
     */
    @Override
    public int send(DisplayMessage message) {
        try {
            return serialPort.writeString(message.toString());
        } catch (Exception e) {
            System.out.println(
                "Exception while sending message to the Marquee.\n" + e);
            return 0;
        }
    }

    /**
     * Returns a {@link Builder} that will create a {@link UsbDisplayConnection}
     * that sends display commands to a specified serial USB device.
     * 
     * @param usbDevicePath the serial USB device path
     * 
     * @return the {@link Builder}, as described above
     */
    public static Builder builder(Path usbDevicePath) {
        return new Builder(usbDevicePath);
    }

    /**
     * Builder for {@link UsbDisplayConnection} instances
     */
    public static class Builder {
        private final Path usbDevicePath;

        private Builder(Path usbDevicePath) {
            this.usbDevicePath = usbDevicePath;
        }

        /**
         * Builds a {@link UsbDisplayConnection} from the current settings.
         * 
         * @return the newly created {@link UsbDisplayConnection} as described above.
         */
        public UsbDisplayConnection build() {
            UsbDisplayConnection built = null;

            SerialPort usbPort = new UsbSerialPortBuilder(usbDevicePath).build();
            if (usbPort != null) {
                built = new UsbDisplayConnection(usbPort);
            } else {
                System.out.println("SerialPort creation failed.");
            }

            return built;
        }
    }
}
