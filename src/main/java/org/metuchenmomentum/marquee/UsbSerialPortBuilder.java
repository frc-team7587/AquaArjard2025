package org.metuchenmomentum.marquee;

import edu.wpi.first.wpilibj.SerialPort;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.util.HashMap;

/**
 * Builds a {@link SerialPort} from its file path and communication
 * settings. The default communication settings are 115200 bits/second,
 * 8 data bits, one stop bit, no parity, no flow control, and flush on
 * write, The default values support the marquee, but can be changed
 * to support future use cases.
 * 
 * <p>The factory only supports serial USB devices. 
 */
public class UsbSerialPortBuilder {
    static final HashMap<Path, SerialPort.Port> USB_PATH_TO_SERIAL_PORT;

    static {
        USB_PATH_TO_SERIAL_PORT = new HashMap<>();
        FileSystem defaultFileSystem = FileSystems.getDefault();
        USB_PATH_TO_SERIAL_PORT.put(
            defaultFileSystem.getPath("/dev", "ttyUSB0"), SerialPort.Port.kUSB);
        USB_PATH_TO_SERIAL_PORT.put(
            defaultFileSystem.getPath("/dev", "ttyUSB1"), SerialPort.Port.kUSB1);
        USB_PATH_TO_SERIAL_PORT.put(
            defaultFileSystem.getPath("/dev", "ttyUSB2"), SerialPort.Port.kUSB2);
    }

    private int baudRate;
    private int dataBits;
    private SerialPort.FlowControl flowControl;
    private SerialPort.Parity parity;
    private SerialPort.StopBits stopBits;
    private SerialPort.WriteBufferMode writeBufferMode;
    private final Path devicePath;

    /**
     * Creates a {@code UsbSerialPathBuilder} for the specified device path and
     * default communication settings
     * 
     * @param devicePath the serial USB device {@link Path}. See
     *   {@code USB_PATH_TO_SERIAL_PORT} for a list of supported device
     *   paths. 
     */
    public UsbSerialPortBuilder(Path devicePath) {
        baudRate = 115200;
        dataBits = 8;
        flowControl = SerialPort.FlowControl.kNone;
        parity = SerialPort.Parity.kNone;
        stopBits = SerialPort.StopBits.kOne;
        writeBufferMode = SerialPort.WriteBufferMode.kFlushOnAccess;
        this.devicePath = devicePath;
    }

    /**
     * Builds a {@link SerialPort} from the current settings. 
     * 
     * @return the newly created and configured {@link SerialPort} if the settings
     *     valud, {@code null} otherwise
     */
    public SerialPort build() {
        SerialPort built = null;
        if (devicePath != null) {
            SerialPort.Port usbPort = USB_PATH_TO_SERIAL_PORT.get(devicePath);
            if (usbPort != null) {
                built = new SerialPort(
                    baudRate,
                    usbPort,
                    dataBits,
                    parity,
                    stopBits);
                built.setFlowControl(flowControl);
                built.setWriteBufferMode(writeBufferMode);
            }
        }

        return built;
    }

    /************************************************************************\
    |  Setters return this to support chaining.                              |
    \************************************************************************/

     public UsbSerialPortBuilder setBaudRate(int baudRate) {
        this.baudRate = baudRate;
        return this;
     }

     public UsbSerialPortBuilder setDataBits(int dataBits) {
        this.dataBits = dataBits;
        return this;
     }

     public UsbSerialPortBuilder setFlowControl(SerialPort.FlowControl flowControl) {
        this.flowControl = flowControl;
        return this;
     }

     public UsbSerialPortBuilder setParity(SerialPort.Parity parity) {
        this.parity = parity;
        return this;
     }

     public UsbSerialPortBuilder setWriteBufferMode(SerialPort.WriteBufferMode writeBufferMode) {

        this.writeBufferMode = writeBufferMode;
        return this;
     }
}
