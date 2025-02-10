package org.metuchenmomentum.marquee;

import java.io.IOException;
import java.nio.file.DirectoryIteratorException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Maintains a list of USB serial devices, file {@link Path Paths} of
 * the form {@code \dev\USB*}. Typically, {@code *} is one or more decimal
 * digits, but all matching paths are included.
 */
public class UsbSerialDeviceList {
    final List<Path> devices;

    /**
     * Creates a new UsbSerialDeviceList whose device list is empty. The
     * Start() method fills the list.
     */
    public UsbSerialDeviceList() {
        devices = new ArrayList<>();
    }

    /**
     * Call at program start to search the /dev directory for USB serial devices. These
     * devices are named {@code USBn}, where {@code n} is a single digit. The first
     * device is named USB0, the second USB1, and so forth. 
     * 
     * <p>This method is idempotant under normal operating conditions. If the
     * first invocation returns {@code true}, further invocations have no
     * effect. If the first invocation returns {@code false}, the method
     * will attempt to fill the list again.
     *
     * @return {@code true} if and only if initialization succeeded.
     */
    public boolean start() {
        boolean result = true;
        FileSystem roboRioFileSystem = FileSystems.getDefault();
        Path devicePath = roboRioFileSystem.getPath("/dev");
        try (DirectoryStream<Path> stream =
            Files.newDirectoryStream(devicePath, "ttyUSB*")) {
                for (Path usbSerialDevice : stream) {
                    devices.add(usbSerialDevice);
                }

        } catch (DirectoryIteratorException | IOException e) {
            System.out.println("USB serial device search failed: " + e.getCause());
            result = false;
        }

        return result;
    }

    /**
     * 
     * @return the known USB serial devices, a copy of the device list.
     */
    public List<Path> getDevices() {
        return new ArrayList<>(devices);
    }
}
