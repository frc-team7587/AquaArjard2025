package frc.robot.subsystems.Marquee.MarqueeLib;

import java.nio.file.Path;
import java.util.List;

/**
 * A factory that builds {@link DeviceConnection} instances.
 */
public class DisplayConnectionFactory {
    /**
     * Connect to the Marquee over a serial USB cable. For the connection to
     * succeed, there must be one and only one USB serial device connected
     * to the RoboRio. Note that serial devices have paths of the form
     * {@code /dev/ttyUSBn}, where {@code n} is 0, 1, or 2.
     * 
     * @return a {@link UsbDisplayConnection} if it was possible to connect,
     *         a {@link VacuousDisplayConnection} otherwise.
     */
    public static DisplayConnection usbConnection() {
        List<Path> usbSerialConnections = findSerialConnections();
        return connectionFromUsbPaths(usbSerialConnections);
    }

    /**
     * Construct a {@link DisplayConnection} from a list of available serial USB devices.
     *
     * @param usbDevicePaths a {@link List} of available serial USB devices as {@link Path}
     *     instances.
     * 
     * @return a {@link DeviceConnection} implementation. If {usbDevicePaths} contains
     *     a single {@link Path} and connection succeeds, then the invocation will
     *     return a {@link UsbDisplayConnection}. Otherwise it will return a
     *     {@link VacuousDisplayConnection}.
     */
    private static DisplayConnection connectionFromUsbPaths(List<Path> usbDevicePaths) {
        DisplayConnection connection = null;
        switch(usbDevicePaths.size()) {
            case 0:
                System.out.println("Marquee not connected, no serial USB devices found.");
                break;
            case 1:
                System.out.println("Found one USB Serial device; opening connection");
               connection =  UsbDisplayConnection.builder(usbDevicePaths
                    .get(0))
                    .build();
                break;
            default:
                System.out.println("Multiple serial devices found, cannot connect.");
                break;
        }


        if (connection == null) {
            connection = new VacuousDisplayConnection();
        }

        return connection;
    }

    /**
     * Scans the /dev directory for USB serial device paths.
     * 
     * @return a {@link List} containing the {@link Path} of each available device
     */
    private static List<Path> findSerialConnections() {
        UsbSerialDeviceList usbSerialDevices = new UsbSerialDeviceList();
        boolean deviceListStatus = usbSerialDevices.start();
        System.out.println("USB serial device query returns: " + deviceListStatus);
        System.out.println("Devices:");
        List<Path> usbDevicePaths = usbSerialDevices.getDevices();
        for (Path device : usbDevicePaths) {
            System.out.println("    " + device);
        }
        return usbDevicePaths;
    }
}
