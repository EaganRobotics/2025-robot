package frc.lib;
/* https://stackoverflow.com/questions/604424/how-to-get-an-enum-value-from-a-string-value-in-java */

/* https://www.tutorialspoint.com/java-program-to-get-system-mac-address-of-windows-and-linux-machine */

import edu.wpi.first.wpilibj.RobotBase;
// import frc.lib.faults.Fault;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.function.Function;

public enum RobotInstance {
  Robot24("00-80-2F-36-FE-34"),
  KrackenSwerve("00-80-2F-17-F8-19"),
  BoxyBot("00-80-2F-36-FD-D6"),
  Simulator("Simulator");

  private String address;
  private static RobotInstance current = getMacAddress();
  // private static Fault fault;

  RobotInstance(String text) {
    this.address = text;
  }

  public String getText() {
    return this.address;
  }

  public static RobotInstance fromString(String text) {
    for (RobotInstance b : RobotInstance.values()) {
      if (b.address.equalsIgnoreCase(text)) {
        return b;
      }
    }
    return null;
  }

  public static String getMacAddressStr() {
    if (RobotBase.isSimulation()) {
      return Simulator.address;
    }

    InetAddress address;
    try {
      address = InetAddress.getLocalHost();
      NetworkInterface networkInterface = NetworkInterface.getByInetAddress(address);
      byte[] mac = networkInterface.getHardwareAddress();
      System.out.print("MAC address : ");
      StringBuilder stringBuilder = new StringBuilder();
      for (int i = 0; i < mac.length; i++) {
        stringBuilder.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
      }
      String result = stringBuilder.toString();
      System.out.println(result);
      return result;
    } catch (Exception e) {
      e.printStackTrace();
      return "Error:" + e.toString();
    }
  }

  public static RobotInstance getMacAddress() {
    var check = fromString(getMacAddressStr());
    if (check == null) {
      // if (fault == null) {
      //     fault = new Fault("Unknown Robot MAC Address: " + getMacAddressStr());
      //     fault.setIsActive(true);
      // }
      System.err.println("no mac address found");
      throw new RuntimeException("Unknown Robot MAC Address: " + getMacAddressStr());
    } else {
      System.out.println("Current robot: " + check.toString());
      return check;
    }
  }

  public static <T extends RobotContainer> T config(Function<RobotInstance, T> config) {
    return config.apply(current);
  }
}
