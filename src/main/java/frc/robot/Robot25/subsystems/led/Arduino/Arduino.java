/*----------------------------------------------------------------------------*/
/* Copyright (c) 2025 Griffin. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

// where is the money whisknousky
package frc.robot.Robot25.subsystems.led.Arduino;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arduino extends SubsystemBase {

  private SerialPort arduino;

  public Arduino() {
    isArduinoConnectedFalse = true;
    try {
      arduino = new SerialPort(9600, SerialPort.Port.kUSB);
      System.out.println("Connected on kUSB!");
      isArduinoConnectedFalse = false;
    } catch (Exception e) {
      System.out.println("Failed to connect on kUSB, trying kUSB 1");

      try {
        arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
        System.out.println("Connected on kUSB1!");
        isArduinoConnectedFalse = false;
      } catch (Exception e1) {
        System.out.println("Failed to connect on kUSB1, trying kUSB 2");

        try {
          arduino = new SerialPort(9600, SerialPort.Port.kUSB2);
          System.out.println("Connected on kUSB2!");
          isArduinoConnectedFalse = false;
        } catch (Exception e2) {
          System.out.println("Failed to connect on kUSB2, all connection attempts failed!");
        }
      }
    }
  }

  private boolean isArduinoConnectedFalse = true;

  public Command runCommand(ArduinoCommand command) {
    return this.runOnce(() -> {
      // System.out.println(Sent to Arduino);
      if (arduino != null) {
        arduino.write(new byte[] { command.getvalue() }, 1);
        if (arduino.getBytesReceived() > 0) {
          System.out.print(arduino.readString());
        }
      } else {
        System.out.println("No Ardunio Detected");
      }
    });
  }

  public boolean isConnected() {
    if (isArduinoConnectedFalse == true) {
      return false;
    } else {
      return true;
    }
  }

  public byte byteWizard(int value) {
    if (value > 255) {
      value = 255;
    }
    if (value < 0) {
      value = 0;
    }
    return (byte) value;
  }

  // CONFIG
  public byte[] colors;
  public int byteCount = 4;

  // FUNCTION
  public void customColor(int R, int G, int B) {
    colors = new byte[] { 0x30, byteWizard(R), byteWizard(G), byteWizard(B) };
    if (arduino != null) {
      arduino.write(colors, byteCount);
      if (arduino.getBytesReceived() > 0) {
        System.out.print(arduino.readString());
      }
    } else {
      System.out.println("No Ardunio Detected");
    }
  }
}
