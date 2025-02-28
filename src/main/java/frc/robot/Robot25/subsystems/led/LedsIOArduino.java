package frc.robot.Robot25.subsystems.led;

import frc.robot.Robot25.subsystems.led.Arduino.Arduino;

public class LedsIOArduino implements LedsIO {

  private Arduino arduino;

  public LedsIOArduino() {
    arduino = new Arduino();
  }

  public void updateInputs(LedsIOInputs inputs) {}

  public void setColor(Color color) {
    arduino.customColor(color.red, color.green, color.blue);
  }
}
