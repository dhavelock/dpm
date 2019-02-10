package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.Navigator.DistanceController;
import ca.mcgill.ecse211.Navigator.UltrasonicPoller;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Main {
  
  private static final Port usPort = LocalEV3.get().getPort("S2");
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  public static final double WHEEL_RAD = 2.2;     // Measured and tweaked Wheel radius
  public static final double TRACK = 13.2;      // Measured and tweaked Track length
  
  public static void main(String[] args) throws OdometerExceptions {
    
    int buttonChoice;
      
    // Initialize odometer
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
    
    // Initialize distance controller
    DistanceController dc = new DistanceController();
    
    // Initialize light sensor
    LightLocalizer lightLocalizer = new LightLocalizer();
    
    do {
      // clear the display
      lcd.clear();
  
      // ask the user whether the motors should use obstacle avoidance or not
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Falling| Rising ", 0, 2);
      lcd.drawString(" Edge  |  Edge  ", 0, 3);
      lcd.drawString("       |        ", 0, 4);
  
      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_DOWN && buttonChoice != Button.ID_RIGHT);
  
    //Left button initiates navigation with obstacle avoidance
    if (buttonChoice == Button.ID_LEFT) {
      
      // US Poller 
      @SuppressWarnings("resource") // Because we don't bother to close this resource
      SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
      SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                                // this instance
      float[] usData = new float[usDistance.sampleSize()]; 
      
      // Initialize localization
      UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(0, dc);
      
      // initialize and start threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, dc);
      usPoller.start();
      Thread localizeThread = new Thread(usLocalizer);
      localizeThread.start();
      
      Button.waitForAnyPress();
      
      Thread lightLocalizeThread = new Thread(lightLocalizer);
      lightLocalizeThread.start();
      
    }
    
    if (buttonChoice == Button.ID_RIGHT) {
      
      // US Poller 
      @SuppressWarnings("resource") // Because we don't bother to close this resource
      SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
      SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                                // this instance
      float[] usData = new float[usDistance.sampleSize()]; 
      
      // Initialize localization
      UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(1, dc);
      
      // initialize and start threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, dc);
      usPoller.start();
      Thread localizeThread = new Thread(usLocalizer);
      localizeThread.start();
      
      Button.waitForAnyPress();
      
      Thread lightLocalizeThread = new Thread(lightLocalizer);
      lightLocalizeThread.start();
    }
  
    if (buttonChoice == Button.ID_DOWN) {
      
      // US Poller 
      @SuppressWarnings("resource") // Because we don't bother to close this resource
      SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
      SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                                // this instance
      float[] usData = new float[usDistance.sampleSize()]; 
      
      // Initialize localization
      UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(2, dc);
      
      // initialize and start threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, dc);
      usPoller.start();
      Thread localizeThread = new Thread(usLocalizer);
      localizeThread.start();
      
      Button.waitForAnyPress();
      
      Thread lightLocalizeThread = new Thread(lightLocalizer);
      lightLocalizeThread.start();
    }
  }
}
