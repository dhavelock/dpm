package ca.mcgill.ecse211.Navigator;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;

public class Lab3 {
	
	// Motor Objects, and Robot related parameters
  private static final Port usPort = LocalEV3.get().getPort("S2");
  public static final EV3LargeRegulatedMotor leftMotor =
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor =
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  public static final double WHEEL_RAD = 2.2;     // Measured and tweaked Wheel radius
  public static final double TRACK = 13.382;      // Measured and tweaked Track length
  
  public static final int[][] WAYPOINTS = {{1,0}, {2,1}, {2,2}, {0,2}, {1,1}};
  // 1) {0,2}, {1,1}, {2,2}, {2,1}, {1,0}
  // 2) {1,1}, {0,2}, {2,2}, {2,1}, {1,0}
  // 3) {1,0}, {2,1}, {2,2}, {0,2}, {1,1}
  // 4) {0,1}, {1,2}, {1,0}, {2,1}, {2,2}

  public static void main(String[] args) throws OdometerExceptions {
	
  	int buttonChoice;
  	  
  	// Initialize odometer
  	Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
  	
  	
  	// Initialize NavigatorObstacle 
  	NavigatorObstacle obstacleAvoidance = new NavigatorObstacle();
  		
  	do {
      // clear the display
      lcd.clear();
  
      // ask the user whether the motors should use obstacle avoidance or not
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Regular| Avoid  ", 0, 2);
      lcd.drawString("       |Obstacle", 0, 3);
      lcd.drawString("       |        ", 0, 4);
  
      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
	
	  //Left button initiates navigation with obstacle avoidance
  	if (buttonChoice == Button.ID_LEFT) {
      
      // initialize and start threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Navigator navigator = new Navigator(odometer);
      navigator.start();
      
    }
	
  	// Right button initiates navigation with obstacle avoidance
  	if (buttonChoice == Button.ID_RIGHT) {
  
  	  @SuppressWarnings("resource") // Because we don't bother to close this resource
  	  SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
  	  SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
  	                                                            // this instance
  	  float[] usData = new float[usDistance.sampleSize()]; 
  	  
  	  // initialize and start threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, obstacleAvoidance);
      usPoller.start();
      Navigator navigator = new Navigator(odometer, obstacleAvoidance);
      navigator.start();
    }
  }
}
