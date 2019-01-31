package ca.mcgill.ecse211.lab3;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import ca.mcgill.ecse211.Navigator.*;

public class Lab3 {
	
	// Motor Objects, and Robot related parameters
  public static final EV3LargeRegulatedMotor leftMotor =
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor =
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 13.382; 
  
  public static final int[] wayPoints = {2,1 , 1,1 , 1,2 , 2,0};

  public static void main(String[] args) throws OdometerExceptions {
	
	int buttonChoice;
	  
	// Odometer related objects
	Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
	OdometryCorrection odometryCorrection = new OdometryCorrection();
	
	
	// Navigator object
	Navigator navigator = new Navigator(odometer, wayPoints);
	
	Display odometryDisplay = new Display(lcd);
	
	do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Start! | Nothing", 0, 2);
      lcd.drawString("       |  Here  ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT);
	
	if (buttonChoice == Button.ID_LEFT) {

      // Display changes in position as wheels are (manually) moved
      
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      Thread navigatorThread = new Thread(navigator);
      navigatorThread.start();
      
    }
  }
}
