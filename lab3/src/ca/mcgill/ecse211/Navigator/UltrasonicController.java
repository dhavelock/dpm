package ca.mcgill.ecse211.Navigator;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
