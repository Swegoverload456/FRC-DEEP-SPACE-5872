package frc.robot.Leds;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Led{

    Spark leds = new Spark(Constants.kBlinkinID);
    Timer timer = new Timer();

    public void off(){

        leds.set(0.99);

    }

    public void set(double color){

        leds.set(color);

    }

    public void blink(double color, double interval, int numberOfBlinks){

        //c = counter for toggle
        int c = 0;

        for(int i = 0; i < numberOfBlinks; i++){

            if(c == 0){

                timer.reset();
                timer.start();
                c = 1;

            }
            else if(c == 1 && timer.get() < interval){

                set(color);
                c = 2;

            }
            else if(c == 2 && timer.get() >= interval){

                timer.reset();
                timer.start();
                c = 3;

            }
            else if(c == 3 && timer.get() < interval){

                off();
                c = 4;

            }
            else if(c == 4 && timer.get() >= interval){

                c = 0;

            }

        }    

    }

}