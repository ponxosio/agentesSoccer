package Equipo;

import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.RobotAPI;

/**
 * Created by angel on 21/01/2016.
 */
public class Passer extends Behaviour {

    boolean hasKick;

    @Override
    public void configure() {
        hasKick = false;
    }

    @Override
    public int takeStep() {
        if(myRobotAPI.blocked() || myRobotAPI.teammateBlocking()){
            myRobotAPI.avoidCollisions();
            myRobotAPI.setSpeed(1.0);
            return RobotAPI.ROBOT_OK;
        }

        if (hasKick) {
            System.out.println("ball advance");
            double vector = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall()).distance(myRobotAPI.getPosition());
            System.out.println(vector);
            hasKick = false;
        }

        if (myRobotAPI.canKick()) {
            System.out.println("kicking ball");
            myRobotAPI.setBehindBall(myRobotAPI.getOpponentsGoal());
            myRobotAPI.passBall(myRobotAPI.getOpponentsGoal());
            hasKick = true;
        } else {
            myRobotAPI.setSteerHeading(myRobotAPI.getBall().t);
            myRobotAPI.setSpeed(1.0);
        }

        return RobotAPI.ROBOT_OK;
    }

    @Override
    public void onInit(RobotAPI robotAPI) {

    }

    @Override
    public void end() {

    }

    @Override
    public void onRelease(RobotAPI robotAPI) {

    }
}
