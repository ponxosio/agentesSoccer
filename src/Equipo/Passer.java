package Equipo;

import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.RobotAPI;

/**
 * Created by angel on 21/01/2016.
 */
public class Passer extends Behaviour {

    @Override
    public void configure() {

    }

    @Override
    public int takeStep() {
        double angle = this.myRobotAPI.getSteerHeading();

        Vec2 ball = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall());
        Vec2 player = myRobotAPI.getPosition();

        Vec2 plainVec = new Vec2(1,0);
        Vec2 pb = new Vec2(ball);
        pb.sub(player);

        double angle2 = pb.angle(plainVec);

        this.myRobotAPI.setDisplayString("radianes: " + Double.toString(Math.abs(angle-angle2)*(180/Math.PI)));

        //this.myRobotAPI.setSteerHeading(angle + 0.174);
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
