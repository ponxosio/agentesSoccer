package Equipo;

import EDU.gatech.cc.is.util.Vec2;
import funciones.Stimuli;
import math.geom2d.polygon.Rectangle2D;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.Message;
import teams.ucmTeam.RobotAPI;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.Semaphore;

/**
 * Created by angel on 22/01/2016.
 */
public class Passer extends AgentPlayer {

    public Passer(AgentManager manager, int player) {
        super(manager, player);
        this.manager = manager;
        this.player = player;

        lastPosition = new Vec2();
        lastBallPosition = new Vec2();
        updatingThresholds = new Semaphore(1,true);
        timerForget = new Timer("timerForget");
        times2Forget = 4;
    }

    @Override
    public int takeStep() {
        if (times2Forget == 0) {
            times2Forget = 4;
            super.forgetThresholds();
        } else {
            times2Forget--;
        }

        lastSterrHeading = myRobotAPI.getSteerHeading();
        lastPosition = myRobotAPI.getPosition();

        Vec2 ball = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall());
        super.updateThreshold(ball);
        lastBallPosition.setx(ball.x);
        lastBallPosition.sety(ball.y);

        Vec2 aux = new Vec2(myRobotAPI.getPosition());
        aux.setx(myRobotAPI.toFieldCoordinates(myRobotAPI.getOurGoal()).x);
        aux.sub(myRobotAPI.getPosition());

        myRobotAPI.setSteerHeading(aux.t);
        myRobotAPI.setSpeed(1.0);

        return RobotAPI.ROBOT_OK;
    }
}
