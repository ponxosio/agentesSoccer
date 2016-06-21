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
public class AgentPlayer extends Behaviour {

    public static final int advanceBall = 0;
    public static final int pass2Player = 1;
    public static final int shoot = 2;
    public static final int passAssist = 3;
    public static final int clear = 4;

    public static final int advance = 5;
    public static final int retreat = 6;
    public static final int blockPlayer = 7;
    public static final int coverGoal = 8;
    public static final int intercept = 9;
    public static final int go2Ball = 10;
    public static final int avoidColission = 11;

    static final double SIGMA_1 = 1.0d;
    static final double SIGMA_2 = 1.0d;
    static final double SIGMA_j = 1.0d;

    static final int MAX_TIME_FOR_CORRECT_PASS = 2000;

    // Thresholds updates
    public static final double score_goal =0.2;
    public static final double retrieve_ball = 0.2;
    public static final double correct_pass = 0.1;
    public static final double enter_oppArea = 0.1;
    public static final double kick_off_area = 0.1;
    public static final double ball_exit_field = 0.0;

    //INTERNAL
    //Thresholds

    //with the ball
    double threshold_advanceBall;
    double threshold_pass2Player;
    double threshold_shoot;
    double threshold_passAssist;
    double threshold_clear;

    //without the ball
    double threshold_advance;
    double threshold_retreat;
    double threshold_blockPlayer;
    double threshold_coverGoal;
    double threshold_intercept;
    double threshold_go2Ball;

    //internal attributes
    double lastSterrHeading;
    Vec2 lastPosition;

    int lastAction;
    Vec2 lastBallPosition;
    boolean inControlBall;
    boolean justPasses;
    long passedTimeStamp;
    int times2Forget;

    AgentManager manager;
    int player;

    //synchronization
    Semaphore updatingThresholds;
    Timer timerForget;

    public AgentPlayer(AgentManager manager, int player) {
        this.manager = manager;
        this.player = player;

        lastPosition = new Vec2();
        lastBallPosition = new Vec2();
        updatingThresholds = new Semaphore(1,true);
        timerForget = new Timer("timerForget");
        times2Forget = 4;
    }

    @Override
    public void configure() {
        //timerForget.scheduleAtFixedRate(new ThresholdForgetTask(this, updatingThresholds), 4000, 10000);

        //with the ball
        threshold_advanceBall = 1.0d;
        threshold_pass2Player= 1.0d;
        threshold_shoot= 1.0d;
        threshold_passAssist= 1.0d;
        threshold_clear= 1.0d;

        //without the ball
        threshold_advance= 1.0d;
        threshold_retreat= 1.0d;
        threshold_blockPlayer= 1.0d;
        threshold_coverGoal= 1.0d;
        threshold_intercept= 1.0d;
        threshold_go2Ball= 1.0d;
    }

    @Override
    public int takeStep() {
        if (times2Forget == 0) {
            times2Forget = 4;
            forgetThresholds();
        } else {
            times2Forget--;
        }

        lastSterrHeading = myRobotAPI.getSteerHeading();
        lastPosition = myRobotAPI.getPosition();

        Vec2 ball = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall());
        updateThreshold(ball);
        lastBallPosition.setx(ball.x);
        lastBallPosition.sety(ball.y);

        if (myRobotAPI.blocked()) {
            myRobotAPI.setDisplayString("avoiding");
            myRobotAPI.avoidCollisions();
            myRobotAPI.setSpeed(1.0);
            lastAction = avoidColission;
        } else if (!getPendingMessages().isEmpty()) {
            Message msg = getPendingMessages().poll();
            if (msg instanceof AssistMensaje) {
                AssistMensaje assit = (AssistMensaje) msg;
                Vec2 assitVec = assit.getAssistVec();

                myRobotAPI.setSteerHeading(assitVec.t);
                myRobotAPI.setSpeed(1.0);

                return RobotAPI.ROBOT_OK;
            } else if (msg instanceof RetreaveMessage) {
                if (justPasses) {
                    RetreaveMessage retreve = (RetreaveMessage) msg;
                    if ((passedTimeStamp -  retreve.getTimeStamp()) < MAX_TIME_FOR_CORRECT_PASS) {
                        double sigma = 8.0d;
                        double alfa = 4.0d;

                        double scoreFactor = Stimuli.scoreFactor(myRobotAPI);
                        double valor = correct_pass;
                        double epsilon = valor + (1.0 / (1.0 + Math.exp(-sigma*scoreFactor + alfa))) * valor;

                        threshold_pass2Player = Math.max(threshold_pass2Player - epsilon, 0.0);
                    }
                    justPasses = false;
                }
            }
        }

        /*if (myRobotAPI.closestToBall()) {
            Vec2 me2ball = new Vec2(ball);
            me2ball.sub(myRobotAPI.getPosition());
            if (me2ball.r <= 0.2) {
                if (lastAction > clear) {
                // if now can kick and last action was an action without the ball then this player has retreive the ball,
                    // sends a message to all player so if someone has made a pass know that was a correct pass
                    RetreaveMessage message = new RetreaveMessage(System.currentTimeMillis());
                    message.setSender(player);
                    message.setType(Message.Type.broadcast);
                    manager.sendMessage(message);
                }
                takeStepWithBall();
            } else {
                takeStepWithOutBall();
            }
        } else {*/
            takeStepWithOutBall();
        //}
        return RobotAPI.ROBOT_OK;
    }

    protected void forgetThresholds() {
        double nu = 0.1d;
        double sigma = 8.0d;
        double alfa = 4.0d;
        double scoreFactor = Stimuli.scoreFactor(myRobotAPI);
        double timeFactor = Stimuli.timeFactor(myRobotAPI);

        double forgetValue = nu - (0.5 * scoreFactor * nu + timeFactor * (1.0 / (1.0 + Math.exp(-sigma * scoreFactor + alfa)) - 0.5) * nu);
        threshold_advanceBall = Math.min(threshold_advanceBall + forgetValue, 1.0);
        threshold_pass2Player = Math.min(threshold_pass2Player + forgetValue, 1.0);
        threshold_shoot = Math.min(threshold_shoot + forgetValue, 1.0);
        threshold_passAssist = Math.min(threshold_passAssist + forgetValue, 1.0);
        threshold_clear = Math.min(threshold_clear + forgetValue, 1.0);
        threshold_advance = Math.min(threshold_advance + forgetValue, 1.0);
        threshold_retreat = Math.min(threshold_retreat + forgetValue, 1.0);
        threshold_blockPlayer = Math.min(threshold_blockPlayer + forgetValue, 1.0);
        threshold_coverGoal = Math.min(threshold_coverGoal + forgetValue, 1.0);
        threshold_intercept = Math.min(threshold_intercept + forgetValue, 1.0);
        threshold_go2Ball = Math.min(threshold_go2Ball + forgetValue, 1.0);
    }

    private void takeStepWithOutBall() {
        Vec2 advanceVector = new Vec2();
        double advanceStimuli = Stimuli.advanceStimuli(myRobotAPI, advanceVector);
        double pAdvance = advanceStimuli / (advanceStimuli + threshold_advance);

        Vec2 retreatVec = new Vec2();
        double retreatStimuli = Stimuli.retreatStimuli(myRobotAPI, retreatVec);
        double pRetreat = retreatStimuli / (retreatStimuli + threshold_retreat);

        Vec2[] opponents = Stimuli.getOpponentsFieldCoordinates(myRobotAPI);
        Vec2 player2Block = new Vec2();
        double maxBlockStimuli = Stimuli.blockPlayerStimuli(myRobotAPI, opponents[0], player2Block);
        for (int i = 1; i < opponents.length; i++) {
            Vec2 auxPlayer2Block = new Vec2();
            double auxBlockstimuli = Stimuli.blockPlayerStimuli(myRobotAPI, opponents[i], player2Block);
            if (auxBlockstimuli > maxBlockStimuli) {
                maxBlockStimuli = auxBlockstimuli;
                player2Block = auxPlayer2Block;
            }
        }
        double pBlock = 0.0;//maxBlockStimuli / (maxBlockStimuli + threshold_blockPlayer);

        Vec2 coverGoalVec = new Vec2();
        double coverGoalstimuli = Stimuli.coverGoalStimuli(myRobotAPI, coverGoalVec);
        double pCoverGoal = coverGoalstimuli / (coverGoalstimuli + threshold_coverGoal);

        Vec2 interceptVec = new Vec2();
        double interceptStimuli = 0.0d;
        if (!inControlBall) {
            Vec2 mainAttacker = getOpponentsMainAttacker();
            interceptStimuli = Stimuli.interceptStimuli(myRobotAPI, mainAttacker, interceptVec);
        }
        double pIntercept = interceptStimuli / (interceptStimuli + threshold_intercept);

        Vec2 toBallVec = new Vec2();
        double toBallStimuli = Stimuli.goToBallStimuli(myRobotAPI, toBallVec);
        double p2Ball = toBallStimuli / (toBallStimuli + threshold_go2Ball);

       if (pAdvance >= pRetreat /*&& pAdvance >= pBlock && pAdvance >= pCoverGoal && pAdvance >= pIntercept && pAdvance >= p2Ball*/) {
            //do advance
            myRobotAPI.setDisplayString("advance");

            lastAction = advance;
            myRobotAPI.setSteerHeading(advanceVector.t);
            myRobotAPI.setSpeed(1.0d);
        } else /*if (pRetreat >= pAdvance && pRetreat >= pBlock && pRetreat >= pCoverGoal && pRetreat >= pIntercept && pRetreat >= p2Ball) */{
           //do retreat
            myRobotAPI.setDisplayString("retreat");

            lastAction = retreat;
            myRobotAPI.setSteerHeading(retreatVec.t);
            myRobotAPI.setSpeed(1.0d);
        }/* else if (pBlock >= pRetreat && pBlock >= pAdvance && pBlock >= pCoverGoal && pBlock >= pIntercept && pBlock >= p2Ball) {
           //do Block
            myRobotAPI.setDisplayString("Block");

            lastAction = blockPlayer;
            Vec2 player2BlockVec = new Vec2(player2Block);
            player2BlockVec.sub(myRobotAPI.getPosition());

            Vec2 player2BlockDestination = new Vec2(player2Block);
            player2BlockDestination.add(player2BlockVec);

            myRobotAPI.surroundPoint(player2Block,player2BlockDestination);
        } else if (pCoverGoal >= pRetreat && pCoverGoal >= pBlock && pCoverGoal >= pAdvance && pCoverGoal >= pIntercept && pCoverGoal >= p2Ball) {
           //do cover goal
            myRobotAPI.setDisplayString("coverGoal");

            lastAction = coverGoal;
            myRobotAPI.setSteerHeading(coverGoalVec.t);
            myRobotAPI.setSpeed(1.0);
        } else if (pIntercept >= pRetreat && pIntercept >= pBlock && pIntercept >= pCoverGoal && pIntercept >= pAdvance && pIntercept >= p2Ball) {
           //do Intercept
            myRobotAPI.setDisplayString("intercept");

            lastAction = intercept;
            myRobotAPI.setSteerHeading(interceptVec.t);
            myRobotAPI.setSpeed(1.0);
        } else { // p2Ball >= pRetreat && p2Ball >= pBlock && p2Ball >= pCoverGoal && p2Ball >= pAdvance && p2Ball >= pAdvance
           //do go to ball
            myRobotAPI.setDisplayString("go2Ball");

            lastAction = go2Ball;
            myRobotAPI.setSteerHeading(toBallVec.t);
            myRobotAPI.avoidCollisions();
            myRobotAPI.setSpeed(1.0);
        }*/
    }

    private void takeStepWithBall() {
        Vec2 advanceVector = new Vec2();
        double advanceStimuli = Stimuli.advanceStimuliBall(myRobotAPI,advanceVector);
        double pAdvance = advanceStimuli / (advanceStimuli + threshold_advanceBall);

        Vec2 passVector = new Vec2();
        int player2pass = 0;
        double maxPassStimuli = 0.0d;
        for (int i = 0; i < 5; i++) {
            AgentPlayer actualp = (AgentPlayer) manager.getDefaultBehaviour(i);
            if (actualp.player != this.player) {
                Vec2 auxPassVector = new Vec2();
                double passStimuli = Stimuli.passToPlayerStimuli(myRobotAPI, actualp.lastPosition, actualp.lastSterrHeading, auxPassVector);
                if (passStimuli > maxPassStimuli) {
                    maxPassStimuli = passStimuli;
                    passVector = auxPassVector;
                    player2pass = i;
                }
            }
        }
        double pPass = maxPassStimuli / (maxPassStimuli + threshold_pass2Player);

        Vec2 shootVec = new Vec2();
        Vec2 assistVec = new Vec2();
        Vec2 assistPlayer = new Vec2();
        double shootStimuli[] = Stimuli.shootStimuli(myRobotAPI, shootVec, assistVec, assistPlayer);
        double pShoot = shootStimuli[0] / (shootStimuli[0] + threshold_shoot);
        double pAssist = shootStimuli[1] / (shootStimuli[1] + threshold_passAssist);

        Vec2 clearVec = new Vec2();
        double clearStimuli = Stimuli.clearStimuli(myRobotAPI, clearVec);
        double pClear = clearStimuli / (clearStimuli + threshold_clear);

        if (/*pAdvance >= pPass &&*/ pAdvance >= pShoot /*&& pAdvance >= pAssist && pAdvance >= pClear*/) {
            //do advance
            myRobotAPI.setDisplayString("advanceBall");

            Vec2 myPosAux = new Vec2(myRobotAPI.getPosition());
            //advanceVector.normalize(1.0);
            myPosAux.add(advanceVector);

            lastAction = advanceBall;
            myRobotAPI.setBehindBall(myPosAux);
            myRobotAPI.setSpeed(1.0);
        /*} else if (pPass >= pAdvance && pPass >= pShoot && pPass >= pAssist && pPass >= pClear) {
            //do pass
            myRobotAPI.setDisplayString("Pass");

            lastAction = pass2Player;
            justPasses = true;
            passedTimeStamp = System.currentTimeMillis();

            myRobotAPI.setSteerHeading(passVector.t);
            myRobotAPI.kick();*/
        } else /*if (pShoot >= pAdvance && pShoot >= pPass && pShoot >= pAssist && pShoot >= pClear)*/ {
            //do shoot
            myRobotAPI.setDisplayString("shoot");

            lastAction = shoot;
            myRobotAPI.setSteerHeading(shootVec.t);
            myRobotAPI.kick();
        } /*else if (pAssist >= pAdvance && pAssist >= pPass && pAssist >= pShoot && pAssist >= pClear) {
            //do assist
            myRobotAPI.setDisplayString("assist");

            AssistMensaje msj = new AssistMensaje(assistVec);
            msj.setSender(this.player);
            msj.setReceiver(player2pass);
            msj.setType(Message.Type.unicast);

            manager.sendMessage(msj);
            myRobotAPI.setSteerHeading(shootVec.t);
            myRobotAPI.kick();
        } else if (pClear >= pAdvance && pClear >= pPass && pClear >= pShoot && pClear >= pAssist) {
            //do clear
            myRobotAPI.setDisplayString("clear");

            lastAction = clear;
            myRobotAPI.setSteerHeading(clearVec.t);
            myRobotAPI.kick();
        }*/

    }

    @Override
    public void onInit(RobotAPI robotAPI) {

    }

    @Override
    public void end() {
        timerForget.cancel();
    }

    @Override
    public void onRelease(RobotAPI robotAPI) {
    }

    protected void updateThreshold(Vec2 ball) {
        double sigma = 8.0d;
        double alfa = 4.0d;

        double scoreFactor = Stimuli.scoreFactor(myRobotAPI);
        double valor = getAmountUpdated(ball);
        double epsilon = valor + (1.0 / (1.0 + Math.exp(-sigma*scoreFactor + alfa))) * valor;

        try {
            updatingThresholds.acquire();
            switch (lastAction) {
                case advanceBall:
                    threshold_advanceBall = Math.max(threshold_advanceBall - epsilon, 0.0);
                    break;
                case pass2Player:
                    threshold_pass2Player = Math.max(threshold_pass2Player - epsilon, 0.0);
                    break;
                case shoot:
                    threshold_shoot = Math.max(threshold_shoot - epsilon, 0.0);
                    break;
                case passAssist:
                    threshold_passAssist = Math.max(threshold_passAssist - epsilon, 0.0);
                    break;
                case clear:
                    threshold_clear = Math.max(threshold_clear - epsilon, 0.0);
                    break;
                case advance:
                    threshold_advance = Math.max(threshold_advance - epsilon, 0.0);
                    break;
                case retreat:
                    threshold_retreat = Math.max(threshold_retreat - epsilon, 0.0);
                    break;
                case blockPlayer:
                    threshold_blockPlayer = Math.max(threshold_blockPlayer - epsilon, 0.0);
                    break;
                case coverGoal:
                    threshold_coverGoal = Math.max(threshold_coverGoal - epsilon, 0.0);
                    break;
                case intercept:
                    threshold_intercept = Math.max(threshold_intercept - epsilon, 0.0);
                    break;
                case go2Ball:
                    threshold_go2Ball = Math.max(threshold_go2Ball - epsilon, 0.0);
                    break;
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            updatingThresholds.release();
        }
    }

    protected double getAmountUpdated(Vec2 ball) {
        double updated = 0.0d;

        if (scored()) {
            updated = score_goal;
        } else if (retrieveBall()) {
            updated = retrieve_ball;
        } else if (enterOppsArea(ball)) {
            updated = enter_oppArea;
        } else if (leaveOurArea(ball)) {
            updated = kick_off_area;
        } else if (ballLeaveOurField(ball)) {
            updated = ball_exit_field;
        }
        return updated;
    }

    protected boolean scored() {
        return myRobotAPI.getJustScored() == 1;
    }

    protected boolean retrieveBall() {
        boolean ballRetreive = false;
        boolean newInControlValue = updateInControlBall();
        if (!inControlBall && newInControlValue) {
            ballRetreive = true;
        }
        inControlBall = newInControlValue;
        return ballRetreive;
    }

    protected boolean enterOppsArea(Vec2 ball) {
        boolean enteredOppArea = false;
        Rectangle2D area = Stimuli.opponentsGoalTerritory(myRobotAPI.getFieldSide());

        if (!area.contains(lastBallPosition.x, lastBallPosition.y) &&
                area.contains(ball.x, ball.y)) {
            enteredOppArea = true;
        }
        return enteredOppArea;
    }

    protected boolean leaveOurArea(Vec2 ball) {
        boolean leaveArea = false;
        Rectangle2D area = Stimuli.golieTerritory(myRobotAPI.getFieldSide());

        if (area.contains(lastBallPosition.x, lastBallPosition.y) &&
                !area.contains(ball.x, ball.y)) {
            leaveArea = true;
        }
        return leaveArea;
    }

    protected boolean ballLeaveOurField(Vec2 ball) {
        boolean leaveField = false;
        if (Math.signum(lastBallPosition.x) == myRobotAPI.getFieldSide() &&
                Math.signum(ball.x) != myRobotAPI.getFieldSide()) {
            leaveField = true;
        }
        return leaveField;
    }

    protected boolean updateInControlBall() {
        Vec2 ball = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall());
        Vec2 mainAttacker = getOpponentsMainAttacker();
        Vec2 mainAttacker2Ball = new Vec2(ball);
        mainAttacker2Ball.sub(mainAttacker);

        Vec2[] mates = Stimuli.getMatesFieldCoordinates(myRobotAPI);
        boolean inControl = false;
        for (int i = 0; !inControl && i < mates.length; i++) {
            Vec2 player2ballaux = new Vec2(ball);
            player2ballaux.sub(mates[i]);
            inControl = (player2ballaux.r > mainAttacker2Ball.r);
        }
        inControlBall = inControl;
        return inControl;
    }

    protected Vec2 getOpponentsMainAttacker() {
        Vec2 ball = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall());
        Vec2[] opponents = Stimuli.getOpponentsFieldCoordinates(myRobotAPI);
        Vec2 mainAttacker = opponents[0];

        for (int i = 1; i < opponents.length; i++) {
            Vec2 auxOpp2Ball = new Vec2(ball);
            auxOpp2Ball.sub(opponents[i]);

            Vec2 auxAttaker2Ball = new Vec2(ball);
            auxAttaker2Ball.sub(mainAttacker);
            if (auxOpp2Ball.r < auxAttaker2Ball.r) {
                mainAttacker = new Vec2(opponents[i]);
            }
        }
        return mainAttacker;
    }

    private class ThresholdForgetTask extends TimerTask {

        private AgentPlayer player;
        private Semaphore thresholdsUpdating;

        public ThresholdForgetTask(AgentPlayer player, Semaphore thresholdsUpdating) {
            this.player = player;
            this.thresholdsUpdating = thresholdsUpdating;
        }

        @Override
        public void run() {
            if (myRobotAPI != null) {
                double nu = 0.1d;
                double sigma = 8.0d;
                double alfa = 4.0d;
                double scoreFactor = Stimuli.scoreFactor(player.myRobotAPI);
                double timeFactor = Stimuli.timeFactor(player.myRobotAPI);

                double forgetValue = nu - (0.5 * scoreFactor * nu + timeFactor * (1.0 / (1.0 + Math.exp(-sigma * scoreFactor + alfa)) - 0.5) * nu);

                try {
                    thresholdsUpdating.acquire();
                    threshold_advanceBall = Math.min(threshold_advanceBall + forgetValue, 1.0);
                    threshold_pass2Player = Math.min(threshold_pass2Player + forgetValue, 1.0);
                    threshold_shoot = Math.min(threshold_shoot + forgetValue, 1.0);
                    threshold_passAssist = Math.min(threshold_passAssist + forgetValue, 1.0);
                    threshold_clear = Math.min(threshold_clear + forgetValue, 1.0);
                    threshold_advance = Math.min(threshold_advance + forgetValue, 1.0);
                    threshold_retreat = Math.min(threshold_retreat + forgetValue, 1.0);
                    threshold_blockPlayer = Math.min(threshold_blockPlayer + forgetValue, 1.0);
                    threshold_coverGoal = Math.min(threshold_coverGoal + forgetValue, 1.0);
                    threshold_intercept = Math.min(threshold_intercept + forgetValue, 1.0);
                    threshold_go2Ball = Math.min(threshold_go2Ball + forgetValue, 1.0);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                } finally {
                    thresholdsUpdating.release();
                }
            }
        }
    }
}
