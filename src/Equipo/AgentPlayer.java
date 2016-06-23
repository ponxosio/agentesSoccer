package Equipo;

import EDU.gatech.cc.is.util.Vec2;
import funciones.Stimuli;
import math.geom2d.polygon.Rectangle2D;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.Message;
import teams.ucmTeam.RobotAPI;

import java.io.*;
import java.util.*;
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
    public static final double score_goal = 0.3;
    public static final double retrieve_ball = 0.2;
    public static final double correct_pass = 0.2;
    public static final double enter_oppArea = 0.1;
    public static final double kick_off_area = 0.15;
    public static final double ball_exit_field = 0.005;

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

    Vec2 lastBallPosition;
    boolean inControlBall;
    boolean justPasses;
    long passedTimeStamp;
    int times2Forget;
    int times2Act;
    AccionAcumulativeDistribution lastAction;

    AgentManager manager;
    int player;

    //statistics
    int advanceBall_count;
    int pass2Player_count;
    int shoot_count;
    int passAssist_count;
    int clear_count;
    int advance_count;
    int retreat_count;
    int blockPlayer_count;
    int coverGoal_count;
    int intercept_count;
    int go2Ball_count;
    int avoidColission_count;


    public AgentPlayer(AgentManager manager, int player) {
        this.manager = manager;
        this.player = player;

        lastPosition = new Vec2();
        lastBallPosition = new Vec2();
        lastAction = new AccionAcumulativeDistribution(avoidColission, 0.0, new Vec2());
        times2Forget = 10;
        times2Act = 0;

        advanceBall_count = 0;
        pass2Player_count = 0;
        shoot_count = 0;
        passAssist_count = 0;
        clear_count = 0;
        advance_count = 0;
        retreat_count = 0;
        blockPlayer_count = 0;
        coverGoal_count = 0;
        intercept_count = 0;
        go2Ball_count = 0;
        avoidColission_count = 0;

        //with the ball
        threshold_advanceBall = 0.5d;
        threshold_pass2Player = 0.5d;
        threshold_shoot = 0.5d;
        threshold_passAssist = 0.5d;
        threshold_clear = 0.5d;

        //without the ball
        threshold_advance = 0.5d;
        threshold_retreat = 0.5d;
        threshold_blockPlayer = 0.5d;
        threshold_coverGoal = 0.5d;
        threshold_intercept = 0.5d;
        threshold_go2Ball = 0.5d;
    }

    @Override
    public void configure() {

    }

    @Override
    public int takeStep() {
        if (times2Forget == 0) {
            times2Forget = 10;
            restore2Default();
        } else {
            times2Forget--;
        }

        lastSterrHeading = myRobotAPI.getSteerHeading();
        lastPosition = myRobotAPI.getPosition();

        Vec2 ball = myRobotAPI.toFieldCoordinates(myRobotAPI.getBall());
        updateThreshold(ball);
        lastBallPosition.setx(ball.x);
        lastBallPosition.sety(ball.y);

        if (times2Act == 0) {
            times2Act = 0;
            if (myRobotAPI.blocked()) {
                myRobotAPI.setDisplayString("avoiding");
                myRobotAPI.avoidCollisions();
                myRobotAPI.setSpeed(1.0);
                lastAction = new AccionAcumulativeDistribution(avoidColission, 1.0, new Vec2());
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
                        if ((passedTimeStamp - retreve.getTimeStamp()) < MAX_TIME_FOR_CORRECT_PASS) {
                            double sigma = 8.0d;
                            double alfa = 4.0d;

                            double scoreFactor = Stimuli.scoreFactor(myRobotAPI);
                            double valor = correct_pass;
                            double epsilon = valor + (1.0 / (1.0 + Math.exp(-sigma * scoreFactor + alfa))) * valor;

                            threshold_pass2Player = Math.max(threshold_pass2Player - epsilon, 0.0);
                        }
                        justPasses = false;
                    }
                }
            }

            if (myRobotAPI.closestToBall()) {
                Vec2 me2ball = new Vec2(ball);
                me2ball.sub(myRobotAPI.getPosition());
                if (me2ball.r <= 0.2) {
                    if (lastAction.action > clear) {
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
            } else {
                takeStepWithOutBall();
            }
        } else {
            times2Act--;
            lastAction.executeAction();
        }
        return RobotAPI.ROBOT_OK;
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
        double pBlock = maxBlockStimuli / (maxBlockStimuli + threshold_blockPlayer);

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

        double total = pAdvance + pRetreat + pBlock + pCoverGoal + pIntercept + p2Ball;

        ArrayList<AccionAcumulativeDistribution> distrubition = new ArrayList<>();
        distrubition.add(new AccionAcumulativeDistribution(advance, pAdvance / total, advanceVector));
        distrubition.add(new AccionAcumulativeDistribution(retreat, pRetreat / total, retreatVec));
        distrubition.add(new AccionAcumulativeDistribution(blockPlayer, pBlock / total, player2Block));
        distrubition.add(new AccionAcumulativeDistribution(coverGoal, pCoverGoal / total, coverGoalVec));
        distrubition.add(new AccionAcumulativeDistribution(intercept, pIntercept / total, interceptVec));
        distrubition.add(new AccionAcumulativeDistribution(go2Ball, p2Ball / total, toBallVec));

        calculateAccumulativeValues(distrubition);

        double random = Math.random();
        Iterator<AccionAcumulativeDistribution> it = distrubition.iterator();

        double disValue;
        AccionAcumulativeDistribution selected;
        do {
            selected = it.next();
            disValue = selected.getProbValue();
        } while (random > disValue && it.hasNext());

        selected.executeAction();
        lastAction = selected;
    }

    private void takeStepWithBall() {
        Vec2 advanceVector = new Vec2();
        double advanceStimuli = Stimuli.advanceStimuliBall(myRobotAPI, advanceVector);
        double pAdvance = advanceStimuli / (advanceStimuli + threshold_advanceBall);

        Vec2 passVector = new Vec2();
        double maxPassStimuli = 0.0d;
        for (int i = 0; i < 5; i++) {
            AgentPlayer actualp = (AgentPlayer) manager.getDefaultBehaviour(i);
            if (actualp.player != this.player) {
                Vec2 auxPassVector = new Vec2();
                double passStimuli = Stimuli.passToPlayerStimuli(myRobotAPI, actualp.lastPosition, actualp.lastSterrHeading, auxPassVector);
                if (passStimuli > maxPassStimuli) {
                    maxPassStimuli = passStimuli;
                    passVector = auxPassVector;
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

        int player2Assist = player == 0 ? 1 : 0;
        AgentPlayer auxPlayer = (AgentPlayer) (player == 0 ? manager.getDefaultBehaviour(1) : manager.getDefaultBehaviour(0));

        Vec2 distAux = new Vec2(auxPlayer.lastPosition);
        distAux.sub(assistPlayer);
        for (int i = 0; i < 5; i++) {
            if (i != player &&
                    i != (player == 0 ? 1 : 0)) {
                AgentPlayer actualp = (AgentPlayer) manager.getDefaultBehaviour(i);
                Vec2 actuaAuxDis = new Vec2(actualp.lastPosition);
                actuaAuxDis.sub(assistPlayer);

                if (actuaAuxDis.r < distAux.r) {
                    distAux = actuaAuxDis;
                    player2Assist = i;
                }
            }
        }

        Vec2 clearVec = new Vec2();
        double clearStimuli = Stimuli.clearStimuli(myRobotAPI, clearVec);
        double pClear = clearStimuli / (clearStimuli + threshold_clear);

        double total = pAdvance + pPass + pShoot + pAssist + pClear;

        ArrayList<AccionAcumulativeDistribution> distrubition = new ArrayList<>();
        distrubition.add(new AccionAcumulativeDistribution(advanceBall, pAdvance / total, advanceVector));
        distrubition.add(new AccionAcumulativeDistribution(pass2Player, pPass / total, passVector));
        distrubition.add(new AccionAcumulativeDistribution(shoot, pShoot / total, shootVec));
        distrubition.add(new AccionAcumulativeDistribution(passAssist, pAssist / total, assistVec, player2Assist));
        distrubition.add(new AccionAcumulativeDistribution(clear, pClear / total, clearVec));

        calculateAccumulativeValues(distrubition);

        double random = Math.random();
        Iterator<AccionAcumulativeDistribution> it = distrubition.iterator();

        double disValue;
        AccionAcumulativeDistribution selected;
        do {
            selected = it.next();
            disValue = selected.getProbValue();
        } while (random > disValue && it.hasNext());

        selected.executeAction();
        lastAction = selected;
    }

    private void calculateAccumulativeValues(ArrayList<AccionAcumulativeDistribution> list) {
        Collections.sort(list);
        for (int i = 1; i < (list.size() - 1); i++) {
            AccionAcumulativeDistribution actual = list.get(i);
            AccionAcumulativeDistribution prev = list.get(i - 1);

            actual.setProbValue(actual.getProbValue() + prev.getProbValue());
        }
    }

    @Override
    public void onInit(RobotAPI robotAPI) {

    }

    @Override
    public void end() {
        File file = new File("thresholds_" + Integer.toString(player) + ".data");
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(file));

            writer.write("advanceBall: " + threshold_advanceBall + "\n");
            writer.write("shoot: " + threshold_shoot + "\n");
            writer.write("pass: " + threshold_pass2Player + "\n");
            writer.write("assist: " + threshold_passAssist + "\n");
            writer.write("clear: " + threshold_clear + "\n");
            writer.write("advance: " + threshold_advance + "\n");
            writer.write("retreat: " + threshold_retreat + "\n");
            writer.write("block: " + threshold_blockPlayer + "\n");
            writer.write("coverGoal: " + threshold_coverGoal + "\n");
            writer.write("intercept: " + threshold_intercept + "\n");
            writer.write("go2Ball: " + threshold_go2Ball + "\n");
            writer.write("\n");
            writer.write("advanceBall_count: " + advanceBall_count + "\n");
            writer.write("pass2Player_count: " + pass2Player_count + "\n");
            writer.write("shoot_count: " + shoot_count + "\n");
            writer.write("passAssist_count: " + passAssist_count + "\n");
            writer.write("clear_count: " + clear_count + "\n");
            writer.write("advance_count: " + advance_count + "\n");
            writer.write("retreat_count: " + retreat_count + "\n");
            writer.write("blockPlayer_count: " + blockPlayer_count + "\n");
            writer.write("coverGoal_count: " + coverGoal_count + "\n");
            writer.write("intercept_count: " + intercept_count + "\n");
            writer.write("go2Ball_count: " + go2Ball_count + "\n");
            writer.write("avoidColission_count: " + avoidColission_count + "\n");

            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onRelease(RobotAPI robotAPI) {
    }

    protected void updateThreshold(Vec2 ball) {
        double sigma_f = 8.0d;
        double alfa_f = 4.0d;
        double sigma = 8.0d;
        double alfa = 4.0d;

        double scoreFactor = Stimuli.scoreFactor(myRobotAPI);
        double timeFactor = Stimuli.timeFactor(myRobotAPI);

        double valor = getAmountUpdated(ball);
        double epsilon = 0;
        if (valor > 0) {
            epsilon = valor + (1.0 / (1.0 + Math.exp(-sigma * scoreFactor + alfa))) * valor;
            epsilon = epsilon * -1;
        } else if (valor < 0) {
            valor = -1 * valor;
            epsilon = valor + (1.0 / (1.0 + Math.exp(-sigma * scoreFactor + alfa))) * valor;
        }

        double max = 1.0;
        switch (lastAction.action) {
            case advanceBall:
                threshold_advanceBall = threshold_advanceBall + epsilon;
                threshold_advanceBall = Math.min(threshold_advanceBall, max);
                threshold_advanceBall = Math.max(threshold_advanceBall, 0.0);
                break;
            case pass2Player:
                threshold_pass2Player = threshold_pass2Player + epsilon;
                threshold_pass2Player = Math.min(threshold_pass2Player, max);
                threshold_pass2Player = Math.max(threshold_pass2Player, 0.0);
                break;
            case shoot:
                threshold_shoot = threshold_shoot + epsilon;
                threshold_shoot = Math.min(threshold_shoot, max);
                threshold_shoot = Math.max(threshold_shoot, 0.0);
                break;
            case passAssist:
                threshold_passAssist = threshold_passAssist + epsilon;
                threshold_passAssist = Math.min(threshold_passAssist, max);
                threshold_passAssist = Math.max(threshold_passAssist, 0.0);
                break;
            case clear:
                threshold_clear =threshold_clear + epsilon;
                threshold_clear = Math.min(threshold_clear, max);
                threshold_clear = Math.max(threshold_clear, 0.0);
                break;
            case advance:
                threshold_advance = threshold_advance + epsilon;
                threshold_advance = Math.min(threshold_advance, max);
                threshold_advance = Math.max(threshold_advance, 0.0);
                break;
            case retreat:
                threshold_retreat = threshold_retreat + epsilon;
                threshold_retreat = Math.min(threshold_retreat, max);
                threshold_retreat = Math.max(threshold_retreat, 0.0);
                break;
            case blockPlayer:
                threshold_blockPlayer = threshold_blockPlayer +epsilon;
                threshold_blockPlayer = Math.min(threshold_blockPlayer, max);
                threshold_blockPlayer = Math.max(threshold_blockPlayer, 0.0);
                break;
            case coverGoal:
                threshold_coverGoal = threshold_coverGoal + epsilon;
                threshold_coverGoal = Math.min(threshold_coverGoal, max);
                threshold_coverGoal = Math.max(threshold_coverGoal, 0.0);
                break;
            case intercept:
                threshold_intercept = threshold_intercept + epsilon;
                threshold_intercept = Math.min(threshold_intercept, max);
                threshold_intercept = Math.max(threshold_intercept, 0.0);
                break;
            case go2Ball:
                threshold_go2Ball =threshold_go2Ball + epsilon;
                threshold_go2Ball = Math.min(threshold_go2Ball, max);
                threshold_go2Ball = Math.max(threshold_go2Ball, 0.0);
                break;
        }
    }

    protected double getAmountUpdated(Vec2 ball) {
        double updated = 0.0d;
        boolean[] ballControl = retrieveBall();

        if (scored()) {
            updated = score_goal;
        } else if (opponentScore()) {
            updated = -score_goal;
        } else if (ballControl[0]) {
            updated = retrieve_ball;
        } else if (ballControl[1]) {
            updated = -retrieve_ball;
        } else if (enterOppsArea(ball)) {
            updated = enter_oppArea;
        } else if (leaveOppsArea(ball)) {
            updated = -enter_oppArea;
        } else if (leaveOurArea(ball)) {
            updated = kick_off_area;
        } else if (entersOurArea(ball)) {
            updated = -kick_off_area;
        } else if (ballLeaveOurField(ball)) {
            updated = ball_exit_field;
        } else if (ballEntersOurField(ball)) {
            updated = -ball_exit_field;
        }
        return updated;
    }

    protected boolean scored() {
        return myRobotAPI.getJustScored() == 1;
    }

    protected  boolean opponentScore() {
        return  myRobotAPI.getJustScored() == -1;
    }

    protected boolean[] retrieveBall() {
        boolean ballRetreive[] = new boolean[2];
        ballRetreive[0] = false;
        ballRetreive[1] = true;
        boolean newInControlValue = updateInControlBall();
        if (!inControlBall && newInControlValue) {
            ballRetreive[0] = true;
        } else if (inControlBall && !newInControlValue) {
            ballRetreive[1] = true;
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

    protected boolean leaveOppsArea(Vec2 ball) {
        boolean leaveOppsArea = false;
        Rectangle2D area = Stimuli.opponentsGoalTerritory(myRobotAPI.getFieldSide());

        if (area.contains(lastBallPosition.x, lastBallPosition.y) &&
                !area.contains(ball.x, ball.y)) {
            leaveOppsArea = true;
        }
        return leaveOppsArea;
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

    protected boolean entersOurArea(Vec2 ball) {
        boolean entersOurArea = false;
        Rectangle2D area = Stimuli.golieTerritory(myRobotAPI.getFieldSide());

        if (!area.contains(lastBallPosition.x, lastBallPosition.y) &&
                area.contains(ball.x, ball.y)) {
            entersOurArea = true;
        }
        return entersOurArea;
    }

    protected boolean ballLeaveOurField(Vec2 ball) {
        boolean leaveField = false;
        if (Math.signum(lastBallPosition.x) == myRobotAPI.getFieldSide() &&
                Math.signum(ball.x) != myRobotAPI.getFieldSide()) {
            leaveField = true;
        }
        return leaveField;
    }

    protected boolean ballEntersOurField(Vec2 ball) {
        boolean ballEntersOurField = false;
        if (Math.signum(lastBallPosition.x) != myRobotAPI.getFieldSide() &&
                Math.signum(ball.x) == myRobotAPI.getFieldSide()) {
            ballEntersOurField = true;
        }
        return ballEntersOurField;
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

    protected void restore2Default() {
        if (threshold_advanceBall > 0.5) {
            threshold_advanceBall = threshold_advanceBall - 0.01;
        } else if (threshold_advanceBall < 0.5) {
            threshold_advanceBall = threshold_advanceBall + 0.01;
        }

        if (threshold_pass2Player > 0.5) {
            threshold_pass2Player = threshold_pass2Player - 0.01;
        } else if (threshold_pass2Player < 0.5) {
            threshold_pass2Player = threshold_pass2Player + 0.01;
        }

        if (threshold_passAssist > 0.5) {
            threshold_passAssist = threshold_passAssist - 0.01;
        } else if (threshold_passAssist < 0.5) {
            threshold_passAssist = threshold_passAssist + 0.01;
        }

        if (threshold_shoot > 0.5) {
            threshold_shoot = threshold_shoot - 0.01;
        } else if (threshold_shoot < 0.5) {
            threshold_shoot = threshold_shoot + 0.01;
        }

        if (threshold_clear > 0.5) {
            threshold_clear = threshold_clear - 0.01;
        } else if (threshold_clear < 0.5) {
            threshold_clear = threshold_clear + 0.01;
        }

        if (threshold_advance > 0.5) {
            threshold_advance = threshold_advance - 0.01;
        } else if (threshold_advance < 0.5) {
            threshold_advance = threshold_advance + 0.01;
        }

        if (threshold_retreat > 0.5) {
            threshold_retreat = threshold_retreat - 0.01;
        } else if (threshold_retreat < 0.5) {
            threshold_retreat = threshold_retreat + 0.01;
        }

        if (threshold_blockPlayer > 0.5) {
            threshold_blockPlayer = threshold_blockPlayer - 0.01;
        } else if (threshold_blockPlayer < 0.5) {
            threshold_blockPlayer = threshold_blockPlayer + 0.01;
        }

        if (threshold_coverGoal > 0.5) {
            threshold_coverGoal = threshold_coverGoal - 0.01;
        } else if (threshold_coverGoal < 0.5) {
            threshold_coverGoal = threshold_coverGoal + 0.01;
        }

        if (threshold_intercept > 0.5) {
            threshold_intercept = threshold_intercept - 0.01;
        } else if (threshold_intercept < 0.5) {
            threshold_intercept = threshold_intercept + 0.01;
        }

        if (threshold_go2Ball > 0.5) {
            threshold_go2Ball = threshold_go2Ball - 0.01;
        } else if (threshold_go2Ball < 0.5) {
            threshold_go2Ball = threshold_go2Ball + 0.01;
        }
    }

    private class AccionAcumulativeDistribution implements Comparable<AccionAcumulativeDistribution> {
        int action;
        double probValue;
        Vec2 vector;
        int player2pass;

        AccionAcumulativeDistribution(int action, double probValue, Vec2 vector) {
            this.action = action;
            this.probValue = probValue;
            this.vector = vector;
            this.player2pass = -1;
        }

        AccionAcumulativeDistribution(int action, double probValue, Vec2 vector, int player2Pass) {
            this.action = action;
            this.probValue = probValue;
            this.vector = vector;
            this.player2pass = player2Pass;
        }

        int getAction() {
            return action;
        }

        double getProbValue() {
            return probValue;
        }

        void setProbValue(double probValue) {
            this.probValue = probValue;
        }

        void executeAction() {
            switch (action) {
                case advance: {
                    //do advance
                    myRobotAPI.setDisplayString("advance");

                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.setSpeed(1.0d);

                    advance_count++;
                    break;
                }
                case retreat: {
                    //do retreat
                    myRobotAPI.setDisplayString("retreat");

                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.setSpeed(1.0d);

                    retreat_count++;
                    break;
                }
                case blockPlayer: {
                    //do Block
                    myRobotAPI.setDisplayString("Block");

                    Vec2 player2BlockVec = new Vec2(vector);
                    player2BlockVec.sub(myRobotAPI.getPosition());

                    Vec2 player2BlockDestination = new Vec2(vector);
                    player2BlockDestination.add(player2BlockVec);

                    myRobotAPI.surroundPoint(vector, player2BlockDestination);

                    blockPlayer_count++;
                    break;
                }
                case coverGoal: {
                    //do cover goal
                    myRobotAPI.setDisplayString("coverGoal");

                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.setSpeed(1.0);

                    coverGoal_count++;
                    break;
                }
                case intercept: {
                    //do Intercept
                    myRobotAPI.setDisplayString("intercept");

                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.setSpeed(1.0);

                    intercept_count++;
                    break;
                }
                case go2Ball: {
                    //do go to ball
                    myRobotAPI.setDisplayString("go2Ball");

                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.avoidCollisions();
                    myRobotAPI.setSpeed(1.0);

                    go2Ball_count++;
                    break;
                }
                case advanceBall: {
                    //do advance
                    myRobotAPI.setDisplayString("advanceBall");

                    Vec2 myPosAux = new Vec2(myRobotAPI.getPosition());
                    //advanceVector.normalize(1.0);
                    myPosAux.add(vector);

                    myRobotAPI.setBehindBall(myPosAux);
                    myRobotAPI.setSpeed(1.0);

                    advanceBall_count++;
                    break;
                }
                case pass2Player: {
                    //do pass
                    myRobotAPI.setDisplayString("Pass");

                    justPasses = true;
                    passedTimeStamp = System.currentTimeMillis();

                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.kick();

                    pass2Player_count++;
                    break;
                }
                case shoot: {
                    //do shoot
                    myRobotAPI.setDisplayString("shoot");

                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.kick();

                    shoot_count++;
                    break;
                }
                case passAssist: {
                    //do assist
                    myRobotAPI.setDisplayString("assist");

                    AssistMensaje msj = new AssistMensaje(vector);
                    msj.setSender(player);
                    msj.setReceiver(player2pass);
                    msj.setType(Message.Type.unicast);

                    manager.sendMessage(msj);
                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.kick();

                    passAssist_count++;
                    break;
                }
                case clear: {
                    //do clear
                    myRobotAPI.setDisplayString("clear");

                    myRobotAPI.setSteerHeading(vector.t);
                    myRobotAPI.kick();

                    clear_count++;
                    break;
                }
            }
        }

        @Override
        public int compareTo(AccionAcumulativeDistribution o) {
            return Double.compare(probValue, o.probValue);
        }
    }
}
