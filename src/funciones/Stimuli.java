package funciones;

import EDU.gatech.cc.is.util.Vec2;
import math.geom2d.Point2D;
import teams.ucmTeam.RobotAPI;

import java.awt.geom.Line2D;
import math.geom2d.polygon.Rectangle2D;
import math.geom2d.Vector2D;
import java.util.ArrayList;
import java.util.Collections;

/**
 * Created by angel on 22/01/2016.
 */
public final class Stimuli {

    public static final int GOILIE_TERRITORY = 0;
    public static final int DEFENSE_TERRITORY = 1;
    public static final int HALF_BACK_TERRITORY = 2;
    public static final int HALF_FORWARD_TERRITORY = 3;
    public static final int FORWARD_TERRITORY = 4;
    public static final int OPPONENTS_GOAL_TERRITORY = 5;

    static double s_score = 0.5;

    static double s_defense = 2.0;

    static double s_competition = 0.7;
    static double s_competition1 = 3.0;
    static double s_competition2 = 3.0;

    static double s_situation = 1.3;

    static double s_coverage = 30.0;

    static double s_penetration1 = 0.28;
    static double s_penetration2 = 0.16;
    static double s_penetration3 = 0.16;

    static double s_shooting1 = 0.33;
    static double s_shooting2 = 0.16;
    static double s_shooting3 = 0.5;
    static double penalizacion_shooting = 0.4;

    static double s_intercep1 = 0.13;
    static double s_intercep2 = 0.05;
    static double s_intercep3 = 0.04;

    static double s_keepGoal = 0.05d;


    /*
     * WITH BALL STIMULI
     */

    public static double advanceStimuliBall (RobotAPI api, Vec2 advanceVector) {
        double vuelta = 0.0d;
        double sigma_sn = 0.30d;

        int nextTerr = getPlayersActualTerritory(api.getPosition(), api.getFieldSide());

        if (nextTerr != 5) {
            //get next territory
            nextTerr = (nextTerr + 1);

            double factorNextTer = situationFactor(getMatesFieldCoordinates(api),
                                                    getOpponentsFieldCoordinates(api),
                                                    nextTerr,
                                                    api.getFieldSide());
            double defenseFactor = defenseFactor(api.toFieldCoordinates(api.getBall()),
                                                api.toFieldCoordinates(api.getOurGoal()),
                                                api.toFieldCoordinates(api.getOpponentsGoal()));

            Vec2 p1 = new Vec2();
            Vec2 p2 = new Vec2();
            double[] coverageFactor = coverageFactor(api.getPosition(),
                                                    api.toFieldCoordinates(api.getOpponentsGoal()),
                                                    getOpponentsFieldCoordinates(api),
                                                    p1,
                                                    p2);

            penetrationFactor(api.getPosition(),p1,p2,coverageFactor[1],advanceVector);

            vuelta = defenseFactor * (sigma_sn * factorNextTer + (1 - sigma_sn) * coverageFactor[0]);
        } else {
            Vec2 p1 = new Vec2();
            Vec2 p2 = new Vec2();
            double[] coverageFactor = coverageFactor(api.getPosition(),
                    api.toFieldCoordinates(api.getOpponentsGoal()),
                    getOpponentsFieldCoordinates(api),
                    p1,
                    p2);

            vuelta = penetrationFactor(api.getPosition(),p1,p2,coverageFactor[1],advanceVector);
        }
        return vuelta;
    }

    public static double passToPlayerStimuli (RobotAPI playeri,  Vec2 playerjPos, double p2Angle, Vec2 passVector) {
        double sigma = 1.5d;
        double sigma_c = 0.4d;
        double sigma_d = 0.25;
        double sigma_s = 0.1d;
        double sigma_cp = 0.25d;

        double nu = 10.0d;
        double beta = 15.0d;


        //calculate line lb
        Vec2 mePos = playeri.toFieldCoordinates(playeri.getPosition());
        Vec2 p2Pos = new Vec2(playerjPos);

        Vec2 lb = new Vec2(mePos);
        lb.sub(p2Pos);

        //gettings distance
        double lbDis = lb.r;

        //getting half point
        Vector2D vlb = new Vector2D(lb.x, lb.y);
        vlb = vlb.normalize();

        //pass vector
        passVector.setx(vlb.x());
        passVector.sety(vlb.y());

        vlb.times(lbDis/2.0d);
        Vector2D vmePos = new Vector2D(mePos.x, mePos.y);
        vmePos.plus(vlb);

        Vec2 halfpoint = new Vec2(vmePos.x(), vmePos.y());

        //getting coverages factor
        double[] pjcoverage = coverageFactor(p2Pos,
                                             playeri.toFieldCoordinates(playeri.getOpponentsGoal()),
                                             getOpponentsFieldCoordinates(playeri),
                                             new Vec2(),
                                             new Vec2());

        double[] picoverage = coverageFactor(playeri.getPosition(),
                                            playeri.toFieldCoordinates(playeri.getOpponentsGoal()),
                                            getOpponentsFieldCoordinates(playeri),
                                            new Vec2(),
                                            new Vec2());

        //calculating line playeri playerj
        math.geom2d.line.Line2D line = new math.geom2d.line.Line2D(mePos.x, mePos.y, p2Pos.x, p2Pos.y);

        Vec2 piClossestOpp = playeri.toFieldCoordinates(playeri.getClosestOpponent());
        Vec2 pjClossestOpp = getClosestOpponent(playeri, p2Pos);//playerj.toFieldCoordinates(playerj.getClosestOpponent());

        Vec2 opponent;
        if (line.distance(piClossestOpp.x,piClossestOpp.y) > line.distance(pjClossestOpp.x, pjClossestOpp.y)) {
            opponent = pjClossestOpp;
        } else {
            opponent = piClossestOpp;
        }

        double competition = competitionFactor(playeri.toFieldCoordinates(playeri.getBall()),
                                                opponent,p2Pos, p2Angle);
        double pjsituation = situationFactor(getMatesFieldCoordinates(playeri),
                                             getOpponentsFieldCoordinates(playeri),
                                             getPlayersActualTerritory(p2Pos, playeri.getFieldSide()),
                                             playeri.getFieldSide());

       return (sigma_c * (1.0d/(1.0d + Math.exp(-nu * (pjcoverage[0]/picoverage[0]) + beta))) + sigma_d * Math.exp(-sigma*lbDis) + sigma_cp*competition + sigma_s*pjsituation);
    }

    private static Vec2 getClosestOpponent(RobotAPI api, Vec2 me) {
        Vec2[] opponents = getOpponentsFieldCoordinates(api);

        Vec2 clossest = opponents[0];
        Vec2 closest2me = new Vec2(me);
        closest2me.sub(clossest);
        for (int i = 1; i < opponents.length; i++) {
            Vec2 act2me = new Vec2(me);
            act2me.sub(opponents[i]);
            if (act2me.r < closest2me.r) {
                clossest = opponents[i];
                closest2me = new Vec2(me);
                closest2me.sub(clossest);
            }
        }
        return clossest;
    }

    public static double[] shootStimuli (RobotAPI me, Vec2 shotVector, Vec2 assistVector, Vec2 assistPlayer) {
        double[] stimuli = new double[2];

        double shoot = shootingFactor(me.getPosition(),
                                        me.toFieldCoordinates(me.getOpponentsGoal()),
                                        getOpponentsFieldCoordinates(me),
                                        shotVector);

        Vec2 p1 = new Vec2();
        Vec2 p2 = new Vec2();
        double[] coverage = coverageFactor(me.getPosition(),
                me.toFieldCoordinates(me.getOpponentsGoal()),
                getOpponentsFieldCoordinates(me),
                p1, p2);


        double penetrationAssist = penetrationAssistfactor(me.getPosition(),
                                    me.toFieldCoordinates(me.getOpponentsGoal()),
                                    getMatesFieldCoordinates(me),
                                    p1, p2, coverage[1], coverage[2],
                                    assistVector,
                                    assistPlayer);

        stimuli[0] = shoot;
        stimuli[1] = penetrationAssist;
        return stimuli;
    }

    public static double clearStimuli (RobotAPI api, Vec2 clearVec) {
        double defense = defenseFactor(api.toFieldCoordinates(api.getBall()),
                                        api.toFieldCoordinates(api.getOurGoal()),
                                        api.toFieldCoordinates(api.getOpponentsGoal()));

        int actualTerr = getPlayersActualTerritory(api.getPosition(), api.getFieldSide());
        double situation = situationFactor(getMatesFieldCoordinates(api),
                                            getOpponentsFieldCoordinates(api),
                                            actualTerr,
                                            api.getFieldSide());

        Vec2 p2 = new Vec2();
        Vec2 p1 = new Vec2();
        double[] coverage = coverageFactor(api.getPosition(),
                                            api.toFieldCoordinates(api.getOpponentsGoal()),
                                            getOpponentsFieldCoordinates(api), p1, p2);
        penetrationFactor(api.getPosition(),p1,p2, coverage[1], clearVec);

        return (defense * (1 - situation));
    }

    //WITHOUT BALL STIMULI

    public static double advanceStimuli(RobotAPI api, Vec2 advanceVector) {
        double stimuli = 0.0d;
        double sigma_s = 0.5d;

        int actualTerr = getPlayersActualTerritory(api.getPosition(),api.getFieldSide());

        System.out.println("territory: " + actualTerr);
        if (actualTerr != 5) {
            int nextTerr = actualTerr + 1;
            double actualSituationFactor = situationFactor(getMatesFieldCoordinates(api),
                                                            getOpponentsFieldCoordinates(api),
                                                            actualTerr, api.getFieldSide());

            double nextSituationFactor = situationFactor(getMatesFieldCoordinates(api),
                                                        getOpponentsFieldCoordinates(api),
                                                        nextTerr, api.getFieldSide());

            double defense = defenseFactor(api.toFieldCoordinates(api.getBall()),
                                            api.toFieldCoordinates(api.getOurGoal()),
                                            api.toFieldCoordinates(api.getOpponentsGoal()));

            stimuli = Math.min((1 - defense) * (1.0 / (1.0 + Math.exp(-sigma_s*(nextSituationFactor - actualSituationFactor)))), 1.0);

            //calculate vector sign
            Vec2 advance = calculateadvanceVector(api, true);
            if (Math.signum(advance.x) != Math.signum(api.toFieldCoordinates(api.getOpponentsGoal()).x)) {
                advance.setx(-1.0 * advance.x);
            }

            advanceVector.setx(advance.x);
            advanceVector.sety(advance.y);
        }
        return stimuli;
    }

    public static double retreatStimuli(RobotAPI api, Vec2 advanceVector) {
        double stimuli = 0.0d;
        double sigma = 5.0d;

        int actualTerr = getPlayersActualTerritory(api.getPosition(),api.getFieldSide());
        if (actualTerr != 0) {
            int prevTerr = (actualTerr - 1);

            double actualSituationFactor = situationFactor(getMatesFieldCoordinates(api),
                                                            getOpponentsFieldCoordinates(api),
                                                            actualTerr, api.getFieldSide());

            double prevSituationFactor = situationFactor(getMatesFieldCoordinates(api),
                                                            getOpponentsFieldCoordinates(api),
                                                            prevTerr, api.getFieldSide());
            double defense = defenseFactor(api.toFieldCoordinates(api.getBall()),
                                            api.toFieldCoordinates(api.getOurGoal()),
                                            api.toFieldCoordinates(api.getOpponentsGoal()));

            stimuli = Math.min(defense * ( 1.0 / (1 + Math.exp(-sigma * (prevSituationFactor - actualSituationFactor)))), 1.0);

            Vec2 advance = calculateadvanceVector(api, false);
            if (Math.signum(advance.x) != Math.signum(api.toFieldCoordinates(api.getOurGoal()).x)) {
                advance.setx(-1.0 * advance.x);
            }

            advanceVector.setx(advance.x);
            advanceVector.sety(advance.y);
        }
        return stimuli;
    }

    public static double blockPlayerStimuli (RobotAPI player_i, Vec2 player_j, Vec2 player2Block) {
        double sigma_c = 1.0d;
        double sigma_a = 0.4d;

        double competitionFactor_i_j = competitionFactor(player_i.toFieldCoordinates(player_i.getBall()),
                                                    player_j, player_i.getPosition(), player_i.getSteerHeading());
        Vec2 p1 = new Vec2();
        Vec2 p2 = new Vec2();
        double[] coverage_j = coverageFactor(player_j,
                                            player_i.toFieldCoordinates(player_i.getOurGoal()),
                                            getMatesFieldCoordinates(player_i),p1,p2);
        Vec2 auxPlayer2Block = new Vec2();
        double penetrationAssist_j = penetrationAssistfactor(player_j,
                                                            player_i.toFieldCoordinates(player_i.getOurGoal()),
                                                            getOpponentsFieldCoordinates(player_i),
                                                            p1, p2, coverage_j[1], coverage_j[2], new Vec2(), auxPlayer2Block);

        double penetrationFactor_j = penetrationFactor(player_j, p1, p2, coverage_j[1], new Vec2());

        if ( penetrationAssist_j < penetrationFactor_j) {
            auxPlayer2Block = player_j;
        }

        player2Block.setx(auxPlayer2Block.x);
        player2Block.sety(auxPlayer2Block.y);

        return (sigma_c * (1 - competitionFactor_i_j)*(sigma_a*penetrationAssist_j + (1-sigma_a)*penetrationFactor_j));
    }

    public static double coverGoalStimuli (RobotAPI me, Vec2 coverVec) {
        double sigma_d = 0.2d;

        double defenseFactor = defenseFactor(me.toFieldCoordinates(me.getBall()),
                                            me.toFieldCoordinates(me.getOurGoal()),
                                            me.toFieldCoordinates(me.getOpponentsGoal()));

        double keepGoalFactor = keepGoalFactor(getMatesFieldCoordinates(me),me.getFieldSide());

        //calculating coverVec
        Vec2 me2goal = me.getOurGoal();
        Vector2D playerToGoal = new Vector2D(me2goal.x, me2goal.y);
        playerToGoal = playerToGoal.normalize();
        playerToGoal.times(5.0d);

        Vector2D auxCoverVec = new Vector2D(playerToGoal.x(), playerToGoal.y());
        Vec2[] mates = me.getTeammates();
        Rectangle2D goalArea = goalArea(me.getFieldSide());
        for (Vec2 mate : mates) {
            Vec2 matePossition = me.toFieldCoordinates(mate);
            if (goalArea.contains(matePossition.x, matePossition.y)) {
                Vector2D mateVector = new Vector2D(mate.x, mate.y);
                mateVector = mateVector.normalize();
                mateVector.times(-1.0);

                auxCoverVec.plus(mateVector);
            }
        }

        coverVec.setx(auxCoverVec.x());
        coverVec.sety(auxCoverVec.y());
        return ((1 - sigma_d) * defenseFactor + sigma_d*keepGoalFactor);
    }

    public static double interceptStimuli (RobotAPI me, Vec2 mainAttacker, Vec2 interceptVector) {
        return interceptFactor(me.getPosition(),
                                me.toFieldCoordinates(me.getOurGoal()),
                                mainAttacker, me.getFieldSide(), interceptVector);
    }

    public static double goToBallStimuli (RobotAPI api, Vec2 ballVec) {
        double sigma_b = 3.0d;
        Vec2 ball = api.getBall();

        ballVec.setx(ball.x);
        ballVec.sety(ball.y);

        return Math.min(Math.exp(-sigma_b * ball.r + 1.0), 1.0);
    }

    static Vec2 calculateadvanceVector(RobotAPI api, boolean advance) {
        Vec2 player = api.getPosition();

        Vec2[] opponents;
        if (advance) {
            ArrayList<Vec2> aux = ahead(new Vec2(player), getOpponentsFieldCoordinates(api), api.toFieldCoordinates(api.getOpponentsGoal()));
            opponents = new Vec2[aux.size()];
            aux.toArray(opponents);
        } else {
            ArrayList<Vec2> aux = ahead(new Vec2(player), getOpponentsFieldCoordinates(api), api.toFieldCoordinates(api.getOurGoal()));
            opponents = new Vec2[aux.size()];
            aux.toArray(opponents);
        }

        Vec2 advanceVector;
        if (advance) {
            advanceVector = new Vec2(api.getOpponentsGoal());
        } else {
            advanceVector = new Vec2(api.getOurGoal());
        }
        advanceVector.sub(player);
        advanceVector.normalize(1.0);
        advanceVector.setx(advanceVector.x * 5);
        advanceVector.sety(advanceVector.y * 5);

        int oppSign = advance ? -1 : 1;
        for (int i = 0; i < opponents.length; i++) {
            Vec2 actualV = new Vec2(opponents[i]);
            actualV.sub(player);
            actualV.normalize(1.0);

            advanceVector.setx(advanceVector.x * oppSign);
            advanceVector.sety(advanceVector.y * oppSign);
            advanceVector.add(actualV);
        }

        Vec2[] teamMates;
        if (advance) {
            ArrayList<Vec2> aux = ahead(player, getMatesFieldCoordinates(api), api.toFieldCoordinates(api.getOpponentsGoal()));
            teamMates = new Vec2[aux.size()];
            aux.toArray(teamMates);
        } else {
            ArrayList<Vec2> aux = ahead(player, getMatesFieldCoordinates(api), api.toFieldCoordinates(api.getOurGoal()));
            teamMates = new Vec2[aux.size()];
            aux.toArray(teamMates);
        }

        oppSign = advance ? 1 : -1;
        for (int i = 0; i < teamMates.length; i++) {
            Vec2 actualV = new Vec2(teamMates[i]);
            actualV.sub(player);
            actualV.normalize(1.0);

            advanceVector.setx(advanceVector.x * oppSign);
            advanceVector.sety(advanceVector.y * oppSign);
            advanceVector.add(actualV);
        }

        return advanceVector;
    }

    //FACTORS

    public static double scoreFactor (RobotAPI api) {
        int mScore = api.getMyScore();
        int oScore = api.getOpponentScore();

        double vuelta = 1 / (1 + Math.exp(s_score * (mScore - oScore)));
        return vuelta;
    }

    public static double timeFactor (RobotAPI api) {
        long timeRemaining = api.getMatchRemainingTime();
        long timeTotal = api.getMatchTotalTime();

        double vuelta = 1 - (timeTotal - timeRemaining) / timeTotal;
        return vuelta;
    }

    static double defenseFactor(Vec2 ball, Vec2 ourGoal, Vec2 oppGoal) {
        double d1 = ball.distance(oppGoal);
        double d2 = ball.distance(ourGoal);

        return  1.0 / (1.0 + Math.exp(-s_defense * (d1 - d2)));
    }

    static double competitionFactor(Vec2 ball, Vec2 opponent, Vec2 me, double myAngle) {
        Vec2 ballOpponentV = new Vec2(ball);
        ballOpponentV.sub(opponent);
        double d2 = ballOpponentV.r;

        Vec2 ballPlayerV = new Vec2(ball);
        ballPlayerV.sub(me);
        double d1 = ballPlayerV.r;

        Vec2 plainV = new Vec2(1,0);
        double sigma1 = Math.abs(plainV.angle(ballPlayerV) - myAngle) * (180/Math.PI);
        double sigma2 = Math.abs(plainV.angle(ballOpponentV)) * (180/Math.PI);

        return s_competition * Math.exp(-s_competition1 * (d1/d2)) + (1.0 - s_competition) * Math.exp(-s_competition2*(sigma1/sigma2));
    }

    static double situationFactor(Vec2[] mates, Vec2[] opponents, int territory, int myField) {
        Rectangle2D terrArea = getDimensionTerritory(myField, territory);

        int teamMates = 0;
        for (Vec2 v : mates) {
            Vec2 point = v;
            if (terrArea.contains(point.x ,point.y)) {
                teamMates++;
            }
        }

        int opponts = 0;
        for (Vec2 v : opponents) {
            Vec2 point = v;
            if (terrArea.contains(point.x ,point.y)) {
                opponts++;
            }
        }
        return (1.0 / (1.0 + Math.exp(-((teamMates - opponts)/s_situation))));
    }

    static double[] coverageFactor (Vec2 me, Vec2 opponentsGoal, Vec2[] opponents , Vec2 selectedPoint1, Vec2 selectedPoint2) {
        ArrayList<Vec2> aheadOpps = ahead(me, opponents, opponentsGoal);
        double[] vuelta = new double[3];

        if (!aheadOpps.isEmpty()) {
            Vec2 player = new Vec2(me);
            Vec2 goal = new Vec2(opponentsGoal);

            Vec2 upper = new Vec2(aheadOpps.get(aheadOpps.size() - 1));
            upper.sety(0.7625);
            aheadOpps.add(upper);

            Vec2 lower = new Vec2(aheadOpps.get(0));
            lower.sety(-0.7625);
            aheadOpps.add(0, lower);

            double maxAngleA = 0.0;
            double maxAngleB = 0.0;
            Vec2 maxda1 = new Vec2();
            Vec2 maxda2 = new Vec2();
            for (int i = 1; i < aheadOpps.size(); i++) {
                Vec2 da1 = new Vec2(aheadOpps.get(i - 1));
                da1.sub(player);

                Vec2 da2 = new Vec2(aheadOpps.get(i));
                da2.sub(player);
                double angleA = da1.angle(da2);

                Vec2 db1 = new Vec2(aheadOpps.get(i - 1));
                db1.sub(goal);
                Vec2 db2 = new Vec2(aheadOpps.get(i));
                db2.sub(goal);
                double angleB = db1.angle(db2);

                if ((maxAngleA + maxAngleB) < (angleA + angleB)) {
                    maxAngleA = angleA;
                    maxAngleB = angleB;

                    maxda1 = new Vec2(aheadOpps.get(i - 1));
                    maxda2 = new Vec2(aheadOpps.get(i));
                }
            }
            maxAngleA = maxAngleA * (180.0/Math.PI);
            maxAngleB = maxAngleB * (180.0/Math.PI);

            vuelta[0] = 1.0 - Math.exp(-(maxAngleA + maxAngleB)/s_coverage);
            vuelta[1] = maxAngleA;
            vuelta[2] = maxAngleB;

            //selectedPoints.add(maxda1);
            selectedPoint1.setx(maxda1.x);
            selectedPoint1.sety(maxda1.y);

            selectedPoint2.setx(maxda2.x);
            selectedPoint2.sety(maxda2.y);
        } else {
            Vec2 player = new Vec2(me);
            Vec2 goal =new Vec2(opponentsGoal);

            Vec2 goalPlayer = new Vec2(goal);
            goalPlayer.sety(player.y);

            double dist = player.distance(goalPlayer);

            Vec2 upper = new Vec2();
            upper.setx(player.x + dist/2.0);
            upper.sety(0.7625);
            Vec2 lower = new Vec2(upper);
            lower.sety(-0.7625);

            Vec2 da1 = new Vec2(lower);
            da1.sub(player);
            Vec2 da2 = new Vec2(upper);
            da2.sub(player);

            Vec2 db1 = new Vec2(lower);
            db1.sub(goal);
            Vec2 db2 = new Vec2(upper);
            db2.sub(goal);

            double maxAngleA = da1.angle(da2) * (180.0/Math.PI);
            double maxAngleB = db1.angle(db2) * (180.0/Math.PI);

            vuelta[0] = 1 - Math.exp(-(maxAngleA + maxAngleB)/s_coverage);
            vuelta[1] = maxAngleA;
            vuelta[2] = maxAngleB;

            //selectedPoints.add(maxda1);
            selectedPoint1.setx(lower.x);
            selectedPoint1.sety(lower.y);

            selectedPoint2.setx(upper.x);
            selectedPoint2.sety(upper.y);
        }

        return vuelta;
    }

    static double penetrationFactor (Vec2 me, Vec2 p1, Vec2 p2, double angleA, Vec2 penetrationvector) {
        double vuelta = 0.0d;
        Vec2 player = new Vec2(me);

        Vec2 vp1 = new Vec2(p1);
        vp1.sub(player);

        Vec2 vp2 = new Vec2(p2);
        vp2.sub(player);

        //closest distance from opponent to player
        double d3 = Math.max(vp1.r, vp2.r);

        //calculate line la: player -> laEndp
        vp2.rotate((angleA * (Math.PI/180.0)) / 2.0d);
        //return penetration vector
        penetrationvector.setx(vp2.x);
        penetrationvector.sety(vp2.y);

        Vec2 laEndp = new Vec2(player);
        laEndp.add(vp2);

        //closest distance from opponent to penetration line la
        double d1 = Math.max(Line2D.Double.ptLineDist(player.x, player.y, laEndp.x, laEndp.y, p1.x, p1.y),
                Line2D.Double.ptLineDist(player.x, player.y, laEndp.x, laEndp.y, p2.x, p2.y));

        // penetration factor
        vuelta = 1 - 0.5 * Math.exp((-d3) / s_penetration1) - 0.5 * Math.exp((-d3 * s_penetration2 - d1 * s_penetration1) / (s_penetration1 * s_penetration2));

        return vuelta;
    }

    static double penetrationAssistfactor(Vec2 me, Vec2 oppGoal, Vec2[] mates, Vec2 p1, Vec2 p2, double angleA, double angleB, Vec2 penetrationvector, Vec2 assistPlayer) {
        double vuelta = 0.0;
        Vec2 player = new Vec2(me);
        Vec2 goal = new Vec2(oppGoal);

        Vec2 vda1 = new Vec2(p1);
        vda1.sub(player);
        Vec2 vda2 = new Vec2(p2);
        vda2.sub(player);

        //closest distance from opponent to player
        double d3 = Math.max(vda1.r, vda2.r);

        //calculate line la: player -> laEndp
        vda2.rotate(angleA/2.0d);

        Vec2 laEndp = new Vec2(player);
        laEndp.add(vda2);

        //calculate line lb: player -> lbEndp
        Vec2 vdb2 = new Vec2(p2);
        vdb2.sub(goal);
        vdb2.rotate(-angleB/2.0d);

        Vec2 lbEndp = new Vec2(goal);
        lbEndp.add(vdb2);

        //calculate intersection points between la and lb
        math.geom2d.Point2D intersect = math.geom2d.line.StraightLine2D.getIntersection(new math.geom2d.Point2D(player.x, player.y),
                new math.geom2d.Point2D(laEndp.x,laEndp.y),
                new math.geom2d.Point2D(goal.x, goal.y),
                new math.geom2d.Point2D(lbEndp.x, lbEndp.y));

        if (intersect != null) {
            //distance from closest mate to intersect d2
            Vec2 intersectVec2 = new Vec2(intersect.x(), intersect.y());
            Vec2 assist = closesMateToPoint(mates, intersectVec2);

            assistPlayer.setx(assist.x);
            assistPlayer.sety(assist.y);
            penetrationvector.setx(assist.x);
            penetrationvector.sety(assist.y);

            double d2 = assist.distance(intersectVec2);

            // penetration factor
            vuelta = Math.exp((-s_penetration3 * d3 - s_penetration1 * d2) / (s_penetration1 * s_penetration3));
        }
        return vuelta;
    }

    static double shootingFactor(Vec2 me, Vec2 opponentsGoal, Vec2[] opponents, Vec2 shootVector) {
        double sigma_1 = 10.0;
        double alfa_1 = 4.0;
        double sigma_2 = 85.0;
        double alfa_2 = 10.0;

        Vec2 player = new Vec2(me);
        Vec2 goal = new Vec2(opponentsGoal);

        //calculating shoot vector
        Vec2 auxSv = new Vec2(goal);
        auxSv.sub(player);

        double d1 = auxSv.r;
        Vec2 opp = new Vec2();
        double d2 = closestOpponentToLine(opponents, player, goal, opp);

        shootVector.setx(auxSv.x);
        shootVector.sety(auxSv.y);

        return (Math.min((1.0 / (1.0 + Math.exp(-sigma_2*d2 + alfa_2))),1.0) * Math.min((Math.exp(-sigma_1*d1 + alfa_1)),1.0));
    }

    static double interceptFactor (Vec2 me, Vec2 myGoal, Vec2 mainAttaker, int fieldSide, Vec2 interceptVector) {
        double vuelta = 0.0d;
        Vec2 goal =new Vec2(myGoal);

        //calculating goal line
        math.geom2d.line.Line2D goalLine = new math.geom2d.line.Line2D(-fieldSide*1.37d,0.7625d,-fieldSide*1.37d,-0.7625d);

        //calculating shoot vector
        Vec2 shootVector = new Vec2(myGoal);
        shootVector.sub(mainAttaker);

        //calculating 2 point of shoot line
        Vec2 shootEnd = new Vec2(mainAttaker);
        shootEnd.add(shootVector);

        math.geom2d.line.Line2D shootLine = new math.geom2d.line.Line2D(mainAttaker.x, mainAttaker.y, shootEnd.x, shootEnd.y);

        //intersect point
        Point2D goalShoot = goalLine.intersection(shootLine);

        if (goalShoot != null) {
            Vec2 vShoot = new Vec2(goalShoot.x(), goalShoot.y());
            vShoot.sub(me);

            double d1 = vShoot.r;
            double d3 = Math.abs(goal.y - goalShoot.y());

            double d2 = Line2D.Double.ptLineDist(mainAttaker.x, mainAttaker.y, shootEnd.x, shootEnd.y, me.x, me.y);

            interceptVector.setx(vShoot.x);
            interceptVector.sety(vShoot.y);

            vuelta = Math.exp((-d1 * s_intercep2 * s_intercep3 - d2 * s_intercep1 * s_intercep3 - d3 * s_intercep1 * s_intercep2) / (s_intercep1 * s_intercep2 * s_intercep3));
        }

        return vuelta;
    }

    static double keepGoalFactor (Vec2[] mates, int fieldSide) {
        double sigma = 0.7;
        Rectangle2D area = goalArea(fieldSide);
        int numPlayerArea = 0;

        for(Vec2 mate: mates) {
            if (area.contains(mate.x,mate.y)) {
                numPlayerArea++;
            }
        }
        return Math.exp(-sigma * numPlayerArea);
    }

    //UTILITIES

    static Rectangle2D getDimensionTerritory(int myField, int territory) {
        Rectangle2D vuelta;

        switch (territory) {
            case GOILIE_TERRITORY:
                vuelta = golieTerritory(myField);
                break;
            case DEFENSE_TERRITORY:
                vuelta = defenseTerritory(myField);
                break;
            case HALF_BACK_TERRITORY:
                vuelta = halfBackTerritory(myField);
                break;
            case HALF_FORWARD_TERRITORY:
                vuelta = halfForwardTerritory(myField);
                break;
            case FORWARD_TERRITORY:
                vuelta = forwardTerritory(myField);
                break;
            case OPPONENTS_GOAL_TERRITORY:
                vuelta = opponentsGoalTerritory(myField);
                break;
            default:
                vuelta = new Rectangle2D();
                break;
        }
        return vuelta;
    }

    static Vec2 closesMateToPoint (Vec2[] mates, Vec2 point) {
        Vec2 closest = mates[0];
        for (int i = 1; i < mates.length; i++) {
            Vec2 nextMate = mates[i];
            if (closest.distance(point) < nextMate.distance(point)) {
                closest = nextMate;
            }
        }
        return closest;
    }

    static double closestOpponentToLine (Vec2[] opps, Vec2 point1, Vec2 point2, Vec2 opp) {
        int max = 0;
        double maxDis = Line2D.Double.ptLineDist(point1.x, point1.y, point2.x, point2.y, opps[0].x, opps[0].y);
        for (int i = 1; i < opps.length; i++) {
            double actualDis = Line2D.Double.ptLineDist(point1.x, point1.y, point2.x, point2.y, opps[i].x, opps[i].y);
            if (actualDis > maxDis) {
                maxDis = actualDis;
                max = i;
            }
        }
        opp.setx(opps[max].x);
        opp.sety(opps[max].y);

        return maxDis;
    }

    static ArrayList<Vec2> ahead (Vec2 me, Vec2[] opponents, Vec2 oppGoal) {
        ArrayList<Vec2> vuelta = new ArrayList<>();

        Vec2 me2OppG = new Vec2(oppGoal);
        me2OppG.sub(me);

        for(Vec2 opp: opponents) {
            Vec2 dist = new Vec2(opp);
            dist.sub(oppGoal);

            if (dist.r < me2OppG.r) {
                vuelta.add(opp);
            }
        }

        Collections.sort(vuelta, new Vec2YComparator());
        return vuelta;
    }

    static int getPlayersActualTerritory(Vec2 player, int myField) {
        int territory = -1;
        boolean finded = false;
        for (int i = 0; !finded && i < 6; i++) {
            Rectangle2D actualTerr = getTerritory(myField, i);
            finded = actualTerr.contains(player.x, player.y);
            if (finded) {
                territory = i;
            }
        }
        return territory;
    }

    static Rectangle2D getTerritory (int myField, int territory) {
        Rectangle2D finded = null;
        switch(territory) {
            case GOILIE_TERRITORY:
                finded = golieTerritory(myField);
                break;
            case DEFENSE_TERRITORY:
                finded = defenseTerritory(myField);
                break;
            case HALF_BACK_TERRITORY:
                finded = halfBackTerritory(myField);
                break;
            case HALF_FORWARD_TERRITORY:
                finded = halfForwardTerritory(myField);
                break;
            case FORWARD_TERRITORY:
                finded = forwardTerritory(myField);
                break;
            case OPPONENTS_GOAL_TERRITORY:
                finded = opponentsGoalTerritory(myField);
                break;
        }
        return finded;
    }

    public static Rectangle2D golieTerritory (int field) {
        return new Rectangle2D(new Point2D(field * 1.145, 0.7625),new Point2D(field * 1.35, -0.7625));
    }

    public static Rectangle2D defenseTerritory (int field) {
        return new Rectangle2D(new Point2D(field * 0.5725, 0.7625),new Point2D(field * 1.145, -0.7625));
    }

    public static Rectangle2D halfBackTerritory (int field) {
        return new Rectangle2D(new Point2D(0, 0.7625),new Point2D(field * 0.5725, -0.7625));
    }

    public static Rectangle2D opponentsGoalTerritory (int field) {
        return new Rectangle2D(new Point2D(-field * 1.145, 0.7625),new Point2D(-field * 1.35, -0.7625));
    }

    public static Rectangle2D forwardTerritory (int field) {
        return new Rectangle2D(new Point2D(-field * 0.5725, 0.7625),new Point2D(-field * 1.145, -0.7625));
    }

    public static Rectangle2D halfForwardTerritory (int field) {
        return new Rectangle2D(new Point2D(0, 0.7625),new Point2D(-field * 0.5725, -0.7625));
    }

    public static Rectangle2D goalArea (int fieldSide) {
        return new Rectangle2D(new Point2D(fieldSide * 1.145, 0.25), new Point2D(fieldSide * 1.35, -0.25));
    }

   public static Vec2[] getOpponentsFieldCoordinates(RobotAPI api) {
        Vec2[] opponents = api.getOpponents();
        Vec2[] oppField = new Vec2[opponents.length];

        for (int i = 0; i < opponents.length; i++) {
            oppField[i] = api.toFieldCoordinates(opponents[i]);
        }
        return oppField;
    }

   public static Vec2[] getMatesFieldCoordinates(RobotAPI api) {
        Vec2[] mates = api.getTeammates();
        Vec2[] matesField = new Vec2[mates.length];

        for (int i = 0; i < mates.length; i++) {
            matesField[i] = api.toFieldCoordinates(mates[i]);
        }
        return matesField;
    }
}
