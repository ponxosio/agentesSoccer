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

    static double s_defense = 3.0;

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

    public static double advanceStimuliBall (RobotAPI api, Vec2 advanceVector) {
        double vuelta = 0.0d;
        double sigma_sn = 0.30d;

        int actualTerr = getPlayersActualTerritory(api);

        if (actualTerr != 6) {
            //get next territory
            actualTerr = (actualTerr + 1) % 6;

            double factorNextTer = situationFactor(api, actualTerr);
            double defenseFactor = defenseFactor(api);

            ArrayList<Vec2> selectedPoints = new ArrayList<>();
            double[] coverageFactor = coverageFactor(api, selectedPoints);

            ArrayList<Vec2> penetrationVector = new ArrayList<>();
            penetrationFactor(api, selectedPoints.get(0), selectedPoints.get(1), coverageFactor[1], coverageFactor[2], penetrationVector);

            advanceVector.setx(penetrationVector.get(0).x);
            advanceVector.sety(penetrationVector.get(0).y);
            advanceVector.setr(penetrationVector.get(0).r);
            advanceVector.sett(penetrationVector.get(0).t);

            vuelta = defenseFactor * (sigma_sn * factorNextTer + (1 - sigma_sn) * coverageFactor[0]);
        }

        return vuelta;
    }

    public static double passToPlayerStimuli (RobotAPI me,  RobotAPI player2pass, Vec2 opponent, Vec2 passVector) {
        double stimuli;

        double sigma = 1.5d;
        double sigma_c = 0.4d;
        double sigma_d = 0.25;
        double sigma_s = 0.1d;
        double sigma_cp = 0.25d;

        double nu = 10.0d;
        double beta = 15.0d;


        //calculate line lb
        Vec2 mePos = me.toFieldCoordinates(me.getPosition());
        Vec2 p2Pos = player2pass.toFieldCoordinates(player2pass.getPosition());

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
        ArrayList<Vec2> points = new ArrayList<>();
        double[] p2coverage = coverageFactor(player2pass, points);
        double[] meCoverage = coverageFactor(me, points);

        double competition = competitionFactor(player2pass, opponent, halfpoint);
        double p2situation = situationFactor(player2pass, getPlayersActualTerritory(player2pass));

        stimuli = sigma_c * (1.0d/(1.0d + Math.exp(-nu * (p2coverage[0]/meCoverage[0]) + beta))) + sigma_d * Math.exp(-sigma*lbDis) + sigma_cp*competition + sigma_s*p2situation;

        return stimuli;
    }

    public static double shootStimuli (RobotAPI api, Vec2 shotVector) {
        double stimuli;
        double sigma_s = 0.5d;

        double defense = defenseFactor(api);

        ArrayList<Vec2> shootVectors = new ArrayList<>();
        double shoot = shootingFactor(api, shootVectors);
        shotVector.setx(shootVectors.get(0).x);
        shotVector.sety(shootVectors.get(0).y);

        ArrayList<Vec2> areaPoints = new ArrayList<>();
        double[] coverage = coverageFactor(api, areaPoints);

        ArrayList<Vec2> penetration = new ArrayList<>();
        double penetrationAssist = penetrationAssistfactor(api, areaPoints.get(0), areaPoints.get(1), coverage[1], coverage[2], penetration);

        stimuli = (1 - defense) * (sigma_s * shoot + (1-sigma_s)*(1-penetrationAssist));
        return stimuli;
    }

    public static double passAssistStimuli (RobotAPI api, Vec2 passVector) {
        double stimuli;
        double sigma_S = 0.5d;

        double defense = defenseFactor(api);

        ArrayList<Vec2> shootVectors = new ArrayList<>();
        double shoot = shootingFactor(api, shootVectors);

        ArrayList<Vec2> areaPoints = new ArrayList<>();
        double[] coverage = coverageFactor(api, areaPoints);

        ArrayList<Vec2> penetration = new ArrayList<>();
        double penetrationAssist = penetrationAssistfactor(api, areaPoints.get(0), areaPoints.get(1), coverage[1], coverage[2], penetration);
        passVector.setx(penetration.get(0).x);
        passVector.sety(penetration.get(0).y);

        stimuli = (1-defense) * ((1-sigma_S)*penetrationAssist + sigma_S*shoot);

        return stimuli;
    }

    public static double clearStimuli (RobotAPI api, Vec2 clearVec) {
        double stimuli;

        double defense = defenseFactor(api);

        int actualTerr = getPlayersActualTerritory(api);
        double situation = situationFactor(api, actualTerr);

        ArrayList<Vec2> areaPoints = new ArrayList<>();
        double[] coverage = coverageFactor(api, areaPoints);
        ArrayList<Vec2> penetration = new ArrayList<>();
        penetrationAssistfactor(api, areaPoints.get(0), areaPoints.get(1), coverage[1], coverage[2], penetration);

        clearVec.setx(penetration.get(0).x);
        clearVec.sety(penetration.get(0).y);

        stimuli = defense * (1 - situation);
        return stimuli;
    }

    public static double advanceStimuli(RobotAPI api, Vec2 advanceVector) {
        double stimuli = 0.0d;
        double sigma_s = 0.40d;
        double nu = 7.0d;
        double beta = 7.0d;

        int actualTerr = getPlayersActualTerritory(api);
        if (actualTerr != 6) {
            int nextTerr = (actualTerr + 1) % 6;
            double actualSituationFactor = situationFactor(api, actualTerr);
            double nextSituationFactor = situationFactor(api, nextTerr);
            double defense = defenseFactor(api);

            stimuli = sigma_s * (1.0d/ (1.0d + Math.exp(-nu * (nextSituationFactor/actualSituationFactor) + beta))) + (1-sigma_s)*(1- defense);

            Vec2 advance = calculateadvanceVector(api, true);

            advanceVector.setx(advance.x);
            advanceVector.sety(advance.y);
        }
        return stimuli;
    }

    public static double retreatStimuli(RobotAPI api, Vec2 advanceVector) {
        double stimuli = 0.0d;
        double sigma_s = 0.40d;
        double nu = 7.0d;
        double beta = 7.0d;

        int actualTerr = getPlayersActualTerritory(api);
        if (actualTerr != 6) {
            int prevTerr = (actualTerr - 1);
            prevTerr = prevTerr < 0 ? 0 : prevTerr;

            double actualSituationFactor = situationFactor(api, actualTerr);
            double prevSituationFactor = situationFactor(api, prevTerr);
            double defense = defenseFactor(api);

            stimuli = sigma_s * (1.0d/ (1.0d + Math.exp(-nu * (actualSituationFactor/prevSituationFactor) + beta))) + (1-sigma_s)*(defense);

            Vec2 advance = calculateadvanceVector(api, false);

            advanceVector.setx(advance.x);
            advanceVector.sety(advance.y);
        }
        return stimuli;
    }

    public static double blockPlayerStimuli (RobotAPI me, Vec2 opponent) {
        double stimuli;
        double sigma_c = 1.0d;
        double sigma_a = 0.4d;

        Vec2 ball = me.toFieldCoordinates(me.getBall());
        double competitionFactor = competitionFactor(me, opponent, ball);

        ArrayList<Vec2> areaPoints = new ArrayList<>();

        double[] coverage = coverageFactor(opponent, areaPoints);
        ArrayList<Vec2> penetration = new ArrayList<>();
        double penetrationAssist = penetrationAssistfactor(opponent, areaPoints.get(0), areaPoints.get(1), coverage[1], coverage[2], penetration);
        double penetrationFactor = penetrationFactor(opponent, areaPoints.get(0), areaPoints.get(1), coverage[1], coverage[2], penetration);

        stimuli = sigma_c * (1 - competitionFactor)*(sigma_a*penetrationAssist + (1-sigma_a)*penetrationFactor);

        return stimuli;
    }

    public static double coverGoalStimuli (RobotAPI me) {
        double sigma_d = 0.4d;

        double defenseFactor = defenseFactor(me);
        double keepGoalFactor = keepGoalFactor(me);

        double stimuli = (1 - sigma_d) * defenseFactor + sigma_d*keepGoalFactor;
        return stimuli;
    }

    public static double goToBallStimuli (RobotAPI api) {
        double sigma_b = 15.0d;
        Vec2 ball = api.getBall();

        return Math.exp(sigma_b * ball.r);
    }

    static Vec2 calculateadvanceVector(RobotAPI api, boolean advance) {
        Vec2 player = api.toFieldCoordinates(api.getPosition());
        Vec2[] opponents = api.getOpponents();

        Vec2 advanceVector = new Vec2();
        int oppSign = advance ? -1 : 1;
        for (int i = 0; i < opponents.length; i++) {
            Vec2 actualV = new Vec2(api.toEgocentricalCoordinates(opponents[i]));

            actualV.sub(player);
            actualV.setx(actualV.x * oppSign);
            actualV.sety(actualV.y * oppSign);

            advanceVector.add(actualV);
        }

        Vec2[] teamMates = api.getTeammates();
        oppSign = advance ? 1 : -1;
        for (int i = 0; i < teamMates.length; i++) {
            Vec2 actualV = new Vec2(api.toEgocentricalCoordinates(teamMates[i]));

            actualV.sub(player);
            actualV.setx(actualV.x * oppSign);
            actualV.sety(actualV.y * oppSign);

            advanceVector.add(actualV);
        }

        return advanceVector;
    }

    static double scoreFactor (RobotAPI api) {
        int mScore = api.getMyScore();
        int oScore = api.getOpponentScore();

        double vuelta = 1 / (1 + Math.exp(s_score * (mScore - oScore)));
        return vuelta;
    }

    static double timeFactor (RobotAPI api) {
        long timeRemaining = api.getMatchRemainingTime();
        long timeTotal = api.getMatchTotalTime();

        double vuelta = 1 - (timeTotal - timeRemaining) / timeTotal;
        return vuelta;
    }

    static double defenseFactor(RobotAPI api) {
        Vec2 ourGoal = api.toFieldCoordinates(api.getOurGoal());
        Vec2 oppGoal = api.toFieldCoordinates(api.getOpponentsGoal());
        Vec2 ball = api.toFieldCoordinates(api.getBall());

        double d1 = ball.distance(oppGoal);
        double d2 = ball.distance(ourGoal);

        return Math.exp(-s_defense*(d1/d2));
    }

    static double competitionFactor(RobotAPI our, Vec2 opponent, Vec2 point) {
        Vec2 ourP = our.toFieldCoordinates(our.getPosition());
        Vec2 oppP = opponent;

        Vec2 ourDir = new Vec2();
        ourDir.rotate(our.getSteerHeading());
        Vec2 oppDir = new Vec2();
        oppDir.rotate(our.getSteerHeading());

        Vec2 d1 = new Vec2(point);
        d1.sub(ourP);
        Vec2 d2 = new Vec2(point);
        d2.sub(oppP);

        double angle1 = ourDir.angle(d1);
        double angle2 = oppDir.angle(d2);

        return s_competition * Math.exp(-s_competition1 * (d1.t/d2.t)) + (1 - s_competition) * Math.exp(-s_competition2*(angle1/angle2));
    }

    static double situationFactor(RobotAPI api, int territory) {
        Rectangle2D terrArea = getDimensionTerritory(api, territory);
        Vec2[] mates = api.getTeammates();
        Vec2[] opp = api.getOpponents();

        int teamMates = 0;
        for (Vec2 v : mates) {
            Vec2 point = api.toFieldCoordinates(v);
            if (terrArea.contains(point.x ,point.y)) {
                teamMates++;
            }
        }

        int opponts = 0;
        for (Vec2 v : opp) {
            Vec2 point = api.toFieldCoordinates(v);
            if (terrArea.contains(point.x ,point.y)) {
                opponts++;
            }
        }
        return (1 / (1 + Math.exp(-((teamMates - opponts)/s_situation))));
    }

    static Rectangle2D getDimensionTerritory(RobotAPI api, int territory) {
        Rectangle2D vuelta;

        switch (territory) {
            case GOILIE_TERRITORY:
                vuelta = golieTerritory(api);
                break;
            case DEFENSE_TERRITORY:
                vuelta = defenseTerritory(api);
                break;
            case HALF_BACK_TERRITORY:
                vuelta = halfBackTerritory(api);
                break;
            case HALF_FORWARD_TERRITORY:
                vuelta = halfForwardTerritory(api);
                break;
            case FORWARD_TERRITORY:
                vuelta = forwardTerritory(api);
                break;
            case OPPONENTS_GOAL_TERRITORY:
                vuelta = opponentsGoalTerritory(api);
                break;
            default:
                vuelta = new Rectangle2D();
                break;
        }
        return vuelta;
    }

    static double[] coverageFactor (RobotAPI api, ArrayList<Vec2> selectedPoints) {
        ArrayList<Vec2> aheadOpps = ahead(api, api.getOpponents());
        double[] vuelta = new double[3];

        if (!aheadOpps.isEmpty()) {
            Vec2 player = api.toFieldCoordinates(api.getPosition());
            Vec2 goal = api.toFieldCoordinates(api.getOpponentsGoal());

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
            vuelta[0] = 1 - Math.exp(-(maxAngleA + maxAngleB)/s_coverage);
            vuelta[1] = maxAngleA;
            vuelta[2] = maxAngleB;
            selectedPoints.add(maxda1);
            selectedPoints.add(maxda2);
        } else {
            Vec2 player = api.toFieldCoordinates(api.getPosition());
            Vec2 goal = api.toFieldCoordinates(api.getOpponentsGoal());
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

            double maxAngleA = da1.angle(da2);
            double maxAngleB = db1.angle(db2);

            vuelta[0] = 1 - Math.exp(-(maxAngleA + maxAngleB)/s_coverage);
            vuelta[1] = maxAngleA;
            vuelta[2] = maxAngleB;
        }

        return vuelta;
    }

   /* public static Point2D.Float getIntersectionPoint(Line2D.Float line1, Line2D.Float line2) {
        if (! line1.intersectsLine(line2) ) return null;
        double px = line1.getX1(),
                py = line1.getY1(),
                rx = line1.getX2()-px,
                ry = line1.getY2()-py;
        double qx = line2.getX1(),
                qy = line2.getY1(),
                sx = line2.getX2()-qx,
                sy = line2.getY2()-qy;

        double det = sx*ry - sy*rx;
        if (det == 0) {
            return null;
        } else {
            double z = (sx*(qy-py)+sy*(px-qx))/det;
            if (z==0 ||  z==1) return null;  // intersection at end point!
            return new Point2D.Float(
                    (float)(px+z*rx), (float)(py+z*ry));
        }
    }*/

    static double penetrationFactor (RobotAPI api, Vec2 da1, Vec2 da2, double angleA, double angleB, ArrayList<Vec2> penetrationvectors) {
        double vuelta = 0.0d;


            Vec2 player = api.toFieldCoordinates(api.getPosition());

            Vec2 vda1 = new Vec2(da1);
            da1.sub(player);
            Vec2 vda2 = new Vec2(da2);
            da2.sub(player);

            //closest distance from opponent to player
            double d3 = Math.max(vda1.r, vda2.r);

            //calculate line la: player -> laEndp
            vda2.rotate(-(angleA / 2.0d));
            //return penetration vector
            penetrationvectors.add(new Vec2(vda2));

            Vec2 laEndp = new Vec2(player);
            laEndp.add(vda2);

            //closest distance from opponent to penetration line la
            double d1 = Math.max(Line2D.Double.ptLineDist(player.x, player.y, laEndp.x, laEndp.y, da1.x, da1.y),
                    Line2D.Double.ptLineDist(player.x, player.y, laEndp.x, laEndp.y, da2.x, da2.y));

        if (api.closestToBall()) {
            // penetration factor
            vuelta = 1 - 0.5 * Math.exp((-d3) / s_penetration1) - 0.5 * Math.exp((-d3 * s_penetration2 - d1 * s_penetration1) / (s_penetration1 * s_penetration2));
        }
        return vuelta;
    }

    static double penetrationAssistfactor(RobotAPI api, Vec2 da1, Vec2 da2, double angleA, double angleB, ArrayList<Vec2> penetrationvectors) {
        double vuelta;
        Vec2 player = api.toFieldCoordinates(api.getPosition());
        Vec2 goal = api.toFieldCoordinates(api.getOpponentsGoal());

        Vec2 vda1 = new Vec2(da1);
        da1.sub(player);
        Vec2 vda2 = new Vec2(da2);
        da2.sub(player);

        //closest distance from opponent to player
        double d3 = Math.max(vda1.r, vda2.r);

        //calculate line la: player -> laEndp
        vda2.rotate(-(angleA/2.0d));
        Vec2 laEndp = new Vec2(player);
        laEndp.add(vda2);

        //calculate line lb: player -> lbEndp
        Vec2 vdb2 = new Vec2(da2);
        vdb2.sub(goal);
        vdb2.rotate(-(angleB/2.0d));
        Vec2 lbEndp = new Vec2(goal);
        lbEndp.add(vdb2);

        //calculate intersection points between la and lb
        math.geom2d.Point2D intersect = math.geom2d.line.StraightLine2D.getIntersection(new math.geom2d.Point2D(player.x, player.y),
                new math.geom2d.Point2D(laEndp.x,laEndp.y),
                new math.geom2d.Point2D(goal.x, goal.y),
                new math.geom2d.Point2D(lbEndp.x, lbEndp.y));
        //distance from closest mate to intersect d2
        Vec2 intersectVec2 = new Vec2(intersect.x(), intersect.y());
        Vec2 assist = closesMateToPoint(api, intersectVec2);
        penetrationvectors.add(assist);
        double d2 = assist.distance(intersectVec2);

        // penetration factor
        vuelta = Math.exp((-s_penetration3*d3 - s_penetration1*d2)/(s_penetration1*s_penetration3));

        return vuelta;
    }

    static double shootingFactor (RobotAPI api, ArrayList<Vec2> shootVector) {
        double vuelta = 0.0d;


            Vec2 player = api.toFieldCoordinates(api.getPosition());
            Vec2 goal = api.toFieldCoordinates(api.getOpponentsGoal());

            Vec2 d1LineEnd = new Vec2(player);
            d1LineEnd.setx(goal.x);
            math.geom2d.line.Line2D goalLine = new math.geom2d.line.Line2D(api.getFieldSide() * -1.37d, 0.7625d, api.getFieldSide() * -1.37d, -0.7625d);

            //calculating shoot vector
            d1LineEnd.sub(player);
            d1LineEnd.rotate(api.getSteerHeading());
            Vec2 shootEnd = new Vec2(player);
            shootEnd.add(d1LineEnd);

            math.geom2d.line.Line2D shootLine = new math.geom2d.line.Line2D(player.x, player.y, shootEnd.x, shootEnd.y);

            //intersect point
            Point2D goalShoot = goalLine.intersection(shootLine);

            Vec2 vShoot = new Vec2(goalShoot.x(), goalShoot.y());
            vShoot.sub(player);

            double d1 = vShoot.r;
            double d3 = Math.abs(goal.y - goalShoot.y());

            Vec2 opp = new Vec2();
            double d2 = closestOpponentToLine(api, player, new Vec2(goalShoot.x(), goalShoot.y()), opp);

            shootVector.add(new Vec2(vShoot));

        if (api.closestToBall()) {
            vuelta = Math.exp(-d1 / s_shooting1) - Math.exp((-d1 * s_shooting2 - d2 * s_shooting1) / (s_shooting1 * s_shooting2)) + penalizacion_shooting * (-1 - Math.exp(-d2 / s_shooting2) + Math.exp(-d3 / s_shooting3) - Math.exp((-d3 * s_shooting3 - d2 * s_shooting2) / (s_shooting3 * s_shooting2)));
        }
        return vuelta;
    }

    /*static double interceptFactor (RobotAPI me, Vec2 opponent, ArrayList<Vec2> shootVector) {
        double vuelta;

        Vec2 goal = me.toFieldCoordinates(me.getOurGoal());

        Vec2 d1LineEnd = new Vec2(opponent);
        d1LineEnd.setx(goal.x);
        math.geom2d.line.Line2D goalLine = new math.geom2d.line.Line2D(me.getFieldSide()*1.37d,0.7625d,me.getFieldSide()*1.37d,-0.7625d);

        //calculating shoot vector
        d1LineEnd.sub(opponent);
        d1LineEnd.rotate(api.getSteerHeading());
        Vec2 shootEnd = new Vec2(player);
        shootEnd.add(d1LineEnd);

        math.geom2d.line.Line2D shootLine = new math.geom2d.line.Line2D(player.x, player.y, shootEnd.x, shootEnd.y);

        //intersect point
        Point2D goalShoot = goalLine.intersection(shootLine);

        Vec2 vShoot = new Vec2(goalShoot.x(), goalShoot.y());
        vShoot.sub(player);

        double d1 = vShoot.r;
        double d3 = Math.abs(goal.y - goalShoot.y());

        double d2 = closestOpponentToLine(api, player, new Vec2(goalShoot.x(), goalShoot.y()));

        shootVector.add(new Vec2(vShoot));

        vuelta = Math.exp((-d1*s_intercep2*s_intercep3 -d2*s_intercep1*s_intercep3 -d3*s_intercep1*s_intercep2)/(s_intercep1*s_intercep2*s_intercep3));

        return vuelta;
    }*/

    static double keepGoalFactor (RobotAPI api) {
        Vec2[] mates = api.getTeammates();
        Vec2 goal = api.toFieldCoordinates(api.getOurGoal());

        double temp = 0.0;
        for (int i = 0; i < mates.length; i++){
            Vec2 v2Goal = new Vec2(goal);
            v2Goal.sub(mates[i]);
            temp +=  Math.exp(v2Goal.r/s_keepGoal);
        }
        double vuelta = (5.0d - temp) / 5.0d;
        return vuelta;
    }

    static Vec2 closesMateToPoint (RobotAPI api, Vec2 point) {
        Vec2[] mates = api.getTeammates();
        Vec2 closest = mates[0];

        for (int i = 1; i < mates.length; i++) {
            Vec2 nextMate = mates[i];
            if (closest.distance(point) < nextMate.distance(point)) {
                closest = nextMate;
            }
        }
        return closest;
    }

    static double closestOpponentToLine (RobotAPI api, Vec2 point1, Vec2 point2, Vec2 opp) {
        Vec2[] opps = api.getOpponents();

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

    static ArrayList<Vec2> ahead (RobotAPI api, Vec2[] opponents) {
        ArrayList<Vec2> vuelta = new ArrayList<>();
        Vec2 oppGoal = api.getOpponentsGoal();

        Vec2 globalOppGoal = api.toFieldCoordinates(oppGoal);
        for(Vec2 opp: opponents) {
            Vec2 dist = api.toFieldCoordinates(opp);
            dist.sub(globalOppGoal);
            if (dist.r < oppGoal.r) {
                vuelta.add(opp);
            }
        }

        Collections.sort(vuelta, new Vec2YComparator());
        return vuelta;
    }

    static int getPlayersActualTerritory(RobotAPI api) {
        Vec2 player = api.toFieldCoordinates(api.getPosition());
        int territory = 0;
        boolean finded = false;
        for (int i = 0; !finded && i < 6; i++) {
            Rectangle2D actualTerr = getTerritory(api, i);
            finded = actualTerr.contains(player.x, player.y);
            territory = i;
        }
        return territory;
    }

    static Rectangle2D getTerritory (RobotAPI api, int territory) {
        Rectangle2D finded = null;
        switch(territory) {
            case GOILIE_TERRITORY:
                finded = golieTerritory(api);
                break;
            case DEFENSE_TERRITORY:
                finded = defenseTerritory(api);
                break;
            case HALF_BACK_TERRITORY:
                finded = halfBackTerritory(api);
                break;
            case HALF_FORWARD_TERRITORY:
                finded = halfForwardTerritory(api);
                break;
            case FORWARD_TERRITORY:
                finded = forwardTerritory(api);
                break;
            case OPPONENTS_GOAL_TERRITORY:
                finded = opponentsGoalTerritory(api);
                break;
        }
        return finded;
    }

    static Rectangle2D golieTerritory (RobotAPI api) {
        int field = api.getFieldSide();
        return new Rectangle2D(field * 1.35, 0.7625, 0.205, 1.525);
    }

    static Rectangle2D defenseTerritory (RobotAPI api) {
        int field = api.getFieldSide();
        return new Rectangle2D(field * 1.145, 0.7625, 0.5725, 1.525);
    }

    static Rectangle2D halfBackTerritory (RobotAPI api) {
        int field = api.getFieldSide();
        return new Rectangle2D(field * 0.5725, 0.7625, 0.5725, 1.525);
    }

    static Rectangle2D opponentsGoalTerritory (RobotAPI api) {
        int field = api.getFieldSide();
        return new Rectangle2D(-field * 1.35, 0.7625, 0.205, 1.525);
    }

    static Rectangle2D forwardTerritory (RobotAPI api) {
        int field = api.getFieldSide();
        return new Rectangle2D(-field * 1.145, 0.7625, 0.5725, 1.525);
    }

    static Rectangle2D halfForwardTerritory (RobotAPI api) {
        int field = api.getFieldSide();
        return new Rectangle2D(-field * 0.5725, 0.7625, 0.5725, 1.525);
    }
}
