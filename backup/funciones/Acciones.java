package funciones;

import java.util.Random;

import EDU.gatech.cc.is.util.Vec2;

import teams.ucmTeam.RobotAPI;
import teams.ucmTeam.RobotAPI.Pair;

public abstract class Acciones {

	//public static void cubrirJugador(RobotAPI r,)
	
	/**
	 * El jugador patrulla alrededor de un punto sin alejarse demasiado
	 * @param robotAPI RobotAPI del jugador
	 * @param punto Punto alrededor del cual se quiere patrullar
	 */
	public static void patrullaPunto(RobotAPI robotAPI, Vec2 punto){
		Vec2 pos = robotAPI.getPosition(); 
		if(punto.r<=0.2){
			double orientacion = pos.t+Math.random()*Vec2.PI;
        	robotAPI.setSteerHeading(orientacion);        	
        	robotAPI.setSpeed(0.5);
		}else{
			robotAPI.setSteerHeading(punto.t);
			robotAPI.setSpeed(1.0);
		}
	}
	
	public static void patrullaOctante(RobotAPI robotAPI, Vec2 punto){
		Vec2 pos = robotAPI.getPosition();
		if(robotAPI.toFieldCoordinates(pos).octant() == robotAPI.toFieldCoordinates(punto).octant()){
			double orientacion = pos.t+Math.random()*Vec2.PI;
        	robotAPI.setSteerHeading(orientacion);
        	robotAPI.setSpeed(0.5);
		}else{
			robotAPI.setSteerHeading(punto.t);
			robotAPI.setSpeed(1.0);
		}
	}
	
	/**
	 * @param robotAPI RobotAPI del jugador
	 * @param punto1 esquina superior izquierda de la zona
	 * @param punto2 esquina inferior derecha de la zona
	 */
	public static void patrullaZona(RobotAPI robotAPI, Vec2 punto1, Vec2 punto2){
		Vec2 pos = robotAPI.getPosition();
		Vec2 posField = robotAPI.toFieldCoordinates(pos);
		Vec2 p1 = robotAPI.toFieldCoordinates(punto1);
		Vec2 p2 = robotAPI.toFieldCoordinates(punto2);
		if(p1.x<=posField.x && p1.y>=posField.y && p2.x>=posField.x && p2.y<=posField.y){
			double orientacion = pos.t+Math.random()*Vec2.PI;
        	robotAPI.setSteerHeading(orientacion);
        	robotAPI.setSpeed(0.5);
		}else{
			Vec2 centro = new Vec2((p1.x+p2.x)/2, (p1.y + p2.y)/2);
			robotAPI.setSteerHeading(robotAPI.toEgocentricalCoordinates(centro).t);
			robotAPI.setSpeed(1.0);
		}
	}
	
	public static void irAPunto(RobotAPI robotAPI, Vec2 punto){
		robotAPI.setSteerHeading(punto.t);
		robotAPI.setSpeed(1.0);
	}
	
	public static void bloquearPelota(RobotAPI robotAPI){
		robotAPI.setBehindBall(robotAPI.toEgocentricalCoordinates(new Vec2(1.37*robotAPI.getFieldSide(),0.7625)));
		robotAPI.setSpeed(1.0);
	}
	
	public static void paraleloAlBalonEjeY(RobotAPI robotAPI){
		//.sety(robotAPI.getBall().y);
		double posy = robotAPI.toFieldCoordinates(robotAPI.getBall()).y;
		if(posy>-0.5 && posy<0.5){
			robotAPI.setSteerHeading(new Vec2(robotAPI.toEgocentricalCoordinates(new Vec2(1.145*robotAPI.getFieldSide(),0)).x,robotAPI.getBall().y).t);
			robotAPI.setSpeed(1.0);
		}else{
			if(robotAPI.toFieldCoordinates(robotAPI.getPosition()).x*robotAPI.getFieldSide()>=1.5){
				robotAPI.setSteerHeading(robotAPI.getBall().t);
				robotAPI.setSpeed(0.00001);
			}else{
				robotAPI.setSteerHeading(robotAPI.getOurGoal().t);
				robotAPI.setSpeed(1.0);
			}				
		}
	}
	
	public static void esquivaOPasa(RobotAPI robotAPI){
		Pair passh = robotAPI.passHeuristic(robotAPI);
		if(((Double)passh.value)>0.5){
			robotAPI.passBall((Vec2)passh.position);
		}else{			
			robotAPI.avoidCollisions();
			robotAPI.setSpeed(1.0);
		}
	}
}
