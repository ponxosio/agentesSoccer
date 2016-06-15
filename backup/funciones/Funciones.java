package funciones;

import java.awt.Robot;

import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.RobotAPI;

public abstract class Funciones {
	
	/**
	 * 
	 * @param myRobotAPI RobotApi del jugador
	 * @return si hay un oponente cerca y en frente del jugador devuleve true
	 * si no false
	 */
	public static boolean oponenteEnFrente(RobotAPI myRobotAPI){
		return myRobotAPI.blocked() ||(myRobotAPI.getClosestOpponent().r<=myRobotAPI.getPlayerRadius()*2 && 
				myRobotAPI.getClosestOpponent().t<=myRobotAPI.getSteerHeading()+Vec2.PI/4 &&
				myRobotAPI.getClosestOpponent().t>=myRobotAPI.getSteerHeading()-Vec2.PI/4);
	}
	
	/**
	 * 
	 * @param myRobotApi  RobotApi del jugador
	 * @return	si la posesion la tiene el equipo del jugador devuelve 1
	 * 			si la posesion la tiene el equipo contrario devuelve -1
	 * 			si nadie tiene la posesion devuelve 0
	 */
	public static int posesion(RobotAPI myRobotApi){
		Vec2 posPelota = myRobotApi.getBall();
		Vec2 oponentCercano = myRobotApi.closestTo(myRobotApi.getOpponents(), posPelota);
		Vec2 teamMateCercano = myRobotApi.closestTo(myRobotApi.getTeammates(), posPelota);
		Vec2 []jugs = {oponentCercano,teamMateCercano};
		Vec2 masCercano = myRobotApi.closestTo(jugs,posPelota);
		if(masCercano.equals(oponentCercano) && oponentCercano.distance(posPelota)<=myRobotApi.getPlayerRadius()+0.05)
			return -1;
		if(masCercano.equals(teamMateCercano) && teamMateCercano.distance(posPelota)<=myRobotApi.getPlayerRadius()+0.05)
			return 1;		
		return 0;
	}
	
	/**
	 * @param p1 punto anterior de la trayectoria
	 * @param p2 punto actual   de la trayectoria
	 * @param robotAPI RobotApi del jugador
	 * @return si los dos puntos se han tomado en nuestro campo de juego, 
	 * el balón se encuentra a 1/4 de campo de la porteria y su trayectoria está dirigida a puerta	 
	 */
	
	public static boolean trayectoriaDeGol(Vec2 punto1, Vec2 punto2, RobotAPI robotAPI){
		Vec2 p1,p2;
		p1 = robotAPI.toFieldCoordinates(punto1);
		p2 = robotAPI.toFieldCoordinates(punto2);
		int campoPropio = robotAPI.getFieldSide();
		/*System.out.println(p1.angle(p2));
		System.out.println(p1.angle(new Vec2(campoPropio*1.37, 0.5)));
		System.out.println(p1.angle(new Vec2(campoPropio*1.37, -0.5)));*/
		return p1.x*campoPropio>=0 && p2.x*campoPropio>0 && p2.x*campoPropio>=1.145/2 && (p1.angle(p2) <= p1.angle(new Vec2(campoPropio*1.37, 0.5)) && p1.angle(p2) >= p1.angle(new Vec2(campoPropio*1.37, -0.5)));
		
	}
	
	public static boolean estaEnZona(Vec2 objeto, Vec2 pZona1, Vec2 pZona2, RobotAPI r){
		Vec2 obj = r.toFieldCoordinates(objeto);
		Vec2 z1 = r.toFieldCoordinates(pZona1);
		Vec2 z2 = r.toFieldCoordinates(pZona2);
		return z1.x<=obj.x && z2.x>=obj.x && z1.y>=obj.y && z2.y<=obj.y; 
	}
	
}
