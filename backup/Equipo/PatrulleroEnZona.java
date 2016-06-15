package Equipo;

import funciones.Acciones;
import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.RobotAPI;

public class PatrulleroEnZona extends Behaviour {
	
	private Vec2 inicio;
	private Vec2 fin;
	
	public PatrulleroEnZona(Vec2 inicio, Vec2 fin) {
		super();
		this.inicio = inicio;
		this.fin = fin;
	}

	@Override
	public void configure() {
		// TODO Auto-generated method stub

	}

	@Override
	public void end() {
		// TODO Auto-generated method stub

	}

	@Override
	public void onInit(RobotAPI arg0) {
		// TODO Auto-generated method stub
		//cambiamos las coordenadas segun el lado del campo
		if(inicio.x == 1 && inicio.y == 0.7625 && fin.x == -1 && fin.y == -0.7625){
			inicio.setx(inicio.x*arg0.getFieldSide());
			fin.setx(fin.x*arg0.getFieldSide());
		}
		inicio = arg0.toEgocentricalCoordinates(inicio);
		fin = arg0.toEgocentricalCoordinates(fin);
		arg0.setDisplayString("Patrullando zona");
	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public int takeStep() {
		// TODO Auto-generated method stub
		if(myRobotAPI.blocked() ){
			myRobotAPI.avoidCollisions();
			myRobotAPI.setSpeed(1);
			myRobotAPI.setDisplayString("desbloqueandome");
			return RobotAPI.ROBOT_OK;
		}
		if(myRobotAPI.closestToBall()){
			if((Double)myRobotAPI.passHeuristic(myRobotAPI).value > 0.5)
				myRobotAPI.passBall((Vec2)myRobotAPI.passHeuristic(myRobotAPI).position);
			else{
				myRobotAPI.setSteerHeading(myRobotAPI.getOpponentsGoal().t);
				myRobotAPI.kick();
				myRobotAPI.setSpeed(1);
			}
			myRobotAPI.setDisplayString("patrullando");
			return RobotAPI.ROBOT_OK;
		}
		Acciones.patrullaZona(myRobotAPI, inicio, fin);
		return RobotAPI.ROBOT_OK;
	}

}
