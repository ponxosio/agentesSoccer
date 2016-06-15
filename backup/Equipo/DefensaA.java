package Equipo;

import java.util.Iterator;
import java.util.Queue;

import funciones.Funciones;

import mensajes.BlockMessage;


import EDU.gatech.cc.is.util.Vec2;

import teams.ucmTeam.Behaviour;
import teams.ucmTeam.Message;
import teams.ucmTeam.RobotAPI;

/*encargado de despejar la pelota del area de la porteria para que el portero no tenga qeu salir
 la despeja hacia el centro del campo donde deberia estar el central */

public class DefensaA extends Behaviour {
	private Vec2 esquinaArriba;
	private Vec2 esquinaAbajo;

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
		esquinaArriba= new Vec2(1.37*arg0.getFieldSide(),0.7625);
		esquinaAbajo= new Vec2(1.37*arg0.getFieldSide(),-0.7625);
		arg0.setDisplayString("Defensa");

	}

	@Override
	public void onRelease(RobotAPI arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public int takeStep() {
		// TODO Auto-generated method stub
		Queue<Message> mensajes = getPendingMessages();
		if(mensajes.isEmpty()){
			
		if(myRobotAPI.blocked() || myRobotAPI.teammateBlocking()){//si esta bloquedo evita colision
				myRobotAPI.setDisplayString("D-evitando colision");
				myRobotAPI.avoidCollisions();
				myRobotAPI.setSpeed(1.0);
				return RobotAPI.ROBOT_OK;
			}else{
				double posx = myRobotAPI.toFieldCoordinates(myRobotAPI.getPosition()).x*myRobotAPI.getFieldSide();
				double posy = myRobotAPI.toFieldCoordinates(myRobotAPI.getPosition()).y;
				if(myRobotAPI.closestToBall() || myRobotAPI.canKick()){
					
					if(posx > 1.145 && posy >= 0.5){//si esta cerca del area intenta ir a la esquina para que la pelota vuelva al centro
						myRobotAPI.setDisplayString("D-a esquina A");
						myRobotAPI.setBehindBall(esquinaArriba);
						myRobotAPI.kick();
					}else{ 
						if(posx > 1.145 && posy <= -0.5){//si esta cerca del area intenta ir a la esquina para que la pelota vuelva al centro
							myRobotAPI.setDisplayString("D-esquina B");
							myRobotAPI.setBehindBall(esquinaAbajo);
							myRobotAPI.kick();
						}else{//sino despeja hacia el centro del campo
							myRobotAPI.setDisplayString("D-despejando");
							myRobotAPI.setBehindBall(myRobotAPI.toEgocentricalCoordinates(new Vec2(0,0)));
							myRobotAPI.kick();
						}
					}
					return RobotAPI.ROBOT_OK;
				}else{//si no puede dar a la pelota la busca o vuelve a la porteria
					if((posx > 0.3)){//esta dentro de la zona de defensa
						myRobotAPI.setDisplayString("D-buscando la pelota");
						myRobotAPI.setSteerHeading(myRobotAPI.getBall().t);
						myRobotAPI.setSpeed(1.0);
						return RobotAPI.ROBOT_OK;
					}else{ //vuelve para defender
						myRobotAPI.setDisplayString("D-volviendo a defender");
						myRobotAPI.setSteerHeading(myRobotAPI.getOurGoal().t);
						myRobotAPI.setSpeed(1.0);
						return RobotAPI.ROBOT_OK;
					}
				}
			}
		}
		
		return RobotAPI.ROBOT_OK;	
		}

}
