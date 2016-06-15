package Equipo;

import java.awt.TrayIcon.MessageType;

import mensajes.BlockMessage;
import EDU.gatech.cc.is.util.Vec2;
import t101116.role.MyRole;
import teams.ucmTeam.Behaviour;
import teams.ucmTeam.RobotAPI;
import teams.ucmTeam.TeamManager;



public class Manager extends TeamManager {
	
	
	int portero;
	int contportero;
	int patrullero;
	int delantero;
	
	@Override
	public Behaviour[] createBehaviours() {
		// TODO Auto-generated method stub
		return new Behaviour[] {
				new GoalKeeper(),new DefensaA(),new PatrulleroEnZona(new Vec2(1, 0.5), new Vec2(-1, -0.5)),new Central(),new Delantero()};
	}

	@Override
	public Behaviour getDefaultBehaviour(int arg0) {
		// TODO Auto-generated method stub
		return _behaviours[arg0];
	}

	@Override
	public int onConfigure() {
		// TODO Auto-generated method stub
		portero=0;
		contportero=0;
		patrullero = 2;
		delantero = 4;
		return RobotAPI.ROBOT_OK;
	}

	@Override
	protected void onTakeStep() {
		// TODO Auto-generated method stub
		RobotAPI rb = _players[0].getRobotAPI();
		if(rb.getJustScored()!=0){
			portero=0;
			for(int i=0;i<_players.length; i++){
				_players[i].setBehaviour(getDefaultBehaviour(i));
			}
		}
		else{
			RobotAPI api =_players[portero].getRobotAPI();
			Vec2 ball = api.toFieldCoordinates(api.getBall());
			/*el portero no mira que este bloqueado si el manager ve que esta bloqueado 
			 * mucho tiempo cambia el defensa por portero*/
			if(api.goalkeeperBlocked()){
				contportero++;
				if(contportero > 500){
					if(portero==0){
						portero=1;
						_players[portero].setBehaviour(new DefensaA());
						_players[0].setBehaviour(new GoalKeeper());
					}else{
						portero=0;
						_players[portero].setBehaviour(new DefensaA());
						_players[1].setBehaviour(new GoalKeeper());
					}
				}
			}else{
				contportero=0;
			}
			//MANAGER PARA ATAQUE Y DEFENSA
			if(ball.x*api.getFieldSide()>0.75){//la pelota esta en nuestro campo DEFENSA	
					
					
			}else{//la pelota esta en el suyo ATAQUE
				if(_players[delantero].getRobotAPI().closestToBall()){
					_players[patrullero].setBehaviour(new BlockPortero());
				}
			}
			
		}
		
	}
	
}
