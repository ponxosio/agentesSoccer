package mensajes;

import EDU.gatech.cc.is.util.Vec2;
import teams.ucmTeam.Message;

public class BlockMessage extends Message {
	
	Vec2 punto;

	public BlockMessage(Vec2 punto) {
		super();
		this.punto = punto;
		// TODO Auto-generated constructor stub
	}

	public Vec2 getPunto() {
		return punto;
	}

	public void setPunto(Vec2 punto) {
		this.punto = punto;
	}
	
	
}
