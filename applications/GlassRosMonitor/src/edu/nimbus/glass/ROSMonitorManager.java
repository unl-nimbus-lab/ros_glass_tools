/*
copyright 2014 UNL Nimbus Lab 

  Licensed under the Apache License, Version 2.0 (the "License");
     you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
  
        http://www.apache.org/licenses/LICENSE-2.0
  
    Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
  limitations under the License.
*/
package edu.nimbus.glass;

import org.json.JSONException;
import org.json.JSONObject;

import android.util.Log;
import de.tavendo.autobahn.WebSocketConnection;
import de.tavendo.autobahn.WebSocketHandler;



/**
 * Class to handle data sent over the websocket connection.
 * 
 *
 */
public class ROSMonitorManager extends WebSocketHandler{

	//service to monitor ros in the background
	private ROSMonitorService _master;
	//The last warning received
	private String _lastWarning;
	//websocket
	WebSocketConnection mConnection;

	/** Topic on which warning messages will be published */
	public final static String WARN_TOPIC_NAME = "glass_warn";
	/** Message type for warning messages */
	public final static String WARN_TOPIC_TYPE = "std_msgs/String";

	/**
	 * Constructor 
	 * @param master : The background service that will control live card creation
	 * @param conn	 WebSocketConnection : connection to send and receive messages
	 */
	public ROSMonitorManager(ROSMonitorService master, WebSocketConnection conn){
		_lastWarning = "";
		_master = master;
		mConnection = conn;
	}

	@Override
	public void onOpen(){
		//Subscribe to the correct topic on open
		JSONObject req = new JSONObject();
		try {
			req.put("op", "subscribe");
			req.put("topic", WARN_TOPIC_NAME);
			req.put("type", WARN_TOPIC_TYPE);

		} catch (JSONException e) {
			Log.d("JSON ERROR", e.getMessage());
		}
		mConnection.sendTextMessage(req.toString());

	}	

	@Override
	public void onTextMessage(String payload){
		//try and parse the message and check to see if we have a new warning that warrants creating a  new live card.
		try {
			JSONObject res = new JSONObject(payload);
			String error_message = res.getJSONObject("msg").getString("data");
			if(!error_message.equals(_lastWarning)){
				_master.createWarning(error_message);
				_lastWarning = error_message;
			}

		} catch (JSONException e) {
			Log.d("JSON ERROR", e.getMessage());
		}

	}


	@Override
	public void onClose(int code, String reason) {
		Log.d("Data Closed", "Connection lost." + reason);

	}
	
	/**
	 * Clear the stored warning so the next one will trigger a new live card
	 */
	public void clearWarning(){
		_lastWarning = "";
	}

	/**
	 * Set the warning text
	 * @param text String : Text to set to current warning.
	 */
	public void setWarning(String text){
		_lastWarning = text;
	}

}
