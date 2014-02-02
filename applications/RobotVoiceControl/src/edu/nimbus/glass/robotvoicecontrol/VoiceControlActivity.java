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
package edu.nimbus.glass.robotvoicecontrol;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.google.android.glass.app.Card;



import de.tavendo.autobahn.WebSocketConnection;
import de.tavendo.autobahn.WebSocketHandler;
 

import android.os.Bundle;
import android.app.Activity;
import android.speech.RecognizerIntent;
import android.util.Log;
import android.view.View;

/**
 * Activity to transmit voice commands to the ROS Server from google glass.
 * @author ataylor
 *
 */
public class VoiceControlActivity extends Activity {
	
	/** This line must be updated to ensure that the glass connects to the correct webserver that is running ROS */
	public final static String HOST_ADDRESS = "ws://10.214.32.10:9090";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }

    @Override
    protected void onResume(){
    	super.onResume();
    	
    	//Get the voice results and send them to the server.
    	String command = "";
    	ArrayList<String> voiceResults = getIntent().getExtras()
    	        .getStringArrayList(RecognizerIntent.EXTRA_RESULTS);
    	
    	for(int i=0;i< voiceResults.size(); i++){
    		Log.d("Voice Results", voiceResults.get(i));
    		command = voiceResults.get(i);
    	}
    	Card card1 = new Card(getBaseContext());
    	card1.setText("Sending Command:\n" + command);
    	// Don't call this if you're using TimelineManager
    	View card1View = card1.toView();
    	sendMessage(command);
    	setContentView(card1View);
    }
   
    /**
     * Send the command to the ROS Server on the specialized topic and wait for the reply back.
     * @param command
     */
    private void sendMessage(String command) {

		final String url = HOST_ADDRESS;
		Log.d("SENDING REQUEST", command);
		final WebSocketConnection mConnection = new WebSocketConnection();
		final String to_send = command;

		try{
			Log.d("Socket", "Atempting connection");
			mConnection.connect(url, new WebSocketHandler(){

				boolean message_received = false;
				@Override
				public void onOpen() {
					//send the request
					JSONObject jsonRequest = new JSONObject();


					try {
						
						//Create the object and call it correctly using the API for the rosbridge server.
						jsonRequest.put("op", "call_service");
						jsonRequest.put("service", "/glass_voice_command");
						
						JSONObject args = new JSONObject();
						args.put("command", to_send);
						
						jsonRequest.put("args", args);
						Log.d("SENDING COMMAND", jsonRequest.toString());

						mConnection.sendTextMessage(jsonRequest.toString());


					} catch(Exception e) {

					}
					Log.d("Main Connection", "Status: Connected to " + url);

				}

				/** 
				 * Display the message we recieved back and close the application.
				 */
				@Override
				public void onTextMessage(String payload) {
					Log.d("Main Payload", payload);
					String result_string = "Falure";
					try {
						JSONObject res = new JSONObject(payload);
						if(res.getBoolean("result") == false){
							result_string = "Service Falure";
						}else{
							result_string = res.getJSONObject("values").getString("result");
						}
					} catch (JSONException e) {
					}
					message_received = true;
					//We got a message back from the server so lets create the cards for selection.
					Card card1 = new Card(getBaseContext());
			    	card1.setText("Done :\n" + result_string);
			    	// Don't call this if you're using TimelineManager
			    	View card1View = card1.toView();
			    	setContentView(card1View);
					mConnection.disconnect();					
					finish();
					
				}

				/**
				 * If it is closed than we have a poor web connection.  Let the user know.
				 */
				@Override
				public void onClose(int code, String reason) {
					Log.d("WEBSOCKET CLOSE", code + "");
					//If connection is lost change the view.
					if (code == 3 && !message_received){
						Card card1 = new Card(getBaseContext());
				    	card1.setText("No Web Connection");
						View card1View = card1.toView();
				    	setContentView(card1View);
						
					}
				}
				
			});
			
		}catch (Exception e){
			e.printStackTrace();
		}
	}
    
}
