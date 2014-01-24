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

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.regex.Pattern;

import org.json.JSONException;
import org.json.JSONObject;

import android.util.Log;
import de.tavendo.autobahn.WebSocketConnection;
import de.tavendo.autobahn.WebSocketHandler;

public class TopicManager extends WebSocketHandler{

	private static final String tabs = "\t\t\t\t\t";

	private String topic;
	private String url;
	private WebSocketConnection mConnection;
	private String text;
	private String originalPayload;
	private Map<String, String> fieldMap;

	public TopicManager(String topic, String url, WebSocketConnection mConnection){
		this.topic = topic;
		this.url = url;
		this.mConnection = mConnection;
		this.text = "NO DATA?";
		fieldMap = new HashMap<String, String>();
	}

	@Override
	public void onOpen() {
		Log.d("Data Status", "Status: Connected to " + url);

		JSONObject jsonRequest = new JSONObject();

		try {
			jsonRequest.put("op", "subscribe");
			jsonRequest.put("topic", topic);
		} catch(Exception e) {


		}
		mConnection.sendTextMessage(jsonRequest.toString());
	}

	@Override
	public void onTextMessage(String payload) {
		originalPayload = payload;
	}

	@Override
	public void onClose(int code, String reason) {
		Log.d("Data Closed", "Connection lost." + reason);
	}

	/**
	 * 
	 * Get the desired field from the message that is currently being stored.
	 * @param field : String - Which field should be parsed.
	 * @return Text message contained in the field.
	 */
	public String getField(String field){
		String [] subfields = field.split(Pattern.quote("."));
		try{

			JSONObject obj = new JSONObject(getOriginalMessage());
			return getJSONMessageRec(obj.getJSONObject("msg"), subfields, 0);

		} catch (JSONException e) {
			e.printStackTrace();
			return "JSON ERROR";

		}	
	}

	/**
	 * Get the text value currently in the topic Manager.
	 * Updated on every message recieved.  
	 */
	public String getText(){
		text = parseMessage(originalPayload);
		return text;
	}

	/**
	 * Get the raw message that was returned from the ROS server.
	 * @return raw ros json string.
	 */
	public String getOriginalMessage(){
		return originalPayload;
	}

	public void start(){
		//Do nothing?
	}

	public void stop(){
		//Do nothing?
	}

	/**
	 * Parse a message and return the string represenation of it.  
	 * @param msg String : The message recieved over the topic.
	 * @return String rep of the message.
	 */
	private String parseMessage(String msg){

		JSONObject obj;

		if(msg == null){
			return "No Data";
		}

		try {
			obj = new JSONObject(msg);
			StringBuilder str = buildRecursiveList(obj.getJSONObject("msg"), 0);
			return str.toString();


		} catch (JSONException e) {
			e.printStackTrace();
			return "JSON ERROR";
		}	
	}

	
	/**
	 * This method builds the string represenation of the message based on the current level within the message.
	 * @param msg : JSONObject - The message information of the current level.
	 * @param tab_level : int - How deep into the message we are.
	 * @return StringBuilder : the string representation of the message formatted correctly.  
	 * @throws JSONException : Thrown in the message is malformed.  
	 */
	private StringBuilder buildRecursiveList(JSONObject msg, int tab_level) throws JSONException{
		Iterator<String> fields = msg.keys();
		StringBuilder build = new StringBuilder();

		while(fields.hasNext()){
			String val = fields.next();
			try{
				StringBuilder a = buildRecursiveList(msg.getJSONObject(val), tab_level + 1);
				for(int j = 0; j < tab_level; j++){
					build.append(tabs);
				}

				build.append(val);
				build.append(":");
				build.append("\n");

				build.append(a);

			}catch(Exception e){
				for(int j = 0;j < tab_level; j++){
					build.append(tabs);
				}

				build.append(val);
				build.append(":");
				build.append(msg.get(val));
				build.append("\n");
			}
		}
		return build;
	}

	private String getJSONMessageRec(JSONObject obj, String [] subfields, int index) throws JSONException{
		if(index + 1 >= subfields.length){
			String ret = obj.getString(subfields[index]);
			if(ret == null){
				return "No Data";
			}else{
				return ret;
			}
		}else{
			return getJSONMessageRec(obj.getJSONObject(subfields[index]), subfields, index+1);
		}
	}

	
	/**
	 * Format a string as a double and return that string. 
	 * @param string String to format or leeave alone
	 * @return String : The string formated as a double or a the string itself.  
	 */
	public static String performFormatting(String string){

		string = string.trim();

		Log.d("String into format", string);

		if(string.contains(":")){
			String [] pieces = string.split(":");

			if(pieces.length>1){
				String num = pieces[1];
				try{
					float f = Float.parseFloat(pieces[1]);
					num = String.format("%.4f", f);
				}catch(java.lang.NumberFormatException e){
					//do nothing
				}

				try{
					double d = Double.parseDouble(pieces[1]);
					num = String.format("%.4f", d);
				}catch(Exception e){
					//do nothing
				}

				return pieces[0]+": "+num;
			}else{
				return string;
			}

		}else{
			return string;
		}
	}
}
