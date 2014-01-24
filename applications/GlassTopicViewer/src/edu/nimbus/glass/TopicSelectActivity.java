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

import java.util.ArrayList;
import java.util.List;

import org.json.JSONException;
import org.json.JSONObject;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;

import com.google.android.glass.app.Card;
import com.google.android.glass.widget.CardScrollAdapter;
import com.google.android.glass.widget.CardScrollView;

import de.tavendo.autobahn.WebSocketConnection;
import de.tavendo.autobahn.WebSocketHandler;


/**
 * This class handles the original execution of the application.
 * On Startup the applications polls the ROSBridge server to get a list of currently publishing topics
 * After it does this it presents the user with a list of those topics.  Once one is selected the application
 * than starts of the TopicService to handle the creation of the live card and the updating and changing of the information
 * being displayed.
 *
 */
public class TopicSelectActivity extends Activity {
	private List<Card> mCards;
	private CardScrollView mCardScrollView;
	private String[] stringArr = null;
    	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		Intent intt = new Intent(TopicSelectActivity.this, TopicService.class);		
		//kill the service if it is already running because we want to subscrive to a new topic.  
		stopService(intt);
		createTopics();

	}
	
	/**
	 * Get the list of topics.
	 * @return
	 */
	private ArrayList<String> getTopics() {
		ArrayList<String> topic_names = new ArrayList<String>();
		if(stringArr != null){
			for(int i=0;i < stringArr.length;i++){
				topic_names.add(stringArr[i]);
			}
		}else{
			topic_names.add("ERROR NO Topics");
		}
		return topic_names;
	}
	
	private void createCards(ArrayList<String> topicNames) {
		mCards = new ArrayList<Card>();
		//TODO: Handle no topics gracefully.
		for(String topic_name : topicNames) {
			Card card = new Card(this);
			card.setText(topic_name);
			card.setFootnote("Topic Card");
			//card.setInfo("Topic card");
			mCards.add(card);
		}
	}
	
	
	/**
	 * Adapter for view of created cards.
	 *
	 */
	private class TopicCardScrollAdapter extends CardScrollAdapter {
		@Override
		public int findIdPosition(Object id) {
			return -1;
		}
		
		@Override
		public int findItemPosition(Object item) {
			return mCards.indexOf(item);
		}
		
		@Override
		public int getCount() {
			return mCards.size();
		}
		
		@Override
		public Object getItem(int position) {
			return mCards.get(position);
		}
		
		@Override
		public View getView(int position, View convertView, ViewGroup parent) {
			return mCards.get(position).toView();
		}
	}
	
	
	/**
	 * Method to poll the ROS Server to get the list of topics that are currenly being published. 
	 */
	private void createTopics() {

		final String url = TopicService.HOST_ADDRESS;
		
		final WebSocketConnection mConnection = new WebSocketConnection();

		try{
			Log.d("Socket", "Atempting connection");
			mConnection.connect(url, new WebSocketHandler(){
				boolean message_received = false;
				@Override
				public void onOpen() {
					
					//send the request
					JSONObject jsonRequest = new JSONObject();

					try {
						jsonRequest.put("op", "call_service");
						jsonRequest.put("service", "/list_topics");
					} catch(Exception e) {

					}
					Log.d("Main Connection", "Status: Connected to " + url);
					mConnection.sendTextMessage(jsonRequest.toString());

				}

				@Override
				public void onTextMessage(String payload) {
					message_received = true;
					Log.d("Main Payload", payload);
					//We got a message back from the server so lets create the cards for selection.
					JSONObject obj;
					String topics = "n/a";
					try {
						obj = new JSONObject(payload);
						topics = obj.getJSONObject("values").getString("topics");

					} catch (JSONException e) {
						topics = "n/a";
				}
					if(topics.equals("n/a")){
						Card card1 = new Card(getBaseContext());
				    	card1.setText("No Topics Returned Check Setup.");
						View card1View = card1.toView();
				    	setContentView(card1View);
					}else{
					//Split topics and create cards.
					stringArr = topics.split(",");
					ArrayList<String> topicNames = getTopics();

					createCards(topicNames);

					mCardScrollView = new CardScrollView(TopicSelectActivity.this);
					TopicCardScrollAdapter adapter = new TopicCardScrollAdapter();
					mCardScrollView.setAdapter(adapter);
					mCardScrollView.setOnItemClickListener(new OnItemClickListener(){

						/** 
						 * On click we are going to get the name of the topic and create the service to display messages
						 * from that topic.
						 */
						@Override
						public void onItemClick(AdapterView<?> parent, View view,
								int position, long id) {
								
								String topic = ((Card) parent.getItemAtPosition(position)).getText();
								
								//bindService(new Intent(TopicSelectActivity.this, TopicService.class), mServiceConnection, 0);
								Intent intt = new Intent(TopicSelectActivity.this, TopicService.class);	
								intt.putExtra("view_state", TopicService.ALL_FIELDS);
								intt.putExtra("topic",topic);
								startService(intt);
						}
						
					});
					
					//Show the scroll cards and close the connection.
					mCardScrollView.activate();
					setContentView(mCardScrollView);
					mConnection.disconnect();

					}
				}

				@Override
				public void onClose(int code, String reason) {
					if(!message_received){
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
