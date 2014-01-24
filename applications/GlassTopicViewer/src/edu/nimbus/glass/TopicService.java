/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

import com.google.android.glass.timeline.LiveCard;
import com.google.android.glass.timeline.TimelineManager;

import de.tavendo.autobahn.WebSocketConnection;
import de.tavendo.autobahn.WebSocketException;

import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;

import android.os.Binder;
import android.os.IBinder;
import android.speech.tts.TextToSpeech;
import android.util.Log;



/**
 * The main application service that manages topic information throughout the apps lifetime.
 */
public class TopicService extends Service {
	WebSocketConnection mConnection;
	
	public static final String ALL_FIELDS = "ALL_FIELDS";
	public static final String GRAPH = "GRAPH";
	public static final String FIELD_FOCUS = "FIELD_FOCUS";
	public static final int NUM_DECIMALS = 2;
	private final TopicBinder mBinder = new TopicBinder();

	private TopicManager mTopicManager;
	private TextToSpeech mSpeech;

	private TimelineManager mTimelineManager;
	private LiveCard mLiveCard;
	private TopicRenderer mRenderer;

	private String topic;
	

	private static final String LIVE_CARD_ID = "topic";
	/** This line must be updated to ensure that the glass connects to the correct webserver that is running ROS */
	public final static String HOST_ADDRESS = "ws://10.214.32.106:9090";


	/**
	 * A binder that gives other components access to this service.  This allows other components to get the original message, topic, and text 
	 * provided by the Topic manager.
	 */
	public class TopicBinder extends Binder {
		public String getTopicText(){
			if(mTopicManager != null){
				return mTopicManager.getText();
			}else{
				return null;
			}
		}
		
		public String getOriginalMessage(){
			if(mTopicManager != null){
				return mTopicManager.getOriginalMessage();
			}else{
				return null;
			}
		}
		
		public String getCurrentTopic(){
			return topic;
			
		}
	}



	@Override
	public void onCreate() {
		super.onCreate();

		//Keep text to speech here but it is not used in our application
		mSpeech = new TextToSpeech(this, new TextToSpeech.OnInitListener() {
			@Override
			public void onInit(int status) {
				// Do nothing.
			}

		});
		if(mConnection==null){
			mConnection = new WebSocketConnection();
		}



	}

	@Override
	public IBinder onBind(Intent intent) {
		return mBinder;
	}

	/**
	 * Note:
	 * Services that have already started will receive information through this method.
	 * 
	 *  "startService() delivers a message to an existing running copy of the service, 
	 *  via the onStartCommand() method, if the service was already created. If it isn't yet
	 *  created, it  will create such a copy (thereby triggering a call to onCreate() first, 
	 *  followed by the onStartCommand) if there is no running copy of the service."
	 */
	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {		
		/*
		 * Will need to add logic to check if topicRenderer was already created.
		 * If it is, then switch the state that it is rendering.
		 */
		mTimelineManager = TimelineManager.from(this);
		String view_state = intent.getStringExtra("view_state");	//view_state -> {graph, field_focus, all_fields}
		Log.d("State to enter", view_state);
		/*
		 * If select all_fields and it isn't the same topic as before, have to reconnect.
		 * Otherwise just switch state back to all_fields
		 */
		if(view_state.equals(TopicService.ALL_FIELDS)){
			String new_topic = intent.getStringExtra("topic");
			if(topic == null || !new_topic.equals(topic)){

				/*
				 * I'm not sure if I should destroy the old one (if it exists) or reuse what I can.
				 */
				Log.d("TOPIC", new_topic);
				topic = new_topic;
				rosTopic(new_topic);
			}else{
				Log.d("Viewing previous topic", topic);
				mRenderer.setRendererAllFieldsState();
			}

			/*
			 * Other options to interact with data: graph and focus on field
			 */
		}else{
			assert topic != null;
			if(view_state.equals(TopicService.GRAPH)){
				Log.d("Graphing Topic: ", topic);
				mRenderer.setRendererGraphState(intent.getStringExtra("field"));
			}else if(view_state.equals(TopicService.FIELD_FOCUS)){
				Log.d("Text focus on topic: ", topic);
				mRenderer.setRendererFieldFocusState(intent.getStringExtra("field"));
			}else{
				Log.d("Incorrect menu option", view_state);
			}
		}

		return START_STICKY;
	}

	/**
	 * This method connects to the server which has an instance of the ROSBridge server running.
	 * On connection it requests a subscription to the passed topic and creates the topic manager to
	 * listen to updates from that topic. 
	 * @param new_topic String: Topic selected for debugging/viewing.
	 */
	private void rosTopic(String new_topic){
		mTopicManager = new TopicManager(new_topic.trim() , HOST_ADDRESS, mConnection);
		try {
			mConnection.connect(HOST_ADDRESS, mTopicManager);
		} catch (WebSocketException e) {
			e.printStackTrace();
		}
		if (mLiveCard == null) {
			mLiveCard = mTimelineManager.createLiveCard(LIVE_CARD_ID);
			mRenderer = new TopicRenderer(this, mTopicManager);

			mLiveCard.setDirectRenderingEnabled(true).getSurfaceHolder().addCallback(mRenderer);
			//display the options menu when the live card is tapped.
			Intent menuIntent = new Intent(this, TopicMenuActivity.class);
			menuIntent.putExtra("Destroy", true);
			menuIntent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_CLEAR_TASK);
			mLiveCard.setAction(PendingIntent.getActivity(this, 0, menuIntent, 0));
			mLiveCard.publish(LiveCard.PublishMode.REVEAL);
		}
	}

	@Override
	public void onDestroy() {
		
		//Get rid of the livecard and clean everything up.
		if (mLiveCard != null && mLiveCard.isPublished()) {
			mLiveCard.unpublish();
			mLiveCard.getSurfaceHolder().removeCallback(mRenderer);
			mLiveCard = null;
		}

		mSpeech.shutdown();
		mConnection.disconnect();
		mConnection = null;
		mSpeech = null;
		mTopicManager = null;

		super.onDestroy();
	}

	
	/**
	 * Get the name of the topic currently being examined.
	 * @return String : Name of the topic that is being examined
	 */
	public String getTopic(){
		return topic;
	}
}
