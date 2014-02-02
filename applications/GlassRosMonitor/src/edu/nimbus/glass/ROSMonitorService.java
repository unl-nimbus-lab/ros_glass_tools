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

import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.widget.RemoteViews;
import android.widget.TextView;

import com.google.android.glass.timeline.LiveCard;
import com.google.android.glass.timeline.TimelineManager;

import de.tavendo.autobahn.WebSocketConnection;
import de.tavendo.autobahn.WebSocketException;


/**
 * Service that will control the monitoring of a specific topic on ROS to monitor for warnings sent to the glass
 * when a warning is received this service will insert a live card into the timeline
 *
 */
public class ROSMonitorService extends Service {


	/** Needed to create/remove live cards */
	private static final String LIVE_CARD_ID = "ROS_WARNING";
	/** EDIT THIS LINE TO MAKE THE CONNECTION TO ROS WORK */
	public final static String HOST_ADDRESS = "ws://10.214.32.10:9090";
	

	/** Needed glass items */
	private TimelineManager mTimelineManager;
	private LiveCard mLiveCard;
	private final ROSMonitorBinder mBinder = new ROSMonitorBinder();

	
	/** Needed  web socket stuff */
	private ROSMonitorManager mROSMonitor;
	private WebSocketConnection mConnection;


	/**
	 * A binder that gives other components access to this service.  
	 * Allows the menu to remove the live card when selected
	 */
	public class ROSMonitorBinder extends Binder {

		public void remove_card() {
				ROSMonitorService.this.removeLiveCard();
			}


	}

	@Override
	public void onCreate(){
		if(mConnection==null){
			mConnection = new WebSocketConnection();
		}

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
		mTimelineManager = TimelineManager.from(this);



		try {
			mROSMonitor = new ROSMonitorManager(this, mConnection);
			mConnection.connect(HOST_ADDRESS,mROSMonitor );
		} catch (WebSocketException e) {
			e.printStackTrace();
		}


		return START_STICKY;
	}


	@Override
	public void onDestroy() {

		//Get rid of the livecard and clean everything up.
		if (mLiveCard != null && mLiveCard.isPublished()) {
			mLiveCard.unpublish();
			mLiveCard = null;
		}

		mConnection.disconnect();
		mConnection = null;
		super.onDestroy();
	}

	@Override
	public IBinder onBind(Intent intent) {
		return mBinder;
	}

	/**
	 * This method takes a string error message and creates a live card displaying that warning on the Glass.
	 * This allows the service to alert the user when a defined behavor has occured.
	 * @param error_message String : error message to dispaly
	 */
	public void createWarning(String error_message) {
		//if we have to live card create one
		if (mLiveCard == null){
			mLiveCard = mTimelineManager.createLiveCard(LIVE_CARD_ID);
		}
		//get rid of the old one
		if(mLiveCard.isPublished()){
			mLiveCard.unpublish();
		}
		//Create the view for the live card including the error message.
		TextView text = new TextView(this);
		text.setText(error_message);
		RemoteViews views = new RemoteViews(getBaseContext().getPackageName(), R.layout.warn_card);
		views.setTextViewText(R.id.warn_text, error_message	);
		mLiveCard.setViews(views);
		
		//publish the live card and ensure that it takes up display
	     Intent intent = new Intent(this, MenuActivity.class);
	     mLiveCard.setAction(PendingIntent.getActivity(this, 0, intent, 0));
	     mLiveCard.publish(LiveCard.PublishMode.REVEAL);
	}
	
	
	/** 
	 * Get rid of the live card and reset the monitors last message
	 */
	public void removeLiveCard(){
		if(mLiveCard != null){
			mLiveCard.unpublish();
			mROSMonitor.setWarning("");
		}
		
	}
}

