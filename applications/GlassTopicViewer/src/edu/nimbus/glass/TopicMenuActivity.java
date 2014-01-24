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


import java.util.ArrayList;
import java.util.Iterator;

import org.json.JSONException;
import org.json.JSONObject;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;

/**
 * This activity manages the options menu that appears when the user taps on the live card display.
 * The menu allows them to quit, or select a different view.
 */
public class TopicMenuActivity extends Activity {

    private TopicService.TopicBinder mCompassService;
    private boolean mResumed;

    private ServiceConnection mConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            if (service instanceof TopicService.TopicBinder) {
                mCompassService = (TopicService.TopicBinder) service;
                openOptionsMenu();
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            // Do nothing.
        }
        
        public void remove_card(){
        	this.remove_card();
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        bindService(new Intent(this, TopicService.class), mConnection, 0);
    }

    @Override
    protected void onResume() {
        super.onResume();
        mResumed = true;
        openOptionsMenu();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mResumed = false;
    }

    @Override
    public void openOptionsMenu() {
        if (mResumed && mCompassService != null) {
            super.openOptionsMenu();
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.topic, menu);
        return true;
    }

    /**
     * This method here will either kill the live card, show or switch the view by starting the 
     * feild select service.
     */
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.stop:
                stopService(new Intent(this, TopicService.class));
                return true;
            case R.id.graph_view:
                //create a list of fields from the current topic and have the user select which field to view.
            	Log.d("Menu:", "Switching to graph view");
            	ArrayList<String> messageFields = createFieldList(mCompassService.getOriginalMessage());
            	Intent startIntent = new Intent(this, TopicFieldSelectActivity.class);
            	startIntent.putStringArrayListExtra("field_names", messageFields);
            	startIntent.putExtra("topic", mCompassService.getCurrentTopic());
            	startActivity(startIntent);
            	return true;
            case R.id.text_view:
                //create a list of fields from the current topic and have the user select which field to view.
            	Log.d("Menu:", "Switching to text view");
            	ArrayList<String> mFields = createFieldList(mCompassService.getOriginalMessage());
            	Intent sint = new Intent(this, TopicFieldSelectActivity.class);
            	sint.putStringArrayListExtra("field_names", mFields);
            	sint.putExtra("textual", true);
            	sint.putExtra("topic", mCompassService.getCurrentTopic());
            	startActivity(sint);
            	return true;
            default:
                return super.onOptionsItemSelected(item);
                
            /*
             * In the event of the user selecting "graph" or "text" and then
             * a specific field for an in depth view of that field, an intent will
             * be sent to the service, which in turn will forward the information
             * to the renderer.
             * 
             * Messages will likely be of the form (0, field) for graphing and (1, field) for
             * in depth text and (2, null) for return to all topics view
             */
        }
    }
    


    /**
     * This method creates an ArrayList of fields that are defined in the
     * message on the topic currently being displayed
     */
    private ArrayList<String> createFieldList(String topicText) {
    	ArrayList<String>  arrList = new ArrayList<String>();
    	JSONObject msg;
		try {
			
			msg = new JSONObject(topicText);
			buildRecursiveList(msg.getJSONObject("msg"), arrList, "");
			return arrList;
		} catch (JSONException e) {
			return null;
		}
	}


    /**
     * This method builds a recursive list of fields within the message that is being currently displayed
     */
    public void buildRecursiveList(JSONObject msg, ArrayList<String> arrList, String currentPrefix) throws JSONException{
		Iterator<String> fields = msg.keys();

		while(fields.hasNext()){
			String val = fields.next();
			try{
				if(currentPrefix.equals("")){
					buildRecursiveList(msg.getJSONObject(val), arrList, val);
				}else{
					buildRecursiveList(msg.getJSONObject(val), arrList, currentPrefix + "."  + val);

				}
			}catch(Exception e){
				arrList.add(currentPrefix +"." + val);
			}
		}
	}
    
    
	@Override
    public void onOptionsMenuClosed(Menu menu) {
        super.onOptionsMenuClosed(menu);

        unbindService(mConnection);

        // We must call finish() from this method to ensure that the activity ends either when an
        // item is selected from the menu or when the menu is dismissed by swiping down.
        finish();
    }
}
