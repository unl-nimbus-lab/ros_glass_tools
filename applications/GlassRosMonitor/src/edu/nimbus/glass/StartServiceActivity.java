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

import com.google.android.glass.app.Card;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;


/**
 * This class is the starting point for the service.  It is called and determines if the glass has a connection
 * to the server. If service has previously been started it will restart the service.  
 * It than displays a simple card to show that it has started.  
 *
 */
public class StartServiceActivity extends Activity {


	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		Log.d("Startup", "STARTING");
		Intent intt = new Intent(this, ROSMonitorService.class);		
		//kill the service if it is already running because we want to subscrive to a new topic.  
		stopService(intt);

		Card card1 = new Card(this);
		card1.setText("Robot Warning Monitor");

		View card1View = card1.toView();

		// Display the card we just created
		setContentView(card1View);
		intt = new Intent(this, ROSMonitorService.class);	
		startService(intt);

	}

}
