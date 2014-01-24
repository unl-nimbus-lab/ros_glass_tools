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

import com.google.android.glass.app.Card;
import com.google.android.glass.widget.CardScrollAdapter;
import com.google.android.glass.widget.CardScrollView;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;


/**
 * Class to select which field should be viewed inside of a topic.
 * This is used to select graph and text views once the topic has already been selected.  
 *
 */
public class TopicFieldSelectActivity extends Activity {

	private CardScrollView mCardScrollView;
	private List<Card> mCards;

 
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
	    super.onCreate(savedInstanceState);
	    mCards = new ArrayList<Card>();
	    
	    //get the topic and if we are selecting on a text view --if it is a text view we must add an all fields option.
	    Intent intt = getIntent();
	    final String topic = intt.getStringExtra("topic");
	    final boolean textual = intt.getBooleanExtra("textual", false);
	    
	    
	    ArrayList<String> listOfFields = intt.getStringArrayListExtra("field_names");
	    if(textual){
	    	listOfFields.add(0, "All Fields");
	    }
	    
	    //create live cards for viewing.
		for(int i=0;i< listOfFields.size();i++){
			Card card = new Card(this);
			card.setText(listOfFields.get(i));
			card.setFootnote("Topic card");
			mCards.add(card);
		}
		mCardScrollView = new CardScrollView(TopicFieldSelectActivity.this);
		
		FieldCardScrollAdapter adapter = new FieldCardScrollAdapter();
		mCardScrollView.setAdapter(adapter);
		mCardScrollView.setOnItemClickListener(new OnItemClickListener(){

			@Override
			public void onItemClick(AdapterView<?> parent, View view,
					int position, long id) {

				String result = ((Card) parent.getItemAtPosition(position)).getText();
				
				//we can simply restart the service which will cause it to set the view correctly due to the behavior
				//of starting an already running service.
				Intent intt = new Intent(TopicFieldSelectActivity.this, TopicService.class);	
				intt.putExtra("topic",topic);
				//put the graphical or text results in correctly.
				if(!textual){
					intt.putExtra("view_state", TopicService.GRAPH);
					intt.putExtra("field",result);

				}else{


					if(result.equals("All Fields")){
						intt.putExtra("view_state", TopicService.ALL_FIELDS);
					}else{
						intt.putExtra("view_state", TopicService.FIELD_FOCUS);
						intt.putExtra("field",result);
					}

				}
				startService(intt);
				finish();
			}
			
			
			
		});
		mCardScrollView.activate();
		setContentView(mCardScrollView);

	}
	
	/**
	 * 
	 * Class to allow card view.
	 */
	private class FieldCardScrollAdapter extends CardScrollAdapter {
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
	

}
