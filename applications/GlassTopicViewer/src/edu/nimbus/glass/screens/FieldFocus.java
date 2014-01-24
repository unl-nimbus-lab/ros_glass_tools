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
package edu.nimbus.glass.screens;

import android.graphics.Canvas;
import android.text.TextPaint;
import android.util.Log;
import edu.nimbus.glass.TopicManager;


/**
 * This class handles drawing when a single field is to be viewed in text form. 
 */
public class FieldFocus implements Screen{

	private String field;
	
	public FieldFocus(String field){
		this.field = field;
	}

	@Override
	public void draw(TopicManager tm, TextPaint mTopicPaint, Canvas canvas, int width, int height) {
		String load = tm.getField(field);
		mTopicPaint.setTextSize((float)100);		
		if(load.contains(".")){
			load = load.substring(0, load.indexOf(".")+5);
		}
		canvas.drawText(load, 30, (float) 100, mTopicPaint);
	}		
}
