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
import edu.nimbus.glass.TopicManager;


/**
 * This class handles drawing when all topics are to be displayed.
 *
 */
public class AllTopics implements Screen{

	public AllTopics(){

	}

	@Override
	public void draw(TopicManager tm, TextPaint mTopicPaint, Canvas canvas, int width, int height) {
		String text = tm.getText();
		String[] arr = text.split("\n");
		//every field gets screen_height / # of fields space.
		double dist = height / (arr.length);
		mTopicPaint.setTextSize((float)dist);
		for(int i=0;i< arr.length;i++){
			canvas.drawText(TopicManager.performFormatting(arr[i]), 0, (float)  ((i+1) * dist) , mTopicPaint);
		}
	}
}
