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

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import android.graphics.Canvas;
import android.graphics.Color;
import android.text.TextPaint;
import android.util.DisplayMetrics;
import android.util.Log;
import edu.nimbus.glass.TopicManager;


/**
 * Class to draw line graph of recent  values of a topic's selected field on the display.  
 * The bounds of the graph are dynamically updated to fit the recently recieved data.  
 * THis means that when a new value is recieved that is larger than the bounds the graphical range will be expanded.
 * After a while the graph shrinks the range to fit the mose recent data recieved.  
 *
 */
public class Graph implements Screen {

	private static final int MAX_POINTS = 100;
	private static final int MILLISECONDS_REDRAW_GRAPH = 5000;
	
	private String field;
	private List<Double> pastValues;
	
	private double recentMax;
	private double recentMin;
	private double min;
	private double max;
	private long counter;

	public Graph(String field){
		this.field = field;
		this.pastValues = new LinkedList<Double>();

		recentMax = Double.MAX_VALUE;
		recentMin = Double.MIN_VALUE;
		min = 0;
		max = 1;
		counter = 0;
	}

	@Override
	public void draw(TopicManager tm, TextPaint mPaint, Canvas canvas, int width, int height) {
		canvas.drawColor(Color.BLACK);
		try{
			//get out the value
			double val = Double.parseDouble(tm.getField(field));
			pastValues.add(val);
			
			//we are at a point that we can update the max and min values
			if(System.currentTimeMillis() - counter > MILLISECONDS_REDRAW_GRAPH){
				counter = System.currentTimeMillis();
				
				min = recentMin;
				max = recentMax;
				
				recentMin = Float.MAX_VALUE;
				recentMax = Float.MIN_VALUE;
			}
			
			if(val < min){
				min = val;
			}
			
			if(val > max){
				max = val;
			}
			
			if(val < recentMin){
				recentMin = val;
			}
			
			if(val > recentMax){
				recentMax = val;
			}
			
		
			//Prune data.
			if(pastValues.size() > MAX_POINTS){
				pastValues.remove(0);
			}
			
			double vertdataRange = max - min;
			double hrange = pastValues.size();
			double horRatio = width / hrange;
			double vertRatio = height / vertdataRange;
			

			int brightness = 10;

			float lastX = 0;
			float lastY = (float) (vertRatio * pastValues.get(0) );
			Log.d("MAXMIN", min+" " + max);
			for(int i=0; i<pastValues.size(); i++){
				mPaint.setColor(Color.argb(brightness, 128, 255, 192));

				// For efficiency, we don't draw all of the samples in the buffer,
				// but only the ones that align with pixel boundaries.
				float x = (float) (i*(horRatio));  
				float y = height - (float) ((pastValues.get(i).floatValue() - min) * vertRatio);
			
				canvas.drawLine(lastX, lastY, x, y, mPaint);
			
				lastX = x;
				lastY = y;

				brightness += 3;
			}
			
			mPaint.setTextSize((float)50);
			
			canvas.drawText(String.format("%.4f", max), 0, 40, mPaint);
			canvas.drawText(String.format("%.4f", min), 0, height, mPaint);
						
			counter ++;
		}catch(java.lang.Exception e){
			//can't graph it.
		}
	}


}
