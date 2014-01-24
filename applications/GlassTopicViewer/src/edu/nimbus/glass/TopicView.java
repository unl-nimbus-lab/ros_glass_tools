/*
 * Copyright (C) 2013 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
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

import java.io.File;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.animation.ValueAnimator;
import android.animation.ValueAnimator.AnimatorUpdateListener;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.text.TextPaint;
import android.util.AttributeSet;
import android.view.View;
import android.view.animation.LinearInterpolator;

import edu.nimbus.glass.screens.AllTopics;
import edu.nimbus.glass.screens.Screen;
import edu.nimbus.glass.util.MathUtils;

/**
 * TopicView
 */
public class TopicView extends View {

	/** Various dimensions and other drawing-related constants. */

	/**
	 * If the difference between two consecutive headings is less than this value, the canvas will
	 * be redrawn immediately rather than animated.
	 */
	private static final float MIN_DISTANCE_TO_ANIMATE = 15.0f;

	/** The actual heading that represents the direction that the user is facing. */
	private float mHeading;

	/**
	 * Represents the heading that is currently being displayed when the view is drawn. This is
	 * used during animations, to keep track of the heading that should be drawn on the current
	 * frame, which may be different than the desired end point.
	 */
	private float mAnimatedHeading;

	private TopicManager topicManager;

	private final TextPaint mTopicPaint;
	private final ValueAnimator mAnimator;
	
	private Screen currentScreen;

	public TopicView(Context context) {
		this(context, null, 0);
	}

	public TopicView(Context context, AttributeSet attrs) {
		this(context, attrs, 0);
	}

	public TopicView(Context context, AttributeSet attrs, int defStyle) {
		super(context, attrs, defStyle);


		mTopicPaint = new TextPaint();
		mTopicPaint.setStyle(Paint.Style.FILL);
		mTopicPaint.setAntiAlias(true);
		mTopicPaint.setColor(Color.WHITE);
		mTopicPaint.setTypeface(Typeface.createFromFile(new File("/system/glass_fonts",
				"Roboto-Light.ttf")));


        //Nan for the first time
		mAnimatedHeading = Float.NaN;

		mAnimator = new ValueAnimator();
		setupAnimator();
		
		currentScreen = new AllTopics();
	}

	/**
	 * Sets the instance of} that this view will use to get the current
	 * heading and location.
	 *
	 * @param orientationManager the instance of {@code OrientationManager} that this view will use
	 */
	public void setTopicManager(TopicManager tManager) {
		topicManager = tManager;
	}


	@Override
	protected void onDraw(Canvas canvas) {
		//Draw the topic.
		super.onDraw(canvas);
		currentScreen.draw(topicManager, mTopicPaint, canvas, getWidth(), getHeight());
	}


	/**
	 * when the distance between two sensor events is large.
	 */
	private void setupAnimator() {
		mAnimator.setInterpolator(new LinearInterpolator());
		mAnimator.setDuration(250);

		// Notifies us at each frame of the animation so we can redraw the view.
		mAnimator.addUpdateListener(new AnimatorUpdateListener() {

			@Override
			public void onAnimationUpdate(ValueAnimator animator) {
				mAnimatedHeading = MathUtils.mod((Float) mAnimator.getAnimatedValue(), 360.0f);
				invalidate();
			}
		});

		// Notifies us when the animation is over. During an animation, the user's head may have
		// continued to move to a different orientation than the original destination angle of the
		// animation. Since we can't easily change the animation goal while it is running, we call
		// animateTo() again, which will either redraw at the new orientation (if the difference is
		// small enough), or start another animation to the new heading. This seems to produce
		// fluid results.
		mAnimator.addListener(new AnimatorListenerAdapter() {

			@Override
			public void onAnimationEnd(Animator animator) {
				animateTo(mHeading);
			}
		});
	}

	/**
	 * Animates the stuff.
	 *
	 * @param end the desired heading
	 */
	private void animateTo(float end) {
		// I'm not sure if this method is needed anymore but I'm still including it here. 
		
		if (!mAnimator.isRunning()) {
			float start = mAnimatedHeading;
			float distance = Math.abs(end - start);
			float reverseDistance = 360.0f - distance;
			float shortest = Math.min(distance, reverseDistance);

			if (Float.isNaN(mAnimatedHeading) || shortest < MIN_DISTANCE_TO_ANIMATE) {

				mAnimatedHeading = end;
				invalidate();
			} else {

				float goal;

				if (distance < reverseDistance) {
					goal = end;
				} else if (end < start) {
					goal = end + 360.0f;
				} else {
					goal = end - 360.0f;
				}

				mAnimator.setFloatValues(start, goal);
				mAnimator.start();
			}
		}
	}

	public void switchScreen(Screen newScreen) {
		this.currentScreen = newScreen;
	}
}
