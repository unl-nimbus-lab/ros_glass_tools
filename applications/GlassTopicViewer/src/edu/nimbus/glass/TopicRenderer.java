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


import edu.nimbus.glass.screens.AllTopics;
import edu.nimbus.glass.screens.FieldFocus;
import edu.nimbus.glass.screens.Graph;

import android.content.Context;
import android.graphics.Canvas;

import android.os.SystemClock;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.SurfaceHolder;
import android.view.View;
import android.widget.FrameLayout;

import java.util.concurrent.TimeUnit;

/**
 * The surface callback that provides the rendering logic for topic live card.
 */
public class TopicRenderer implements SurfaceHolder.Callback {

	private static final String TAG = TopicRenderer.class.getSimpleName();



	/** The refresh rate, in frames per second. */
	private static final int REFRESH_RATE_FPS = 30;

	/** The duration, in milliseconds, of one frame. */
	private static final long FRAME_TIME_MILLIS = TimeUnit.SECONDS.toMillis(1) / REFRESH_RATE_FPS;

	private SurfaceHolder mHolder;
	private RenderThread mRenderThread;
	private int mSurfaceWidth;
	private int mSurfaceHeight;

	private final FrameLayout mLayout;
	private final TopicView mTopicView;
	private final TopicManager mTopicManager;

	/**
	 * Creates new renderer.
	 */
	public TopicRenderer(Context context, TopicManager topicManager) {
		
		Log.d("Starting renderer", "render");
		
		LayoutInflater inflater = LayoutInflater.from(context);
		mLayout = (FrameLayout) inflater.inflate(R.layout.topic, null);
		mLayout.setWillNotDraw(false);

		mTopicView = (TopicView) mLayout.findViewById(R.id.topic);


		mTopicManager = topicManager;
		mTopicView.setTopicManager(mTopicManager);
	}

	public void setRendererAllFieldsState(){
		mTopicView.switchScreen(new AllTopics());
	}

	public void setRendererGraphState(String field){
		mTopicView.switchScreen(new Graph(field));
	}

	public void setRendererFieldFocusState(String field){
		mTopicView.switchScreen(new FieldFocus(field));
	}

	@Override
	public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
		mSurfaceWidth = width;
		mSurfaceHeight = height;
		doLayout();
	}

	@Override
	public void surfaceCreated(SurfaceHolder holder) {
		mHolder = holder;

		mTopicManager.start();
		mRenderThread = new RenderThread();
		mRenderThread.start();
	}

	@Override
	public void surfaceDestroyed(SurfaceHolder holder) {
		mRenderThread.quit();
		mTopicManager.stop();
	}

	/**
	 * Requests that the views redo their layout. This must be called manually every time the
	 * tips view's text is updated because this layout doesn't exist in a GUI thread where those
	 * requests will be enqueued automatically.
	 */
	private void doLayout() {
		// Measure and update the layout so that it will take up the entire surface space
		// when it is drawn.
		int measuredWidth = View.MeasureSpec.makeMeasureSpec(mSurfaceWidth,
				View.MeasureSpec.EXACTLY);
		int measuredHeight = View.MeasureSpec.makeMeasureSpec(mSurfaceHeight,
				View.MeasureSpec.EXACTLY);

		mLayout.measure(measuredWidth, measuredHeight);
		mLayout.layout(0, 0, mLayout.getMeasuredWidth(), mLayout.getMeasuredHeight());
	}

	/**
     * Repaints the view
	 */
	private synchronized void repaint() {
		Canvas canvas = null;

		try {
			canvas = mHolder.lockCanvas();
		} catch (RuntimeException e) {
			Log.d(TAG, "lockCanvas failed", e);
		}

		if (canvas != null) {
			mLayout.draw(canvas);

			try {
				mHolder.unlockCanvasAndPost(canvas);
			} catch (RuntimeException e) {
				Log.d(TAG, "unlockCanvasAndPost failed", e);
			}
		}
	}


	/**
	 * Redraws the topic in the background.
	 */
	private class RenderThread extends Thread {
		private boolean mShouldRun;

		/**
		 * Initializes the background rendering thread.
		 */
		public RenderThread() {
			mShouldRun = true;
		}

		/**
		 * Returns true if the rendering thread should continue to run.
		 *
		 * @return true if the rendering thread should continue to run
		 */
		private synchronized boolean shouldRun() {
			return mShouldRun;
		}

		/**
		 * Requests that the rendering thread exit at the next opportunity.
		 */
		public synchronized void quit() {
			mShouldRun = false;
		}

		@Override
		public void run() {
			while (shouldRun()) {
				long frameStart = SystemClock.elapsedRealtime();
				repaint();
				long frameLength = SystemClock.elapsedRealtime() - frameStart;

				long sleepTime = FRAME_TIME_MILLIS - frameLength;
				if (sleepTime > 0) {
					SystemClock.sleep(sleepTime);
				}
			}
		}
	}
}
