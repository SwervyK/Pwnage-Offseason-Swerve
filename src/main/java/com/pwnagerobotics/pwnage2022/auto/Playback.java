package com.pwnagerobotics.pwnage2022.auto;

import com.pwnagerobotics.pwnage2022.auto.Action.RobotState;

public class Playback {

    private Action[] mActions;
    private int mCurrentAction;
    private boolean mLoop;

    public void startAction(Action[] actions, boolean loop) {
        mActions = actions;
        mCurrentAction = -1;
        mLoop = loop;
    }

    public Action getCurrentAction() {
        if (mCurrentAction == -1) { // Start first action
            mCurrentAction = 0;
            mActions[mCurrentAction].startAction();
        }
        if (mActions[mCurrentAction].isDone()) { // Start next action if current one is done
            mCurrentAction++;
            mActions[mCurrentAction>=mActions.length?mActions.length-1:mCurrentAction].startAction();
        }
        if (mCurrentAction >= mActions.length) {
            if (mLoop) {
                mCurrentAction = 0;
                mActions[0].startAction();
                return mActions[0];
            } else {
                mCurrentAction--;
                return new Action(new RobotState(0, 0, 0), false);
            }
        }
        return mActions[mCurrentAction];
    }
}
