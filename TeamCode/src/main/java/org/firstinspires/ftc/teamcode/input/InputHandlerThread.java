package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

public class InputHandlerThread extends Thread implements Runnable {
    private volatile static int threads = 0;
    private volatile Gamepad gamepad;
    private volatile List<Input.Wrapper> wrappers;
    private volatile List<Runnable> iterationRunnables;
    private final LinearOpMode op;

    public InputHandlerThread(LinearOpMode op, Gamepad gamepad){
        super("InputHandlerThread_" + ++threads);
        this.op = op;
        this.gamepad = gamepad;
        this.wrappers = new CopyOnWriteArrayList<>();
        this.iterationRunnables = new CopyOnWriteArrayList<>();
    }

    @Override
    public void run(){
        while(op.opModeIsActive()){

            if(gamepad.left_bumper)
                callListeners(Input.Source.LEFT_BUMPER);
            if(gamepad.right_bumper)
                callListeners(Input.Source.RIGHT_BUMPER);
            if(gamepad.dpad_down)
                callListeners(Input.Source.DPAD_DOWN);
            if(gamepad.dpad_up)
                callListeners(Input.Source.DPAD_UP);
            if(gamepad.dpad_left)
                callListeners(Input.Source.DPAD_LEFT);
            if(gamepad.dpad_right)
                callListeners(Input.Source.DPAD_RIGHT);
            if(gamepad.right_stick_button)
                callListeners(Input.Source.RIGHT_STICK_BUTTON);
            if(gamepad.left_stick_button)
                callListeners(Input.Source.LEFT_STICK_BUTTON);
            if(gamepad.a)
                callListeners(Input.Source.A);
            if(gamepad.b)
                callListeners(Input.Source.B);
            for(Runnable r : iterationRunnables)
                r.run();
        }
    }

    public void registerListener(Input.Source source, Input.Listener listener){
        wrappers.add(new Input.Wrapper(source, listener));
    }

    private void callListeners(Input.Source source){
        for (Input.Wrapper wrapper : wrappers)
            if(wrapper.source == source)
                wrapper.callInput();
            else
                wrapper.lastInput = false;
    }

    public void addIterationRunnable(Runnable runnable){
        iterationRunnables.add(runnable);
    }
}
