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
                callListeners(gamepad.left_bumper,Input.Source.LEFT_BUMPER);
                callListeners(gamepad.right_bumper,Input.Source.RIGHT_BUMPER);
                callListeners(gamepad.dpad_down,Input.Source.DPAD_DOWN);
                callListeners(gamepad.dpad_up,Input.Source.DPAD_UP);
                callListeners(gamepad.dpad_left,Input.Source.DPAD_LEFT);
                callListeners(gamepad.dpad_right,Input.Source.DPAD_RIGHT);
                callListeners(gamepad.right_stick_button,Input.Source.RIGHT_STICK_BUTTON);
                callListeners(gamepad.left_stick_button,Input.Source.LEFT_STICK_BUTTON);
                callListeners(gamepad.a,Input.Source.A);
                callListeners(gamepad.b,Input.Source.B);
            for(Runnable r : iterationRunnables)
                r.run();
        }
    }

    public void registerListener(Input.Source source, Input.Listener listener){
        wrappers.add(new Input.Wrapper(source, listener));
    }

    private void callListeners(boolean active, Input.Source source){
            for (Input.Wrapper wrapper : wrappers)
                if(wrapper.source == source)
                    if (active) {
                        wrapper.callInput();
                    }else
                        wrapper.lastInput = false;


    }

    public void addIterationRunnable(Runnable runnable){
        iterationRunnables.add(runnable);
    }
}
