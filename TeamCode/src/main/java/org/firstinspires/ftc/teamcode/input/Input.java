package org.firstinspires.ftc.teamcode.input;

public final class Input {

    private Input(){}

    public static enum Source{
        LEFT_BUMPER, RIGHT_BUMPER,
        RIGHT_STICK_BUTTON, LEFT_STICK_BUTTON,
        DPAD_UP, DPAD_DOWN,
        DPAD_LEFT, DPAD_RIGHT,
        A,B
        /*...*/
    }

    static class Wrapper{
        Input.Source source;
        Input.Listener listener;
        boolean lastInput;

        Wrapper(Input.Source source, Input.Listener listener){
            this.source = source;
            this.listener = listener;
            this.lastInput = false;
        }

        private Wrapper(){} //can't be used

        void callInput() {
            if(!this.lastInput)
                listener.input();
            this.lastInput = true;
        }
    }

    public interface Listener{
        void input();
    }
}
