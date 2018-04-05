module Project(
    input [3:0] KEY,
    input [17:0] SW,
    input CLOCK_50,
    inout [15:0] GPIO,
    output [17:0] LEDR,
    output [7:0] LEDG,
    output [6:0] HEX0,
    output [6:0] HEX1,
    output [6:0] HEX2,
    output [6:0] HEX3,
    output [6:0] HEX4,
    output [6:0] HEX5,
    output [6:0] HEX6,
    output [6:0] HEX7,
    // The ports below are for the VGA output.
    output  VGA_CLK,                        //  VGA Clock
    output VGA_HS,                          //  VGA H_SYNC
    output VGA_VS,                          //  VGA V_SYNC
    output VGA_BLANK_N,                     //  VGA BLANK
    output VGA_SYNC_N,                      //  VGA SYNC
    output [9:0] VGA_R,                         //  VGA Red[9:0]
    output [9:0] VGA_G,                         //  VGA Green[9:0]
    output [9:0] VGA_B                          //  VGA Blue[9:0]
    );

    // Input wires
    wire reset = SW[0];
    wire clk = CLOCK_50;
    wire echo = GPIO[1];
    // Ultrasonic sensor
    wire obstacle_detected;
    wire trig;
    wire [3:0] signal;

    sensor sensor_instance(
        .clk(CLOCK_50),
        .trig(trig),
        .echo(echo),
        .obstacle(obstacle_detected)
    );

    // Motor control
    wire motor_left_active;
    wire motor_left_reverse;
    wire motor_left_s1;
    wire motor_left_s2;
    wire motor_left_s3;
    wire motor_left_s4;
    wire motor_right_active;
    wire motor_right_reverse;
    wire motor_right_s1;
    wire motor_right_s2;
    wire motor_right_s3;
    wire motor_right_s4;

    motor motor_left(
        .clk(clk),
        .active(motor_left_active),
        .reverse(motor_left_reverse),
        .s1(motor_left_s1),
        .s2(motor_left_s2),
        .s3(motor_left_s3),
        .s4(motor_left_s4)
    );

    motor motor_right(
        .clk(clk),
        .active(motor_right_active),
        .reverse(motor_right_reverse),
        .s1(motor_right_s1),
        .s2(motor_right_s2),
        .s3(motor_right_s3),
        .s4(motor_right_s4)
    );

    // Distance counter
    wire [63:0] distance_count;
    wire [63:0] distance_raw;
    reg [31:0] distance_display;
    wire distance_division_done;

    localparam CYCLES_PER_CENTIMETRE = 64'd10_000_000;

    initial
        distance_display <= 32'b0;

    always@(posedge distance_division_done)
        distance_display <= distance_raw[31:0];

    simple_divider_64 simple_divider_64_distance(
        .clk(CLOCK_50),
        .reset(reset),
        .dividend(distance_count),
        .divisor(CYCLES_PER_CENTIMETRE),
        .done(distance_division_done),
        .quotient(distance_raw),
        .remainder()
    );

    wire [6:0] hex_out [7:0];

    hex_display hex_display_0(.IN(distance_display[3:0]), .OUT(hex_out[0]));
    hex_display hex_display_1(.IN(distance_display[7:4]), .OUT(hex_out[1]));
    hex_display hex_display_2(.IN(distance_display[11:8]), .OUT(hex_out[2]));
    hex_display hex_display_3(.IN(distance_display[15:12]), .OUT(hex_out[3]));
    hex_display hex_display_4(.IN(distance_display[19:16]), .OUT(hex_out[4]));
    hex_display hex_display_5(.IN(distance_display[23:20]), .OUT(hex_out[5]));
    hex_display hex_display_6(.IN(distance_display[27:24]), .OUT(hex_out[6]));
    hex_display hex_display_7(.IN(distance_display[31:28]), .OUT(hex_out[7]));

    // Control unit
    control control_instance(
        .clk(clk),
        .reset(SW[0]),
        .start(SW[17]),
        .obstacle_detected(obstacle_detected),
        .distance_count(distance_count),
        .motor_left_active(motor_left_active),
        .motor_left_reverse(motor_left_reverse),
        .motor_right_active(motor_right_active),
        .motor_right_reverse(motor_right_reverse),
        .motor_signal(signal),
        .diag0(LEDG[1]),
        .diag1(LEDG[2]),
        .diag2(LEDG[3]),
        .diag3(LEDG[4]),
        .diag4(LEDR[7:4]),
        .diag5(LEDR[11:8]),
        .diag6(LEDR[13])
    );

    // Output wires
    assign GPIO[0] = trig;
    assign HEX0 = hex_out[0];
    assign HEX1 = hex_out[1];
    assign HEX2 = hex_out[2];
    assign HEX3 = hex_out[3];
    assign HEX4 = hex_out[4];
    assign HEX5 = hex_out[5];
    assign HEX6 = hex_out[6];
    assign HEX7 = hex_out[7];
    assign GPIO[2] = motor_left_s1;
    assign GPIO[3] = motor_left_s2;
    assign GPIO[4] = motor_left_s3;
    assign GPIO[5] = motor_left_s4;
    assign GPIO[6] = motor_right_s1;
    assign GPIO[7] = motor_right_s2;
    assign GPIO[8] = motor_right_s3;
    assign GPIO[9] = motor_right_s4;

    // Status indicators
    assign LEDG[0] = obstacle_detected;
    assign LEDR[0] = motor_right_active;
    assign LEDR[1] = motor_right_reverse;
    assign LEDR[2] = motor_left_reverse;
    assign LEDR[3] = motor_left_active;
endmodule

module control(
    input clk,
    input reset,
    input start,
    input obstacle_detected,
    output reg [63:0] distance_count,
    output reg motor_left_active,
    output reg motor_left_reverse,
    output reg motor_right_active,
    output reg motor_right_reverse,
    output reg [3:0] motor_signal,

    output reg diag0,
    output diag1,
    output diag2,
    output diag3,
    output [3:0] diag4,
    output [3:0] diag5,
    output diag6
    );

    // State registers
    reg [3:0] current_state, next_state, prev_state;

    reg correct_next_state_STOP;
    reg correct_next_state_MOVE_FORWARD;
    reg correct_next_state_MOVE_REVERSE;
    reg correct_next_state_TURN_RIGHT;
    reg correct_next_state_TURN_LEFT;
    reg correct_next_state_MOVE_RIGHT;
    reg correct_next_state_MOVE_LEFT;
    reg correct_next_state_MOVE_BACKWARD;
    reg correct_next_state_TURN_BACK;

    reg [3:0] correct_next_state;

    assign diag4 = current_state;
    assign diag5 = correct_next_state;

    // Timing parameters
    reg reverse_active, turn_active, avoid_active, turn_back_active, state_change_active;
    wire turn_complete, reverse_complete, avoid_complete, turn_back_complete, state_change_complete;
    localparam REVERSE_CYCLES      = 32'd150_000_000, // 1500ms
               TURN_CYCLES         = 32'd150_000_000, // 0.4s
               AVOID_CYCLES        = 32'd150_000_000, // 3s
               STATE_CHANGE_CYCLES = 32'd2_500_000;   // 0.5ms

    // Timers
    timer_32 reverse_timer(
        .clk(clk),
        .n_cycles(REVERSE_CYCLES),
        .active(reverse_active),
        .complete(reverse_complete)
    );

    timer_32 turn_timer(
        .clk(clk),
        .n_cycles(TURN_CYCLES),
        .active(turn_active),
        .complete(turn_complete)
    );

    timer_32 turn_back_timer(
        .clk(clk),
        .n_cycles(TURN_CYCLES),
        .active(turn_back_active),
        .complete(turn_back_complete)
    );

    timer_32 avoid_timer(
        .clk(clk),
        .n_cycles(AVOID_CYCLES),
        .active(avoid_active),
        .complete(avoid_complete)
    );

     timer_32 state_change_timer(
            .clk(clk),
            .n_cycles(STATE_CHANGE_CYCLES),
            .active(state_change_active),
            .complete(state_change_complete)
    );

    // Diagnostic outputs
    assign diag1 = turn_complete;
    assign diag2 = reverse_complete;
    assign diag3 = avoid_complete;
     assign diag6 = state_change_complete;

    // States
    localparam S_STOP          = 4'd0,
               S_MOVE_FORWARD  = 4'd1,
               S_MOVE_RIGHT    = 4'd2,
               S_MOVE_LEFT     = 4'd3,
               S_MOVE_BACKWARD = 4'd4,
               S_MOVE_REVERSE  = 4'd5,
               S_TURN_RIGHT    = 4'd6,
               S_TURN_LEFT     = 4'd7,
               S_TURN_BACK     = 4'd8;

    // Initial state
    initial begin
        current_state = S_STOP;
        correct_next_state = S_STOP;
        prev_state = S_STOP;
        correct_next_state_STOP = 1'b1;
        correct_next_state_MOVE_FORWARD= 1'b0;
        correct_next_state_MOVE_REVERSE= 1'b0;
        correct_next_state_TURN_RIGHT= 1'b0;
        correct_next_state_TURN_LEFT= 1'b0;
        correct_next_state_MOVE_RIGHT= 1'b0;
        correct_next_state_MOVE_LEFT= 1'b0;
        correct_next_state_MOVE_BACKWARD= 1'b0;
        correct_next_state_TURN_BACK= 1'b0;
        state_change_active = 1'b0;
    end

    // Timer control
    always@(*) begin
        reverse_active <= 1'b0;
        turn_active <= 1'b0;
        turn_back_active <= 1'b0;
        avoid_active <= 1'b0;

        case (current_state)
            S_MOVE_RIGHT, S_MOVE_LEFT, S_MOVE_BACKWARD: avoid_active <= 1'b1;
            S_MOVE_REVERSE: reverse_active <= 1'b1;
            S_TURN_RIGHT, S_TURN_LEFT: turn_active <= 1'b1;
            S_TURN_BACK: turn_back_active <= 1'b1;
        endcase
    end

    // State control (FSM)
    always@(posedge clk) begin
        case (current_state)
        S_STOP: begin
            if(correct_next_state_STOP && state_change_complete) begin
                state_change_active = 1'b0;

                if(start) begin
                    correct_next_state_STOP <= 1'b0;
                    correct_next_state_MOVE_FORWARD <= 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_MOVE_FORWARD;
                end else
                    correct_next_state <= S_STOP;
            end
        end
        S_MOVE_FORWARD: begin
            if(correct_next_state_MOVE_FORWARD && state_change_complete) begin
                state_change_active = 1'b0;

                if(obstacle_detected) begin
                    correct_next_state_MOVE_FORWARD = 1'b0;
                    correct_next_state_MOVE_REVERSE = 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_MOVE_REVERSE;
                end else
                    correct_next_state <= S_MOVE_FORWARD;
            end
        end
        S_MOVE_RIGHT: begin
            if(correct_next_state_MOVE_RIGHT && state_change_complete) begin
                state_change_active = 1'b0;

                if(obstacle_detected) begin
                    correct_next_state_MOVE_RIGHT = 1'b0;
                    correct_next_state_MOVE_REVERSE = 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_MOVE_REVERSE;
                end else if (avoid_complete) begin
                    correct_next_state_MOVE_RIGHT = 1'b0;
                    correct_next_state_TURN_LEFT = 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_TURN_LEFT;
                end else
                    correct_next_state = S_MOVE_RIGHT;
            end
        end
        S_MOVE_LEFT: begin
            if(correct_next_state_MOVE_LEFT && state_change_complete) begin
                state_change_active = 1'b0;

                if(obstacle_detected) begin
                    correct_next_state_MOVE_LEFT= 1'b0;
                    correct_next_state_MOVE_REVERSE = 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_MOVE_REVERSE;
                end else if(avoid_complete) begin
                    correct_next_state_MOVE_LEFT = 1'b0;
                    correct_next_state_TURN_LEFT = 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_TURN_LEFT;
                end else
                    correct_next_state = S_MOVE_LEFT;
            end
        end
        S_MOVE_BACKWARD:  begin
            if(correct_next_state_MOVE_BACKWARD && state_change_complete) begin
                state_change_active = 1'b0;

                if(obstacle_detected) begin
                    correct_next_state_MOVE_BACKWARD = 1'b0;
                    correct_next_state_MOVE_REVERSE = 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_MOVE_REVERSE;
                end else if(avoid_complete) begin
                    correct_next_state_MOVE_BACKWARD = 1'b0;
                    correct_next_state_TURN_LEFT = 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_TURN_LEFT;
                end else
                    correct_next_state = S_MOVE_BACKWARD;
            end
        end
        S_MOVE_REVERSE: begin
            if(correct_next_state_MOVE_REVERSE && state_change_complete) begin
                state_change_active = 1'b0;

                if(reverse_complete) begin
                    correct_next_state_MOVE_REVERSE = 1'b0;
                    correct_next_state_TURN_RIGHT = 1'b1;
                    state_change_active = 1'b1;
                    correct_next_state = S_TURN_RIGHT;
                end else
                    correct_next_state <= S_MOVE_REVERSE;
            end
        end
        S_TURN_LEFT: begin
            if(correct_next_state_TURN_LEFT && state_change_complete) begin
                state_change_active = 1'b0;

                if (turn_complete) begin
                    if (obstacle_detected) begin
                        correct_next_state_TURN_LEFT = 1'b0;
                        correct_next_state_TURN_BACK = 1'b1;
                        state_change_active = 1'b1;
                        correct_next_state = S_TURN_BACK;
                    end else begin
                        case (prev_state)
                        S_MOVE_RIGHT: begin
                            correct_next_state_TURN_LEFT = 1'b0;
                            correct_next_state_MOVE_FORWARD = 1'b1;
                            state_change_active = 1'b1;
                            correct_next_state = S_MOVE_FORWARD;
                        end
                        S_MOVE_LEFT: begin
                            correct_next_state_TURN_LEFT = 1'b0;
                            correct_next_state_MOVE_BACKWARD = 1'b1;
                            state_change_active = 1'b1;
                            correct_next_state = S_MOVE_BACKWARD;
                        end
                        S_MOVE_BACKWARD: begin
                            correct_next_state_TURN_LEFT = 1'b0;
                            correct_next_state_MOVE_RIGHT = 1'b1;
                            state_change_active = 1'b1;
                            correct_next_state = S_MOVE_RIGHT;
                        end
                        default: begin
                            correct_next_state_TURN_LEFT = 1'b0;
                            correct_next_state_STOP = 1'b1;
                            correct_next_state = S_STOP;
                        end
                        endcase
                    end
                end else
                    correct_next_state <= S_TURN_LEFT;
            end
        end
        S_TURN_RIGHT: begin
            if(correct_next_state_TURN_RIGHT && state_change_complete) begin
                state_change_active = 1'b0;

                if (turn_complete) begin
                    case (prev_state)
                    S_MOVE_FORWARD: begin
                        correct_next_state_TURN_RIGHT = 1'b0;
                        correct_next_state_MOVE_RIGHT = 1'b1;
                        state_change_active = 1'b1;
                        correct_next_state = S_MOVE_RIGHT;
                    end
                    S_MOVE_LEFT: begin
                        correct_next_state_TURN_RIGHT = 1'b0;
                        correct_next_state_MOVE_FORWARD = 1'b1;
                        state_change_active = 1'b1;
                        correct_next_state = S_MOVE_FORWARD;
                    end
                    S_MOVE_RIGHT: begin
                        correct_next_state_TURN_RIGHT = 1'b0;
                        correct_next_state_MOVE_BACKWARD = 1'b1;
                        state_change_active = 1'b1;
                        correct_next_state = S_MOVE_BACKWARD;
                    end
                    S_MOVE_BACKWARD: begin
                        correct_next_state_TURN_RIGHT = 1'b0;
                        correct_next_state_MOVE_LEFT = 1'b1;
                        state_change_active = 1'b1;
                        correct_next_state = S_MOVE_LEFT;
                    end
                    default: begin
                        correct_next_state_TURN_RIGHT = 1'b0;
                        correct_next_state_STOP = 1'b1;
                        correct_next_state = S_STOP;
                    end
                    endcase
                end else
                    correct_next_state = S_TURN_RIGHT;
            end
        end
        S_TURN_BACK: begin
            if(correct_next_state_TURN_BACK && state_change_complete) begin
                state_change_active = 1'b0;

                if(turn_back_complete) begin
                    correct_next_state_TURN_BACK = 1'b0;
                    state_change_active = 1'b1;

                    case(prev_state)
                    S_MOVE_FORWARD: begin
                        correct_next_state_MOVE_FORWARD = 1'b1;
                        correct_next_state = S_MOVE_FORWARD;
                    end
                    S_MOVE_LEFT: begin
                        correct_next_state_MOVE_LEFT = 1'b1;
                        correct_next_state = S_MOVE_LEFT;
                    end
                    S_MOVE_RIGHT: begin
                        correct_next_state_MOVE_RIGHT = 1'b1;
                        correct_next_state = S_MOVE_RIGHT;
                    end
                    S_MOVE_BACKWARD: begin
                        correct_next_state_MOVE_BACKWARD = 1'b1;
                        correct_next_state = S_MOVE_BACKWARD;
                    end
                    default: begin
                        correct_next_state_STOP = 1'b1;
                        correct_next_state <= S_STOP;
                    end
                    endcase
                end else
                    correct_next_state = S_TURN_BACK;
            end
        end
    endcase
    end

    // Output control
    always@(*) begin
        case (current_state)
            S_MOVE_FORWARD, S_MOVE_RIGHT, S_MOVE_LEFT, S_MOVE_BACKWARD: begin
                motor_left_active <= 1'b1;
                motor_left_reverse <= 1'b0;
                motor_right_active <= 1'b1;
                motor_right_reverse <= 1'b0;
            end
            S_MOVE_REVERSE: begin
                motor_left_active <= 1'b1;
                motor_left_reverse <= 1'b1;
                motor_right_active <= 1'b1;
                motor_right_reverse <= 1'b1;
            end
            S_TURN_LEFT: begin
                motor_left_active <= 1'b1;
                motor_left_reverse <= 1'b1;
                motor_right_active <= 1'b1;
                motor_right_reverse <= 1'b0;
            end
            S_TURN_RIGHT: begin
                motor_left_active <= 1'b1;
                motor_left_reverse <= 1'b0;
                motor_right_active <= 1'b1;
                motor_right_reverse <= 1'b1;
            end
            S_TURN_BACK: begin
                motor_left_active <= 1'b1;
                motor_left_reverse <= 1'b0;
                motor_right_active <= 1'b1;
                motor_right_reverse <= 1'b1;

            end
            default: begin
                motor_left_active <= 1'b0;
                motor_left_reverse <= 1'b0;
                motor_right_active <= 1'b0;
                motor_right_reverse <= 1'b0;

            end
        endcase
    end

    // Distance counting
    initial
        distance_count <= 64'b0;

    always@(posedge clk)
        if(reset)
            distance_count <= 64'b0;
        else begin
            if (current_state == S_MOVE_FORWARD)
                distance_count <= distance_count + 64'b1;
            else if (current_state == S_MOVE_REVERSE)
                distance_count <= distance_count - 64'b1;
        end

    // State transition
    always@(posedge clk)
        if (reset) begin
            current_state = S_STOP;
        end else begin
            current_state <= correct_next_state;
            motor_signal <= current_state;
            case (correct_next_state)
                S_MOVE_FORWARD, S_MOVE_LEFT, S_MOVE_RIGHT, S_MOVE_BACKWARD:
                    prev_state <= correct_next_state;
                default:
                    prev_state <= prev_state;
            endcase
        end
endmodule

module timer_32(
    input clk,
    input [31:0] n_cycles,
    input active,
    output complete
    );

    reg [31:0] counter;
    reg activated;

    initial
        activated = 1'b0;

    always@(posedge clk)
        if (active) begin
            if (~activated) begin
                counter <= n_cycles - 32'b1;
                activated <= 1;
            end else if (counter > 32'b0)
                counter <= counter - 32'b1;
        end else begin
            activated <= 0;
            counter <= 32'b0;
        end

    assign complete = counter == 32'b0;
endmodule

module sensor(
    input clk,
    input echo,
    output reg trig,
    output obstacle
    );

    reg [23:0] echo_time;
    reg [22:0] counter;
    reg trig_complete;
    reg [1:0] obstacle_detected_count;

    localparam TRIGGER_WIDTH  = 23'd500, // 10 us
               TRIGGER_PERIOD = 23'd5_000_000, // 100 ms
               ECHO_THRESHOLD = 58_000; // 20 cm

    initial begin
        counter <= TRIGGER_PERIOD;
        echo_time <= 23'b0;
        obstacle_detected_count <= 2'b0;
    end

    always@(posedge clk) begin
        if (counter == TRIGGER_WIDTH) begin
            trig <= 0;
            trig_complete <= 1;
        end else if (counter == TRIGGER_PERIOD) begin
            counter <= 0;
            trig <= 1;
        end

        counter <= counter + 1;

        if (echo)
        echo_time <= echo_time + 1;
        else if (~echo && trig_complete) begin
            if (echo_time < ECHO_THRESHOLD) begin
                if (obstacle_detected_count < 2'b11)
                    obstacle_detected_count <= obstacle_detected_count + 2'b1;
            end else
                obstacle_detected_count <= 2'b00;

            echo_time <= 0;
            trig_complete <= 0;
        end
    end

    assign obstacle = obstacle_detected_count == 2'b11;
endmodule

module motor(
    input clk,
    input active,
    input reverse,
    output reg s1,
    output reg s2,
    output reg s3,
    output reg s4
    );

    initial begin
        s1 <= 1'b0;
        s2 <= 1'b0;
        s3 <= 1'b0;
        s4 <= 1'b0;
    end

    always@(posedge clk) begin
        if (active) begin
            if (!reverse) begin
                s1 <= 1'b1;
                s2 <= 1'b0;
                s3 <= 1'b0;
                s4 <= 1'b1;
            end else begin
                s1 <= 1'b0;
                s2 <= 1'b1;
                s3 <= 1'b1;
                s4 <= 1'b0;
            end
        end else begin
            s1 <= 1'b0;
            s2 <= 1'b0;
            s3 <= 1'b0;
            s4 <= 1'b0;
        end
    end
endmodule

module hex_display(IN, OUT);
    input [3:0] IN;
    output reg [6:0] OUT;

    always @(*)
        case(IN[3:0])
            4'b0000: OUT = 7'b1000000;
            4'b0001: OUT = 7'b1111001;
            4'b0010: OUT = 7'b0100100;
            4'b0011: OUT = 7'b0110000;
            4'b0100: OUT = 7'b0011001;
            4'b0101: OUT = 7'b0010010;
            4'b0110: OUT = 7'b0000010;
            4'b0111: OUT = 7'b1111000;
            4'b1000: OUT = 7'b0000000;
            4'b1001: OUT = 7'b0011000;
            4'b1010: OUT = 7'b0001000;
            4'b1011: OUT = 7'b0000011;
            4'b1100: OUT = 7'b1000110;
            4'b1101: OUT = 7'b0100001;
            4'b1110: OUT = 7'b0000110;
            4'b1111: OUT = 7'b0001110;
            default: OUT = 7'b0111111;
        endcase
endmodule

module simple_divider_64(
    input clk,
    input reset,
    input [63:0] dividend,
    input [63:0] divisor,
    output reg done,
    output reg [63:0] quotient,
    output reg [63:0] remainder
    );

    reg [1:0] current_state, next_state;
    wire divisible = remainder >= divisor;

    localparam S_LOAD     = 2'd0,
               S_SUBTRACT = 2'd1,
               S_DONE     = 2'd2;

    initial
        current_state <= S_LOAD;

    always@(*)
        case (current_state)
            S_LOAD, S_SUBTRACT:
                next_state = divisible ? S_SUBTRACT : S_DONE;
            S_DONE: next_state = S_LOAD;
            default: next_state = S_LOAD;
        endcase

    always@(posedge clk)
        if (reset)
            current_state <= S_LOAD;
        else begin
            case (current_state)
                S_LOAD: begin
                    done <= 1'b0;
                    remainder <= dividend;
                    quotient <= 32'b0;
                end
                S_SUBTRACT: begin
                    remainder <= remainder - divisor;
                    quotient <= quotient + 32'b1;
                end
                S_DONE:
                    done <= 1;
            endcase

            current_state <= next_state;
        end
endmodule
