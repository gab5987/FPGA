// WISHBONE master that brings up the OpenCores SJA1000-compatible core
// (PeliCAN/extended, 250 kbit/s) and, on each tx_start, writes the TX buffer
// with a 29-bit extended frame and issues a Transmit Request.

module can_host_fsm (
    input  logic        clk,
    input  logic        rst,

    input  logic [28:0] tx_id,
    input  logic [3:0]  tx_dlc,
    input  logic [63:0] tx_data,    // byte1 = [63:56] .. byte8 = [7:0]
    input  logic        tx_start,
    output logic        busy,
    output logic        cfg_done,

    output logic [7:0]  wb_adr_o,
    output logic [7:0]  wb_dat_o,
    input  logic [7:0]  wb_dat_i,
    output logic        wb_we_o,
    output logic        wb_cyc_o,
    output logic        wb_stb_o,
    input  logic        wb_ack_i
);

    // addr 16 is aliased: acceptance code (reset mode) vs TX buffer (operating),
    // so these can't be a unique-valued enum.
    localparam logic [7:0] A_MOD  = 8'd0;
    localparam logic [7:0] A_CMR  = 8'd1;
    localparam logic [7:0] A_BTR0 = 8'd6;
    localparam logic [7:0] A_BTR1 = 8'd7;
    localparam logic [7:0] A_OCR  = 8'd8;
    localparam logic [7:0] A_ACR0 = 8'd16;
    localparam logic [7:0] A_AMR0 = 8'd20;
    localparam logic [7:0] A_TXB  = 8'd16;
    localparam logic [7:0] A_CDR  = 8'd31;

    // 250 kbit/s @ 25 MHz CAN clock (10 tq/bit, sample point 80%).
    localparam logic [7:0] BTR0_VAL = 8'h04;
    localparam logic [7:0] BTR1_VAL = 8'h16;

    localparam int CFG_STEPS = 14;
    localparam int TX_STEPS  = 14;

    typedef enum logic [1:0] { S_CONFIG, S_IDLE, S_TX } state_e;

    state_e     state;
    logic [5:0] step;
    logic       bus_active;

    logic [28:0] id_l;
    logic [3:0]  dlc_l;
    logic [63:0] data_l;

    logic [7:0] sel_addr;
    logic [7:0] sel_data;

    function automatic logic [7:0] data_byte(input logic [2:0] idx);
        return data_l[63 - idx*8 -: 8];
    endfunction

    always_comb begin
        sel_addr = 8'h00;
        sel_data = 8'h00;
        if (state == S_CONFIG) begin
            unique case (step)
                6'd0:  begin sel_addr = A_MOD;         sel_data = 8'h01;    end // enter reset mode
                6'd1:  begin sel_addr = A_CDR;         sel_data = 8'h80;    end // PeliCAN/extended
                6'd2:  begin sel_addr = A_BTR0;        sel_data = BTR0_VAL; end
                6'd3:  begin sel_addr = A_BTR1;        sel_data = BTR1_VAL; end
                6'd4:  begin sel_addr = A_OCR;         sel_data = 8'h1A;    end // push-pull, normal
                6'd5:  begin sel_addr = A_ACR0 + 8'd0; sel_data = 8'h00;    end
                6'd6:  begin sel_addr = A_ACR0 + 8'd1; sel_data = 8'h00;    end
                6'd7:  begin sel_addr = A_ACR0 + 8'd2; sel_data = 8'h00;    end
                6'd8:  begin sel_addr = A_ACR0 + 8'd3; sel_data = 8'h00;    end
                6'd9:  begin sel_addr = A_AMR0 + 8'd0; sel_data = 8'hFF;    end // accept everything
                6'd10: begin sel_addr = A_AMR0 + 8'd1; sel_data = 8'hFF;    end
                6'd11: begin sel_addr = A_AMR0 + 8'd2; sel_data = 8'hFF;    end
                6'd12: begin sel_addr = A_AMR0 + 8'd3; sel_data = 8'hFF;    end
                6'd13: begin sel_addr = A_MOD;         sel_data = 8'h08;    end // leave reset, single filter
                default: ;
            endcase
        end else begin
            unique case (step)
                6'd0:  begin sel_addr = A_TXB + 8'd0;  sel_data = {1'b1,1'b0,2'b00,dlc_l}; end // FF=1, RTR=0, DLC
                6'd1:  begin sel_addr = A_TXB + 8'd1;  sel_data = id_l[28:21];           end
                6'd2:  begin sel_addr = A_TXB + 8'd2;  sel_data = id_l[20:13];           end
                6'd3:  begin sel_addr = A_TXB + 8'd3;  sel_data = id_l[12:5];            end
                6'd4:  begin sel_addr = A_TXB + 8'd4;  sel_data = {id_l[4:0],3'b000};    end
                6'd5:  begin sel_addr = A_TXB + 8'd5;  sel_data = data_byte(3'd0);       end
                6'd6:  begin sel_addr = A_TXB + 8'd6;  sel_data = data_byte(3'd1);       end
                6'd7:  begin sel_addr = A_TXB + 8'd7;  sel_data = data_byte(3'd2);       end
                6'd8:  begin sel_addr = A_TXB + 8'd8;  sel_data = data_byte(3'd3);       end
                6'd9:  begin sel_addr = A_TXB + 8'd9;  sel_data = data_byte(3'd4);       end
                6'd10: begin sel_addr = A_TXB + 8'd10; sel_data = data_byte(3'd5);       end
                6'd11: begin sel_addr = A_TXB + 8'd11; sel_data = data_byte(3'd6);       end
                6'd12: begin sel_addr = A_TXB + 8'd12; sel_data = data_byte(3'd7);       end
                6'd13: begin sel_addr = A_CMR;         sel_data = 8'h01;                 end // transmit request
                default: ;
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state      <= S_CONFIG;
            step       <= '0;
            bus_active <= 1'b0;
            busy       <= 1'b1;
            cfg_done   <= 1'b0;
            wb_cyc_o   <= 1'b0;
            wb_stb_o   <= 1'b0;
            wb_we_o    <= 1'b0;
            wb_adr_o   <= 8'h00;
            wb_dat_o   <= 8'h00;
        end else begin
            unique case (state)
                S_CONFIG: begin
                    busy <= 1'b1;
                    if (!bus_active) begin
                        wb_adr_o   <= sel_addr;
                        wb_dat_o   <= sel_data;
                        wb_we_o    <= 1'b1;
                        wb_cyc_o   <= 1'b1;
                        wb_stb_o   <= 1'b1;
                        bus_active <= 1'b1;
                    end else if (wb_ack_i) begin
                        wb_cyc_o   <= 1'b0;
                        wb_stb_o   <= 1'b0;
                        wb_we_o    <= 1'b0;
                        bus_active <= 1'b0;
                        if (step == CFG_STEPS-1) begin
                            step     <= '0;
                            state    <= S_IDLE;
                            cfg_done <= 1'b1;
                        end else begin
                            step <= step + 1'b1;
                        end
                    end
                end

                S_IDLE: begin
                    busy <= 1'b0;
                    if (tx_start) begin
                        id_l   <= tx_id;
                        dlc_l  <= tx_dlc;
                        data_l <= tx_data;
                        step   <= '0;
                        state  <= S_TX;
                        busy   <= 1'b1;
                    end
                end

                S_TX: begin
                    busy <= 1'b1;
                    if (!bus_active) begin
                        wb_adr_o   <= sel_addr;
                        wb_dat_o   <= sel_data;
                        wb_we_o    <= 1'b1;
                        wb_cyc_o   <= 1'b1;
                        wb_stb_o   <= 1'b1;
                        bus_active <= 1'b1;
                    end else if (wb_ack_i) begin
                        wb_cyc_o   <= 1'b0;
                        wb_stb_o   <= 1'b0;
                        wb_we_o    <= 1'b0;
                        bus_active <= 1'b0;
                        if (step == TX_STEPS-1) begin
                            step  <= '0;
                            state <= S_IDLE;
                        end else begin
                            step <= step + 1'b1;
                        end
                    end
                end

                default: state <= S_CONFIG;
            endcase
        end
    end

endmodule
