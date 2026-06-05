// SPI master for the DE0-Nano ADC128S022 (8-ch, 12-bit). Round-robins all
// channels; results are exposed per channel in adc_data[].

module adc_spi_master #(
    parameter int SCLK_DIV = 16   // sys-clk cycles per SCLK half-period (-> ~1.56 MHz)
)(
    input  logic        clk,
    input  logic        rst,

    output logic        adc_cs_n,
    output logic        adc_sclk,
    output logic        adc_saddr,
    input  logic        adc_sdat,

    output logic [11:0] adc_data [8],
    output logic [2:0]  last_channel,
    output logic        sample_valid
);

    localparam int DIVW = (SCLK_DIV <= 1) ? 1 : $clog2(SCLK_DIV);

    logic [DIVW-1:0] div_cnt;
    logic            sclk_tick;

    always_ff @(posedge clk) begin
        if (rst) begin
            div_cnt   <= '0;
            sclk_tick <= 1'b0;
        end else if (div_cnt == SCLK_DIV-1) begin
            div_cnt   <= '0;
            sclk_tick <= 1'b1;
        end else begin
            div_cnt   <= div_cnt + 1'b1;
            sclk_tick <= 1'b0;
        end
    end

    logic [3:0]  rise_cnt;
    logic [15:0] shift_in;
    logic [2:0]  req_channel;
    logic [2:0]  res_channel;   // channel whose result is arriving (request is pipelined by one frame)
    logic        running;

    wire [15:0] ctrl_word = {2'b00, req_channel, 11'b0};  // ADD2..0 in bits [13:11]
    wire [15:0] word_now  = {shift_in[14:0], adc_sdat};

    always_ff @(posedge clk) begin
        if (rst) begin
            adc_cs_n     <= 1'b1;
            adc_sclk     <= 1'b1;
            adc_saddr    <= 1'b0;
            rise_cnt     <= '0;
            shift_in     <= '0;
            req_channel  <= '0;
            res_channel  <= '0;
            running      <= 1'b0;
            last_channel <= '0;
            sample_valid <= 1'b0;
            adc_data     <= '{default: '0};
        end else begin
            sample_valid <= 1'b0;

            if (!running) begin
                adc_cs_n  <= 1'b0;
                adc_sclk  <= 1'b1;
                rise_cnt  <= '0;
                adc_saddr <= ctrl_word[15];
                running   <= 1'b1;
            end else if (sclk_tick) begin
                adc_sclk <= ~adc_sclk;

                // ADC samples DIN / launches DOUT on falling edges, so the
                // master drives DIN and samples DOUT on rising edges.
                if (!adc_sclk) begin
                    shift_in  <= word_now;
                    adc_saddr <= ctrl_word[14 - rise_cnt];

                    if (rise_cnt == 4'd15) begin
                        adc_cs_n              <= 1'b1;
                        running               <= 1'b0;
                        res_channel           <= req_channel;
                        req_channel           <= req_channel + 1'b1;
                        adc_data[res_channel] <= word_now[11:0];
                        last_channel          <= res_channel;
                        sample_valid          <= 1'b1;
                    end else begin
                        rise_cnt <= rise_cnt + 1'b1;
                    end
                end
            end
        end
    end

endmodule
