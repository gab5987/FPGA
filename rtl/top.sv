// CAN_ADC top level (DE0-Nano, EP4CE22F17C6N): samples the on-board ADC and
// transmits a 29-bit extended CAN frame once per second via can_host_fsm.
// The DE0-Nano has no CAN transceiver: CAN_TX/CAN_RX go to GPIO_0 for an
// external one (e.g. SN65HVD230).

module top (
    input  logic        CLOCK_50,
    input  logic [1:0]  KEY,

    output logic        ADC_CS_N,
    output logic        ADC_SCLK,
    output logic        ADC_SADDR,
    input  logic        ADC_SDAT,

    output logic        CAN_TX,
    input  logic        CAN_RX,

    output logic [7:0]  LED
);

    logic rst;
    assign rst = ~KEY[0];   // KEY[0] is active-low

    // 25 MHz CAN bit-timing clock; BTR values assume this rate.
    logic can_clk = 1'b0;
    always_ff @(posedge CLOCK_50) can_clk <= ~can_clk;

    logic [11:0] adc_ch [8];
    logic [2:0]  adc_last_channel;
    logic        adc_sample_valid;

    adc_spi_master #(.SCLK_DIV(16)) u_adc (
        .clk          (CLOCK_50),
        .rst          (rst),
        .adc_cs_n     (ADC_CS_N),
        .adc_sclk     (ADC_SCLK),
        .adc_saddr    (ADC_SADDR),
        .adc_sdat     (ADC_SDAT),
        .adc_data     (adc_ch),
        .last_channel (adc_last_channel),
        .sample_valid (adc_sample_valid)
    );

    logic [7:0] can_wb_dat_o;
    logic       can_wb_ack_o;
    logic       can_bus_off_on;
    logic       can_irq_on;
    logic       can_clkout;

    logic [7:0] host_wb_adr;
    logic [7:0] host_wb_dat;
    logic       host_wb_we;
    logic       host_wb_cyc;
    logic       host_wb_stb;
    logic       host_busy;
    logic       host_cfg_done;

    localparam int unsigned ONE_SEC = 50_000_000;

    logic [25:0] sec_cnt;
    logic        tx_pending;
    logic        tx_start;
    logic        tx_heartbeat;
    logic [15:0] msg_counter;

    localparam logic [28:0] CAN_ID = 29'h18FF_0001;

    logic [63:0] tx_data;
    assign tx_data = {adc_ch[0], adc_ch[1], adc_ch[2], adc_ch[3], msg_counter};

    // Latch the request and only fire when the host is idle, so a tick that
    // lands mid-transmission isn't lost.
    assign tx_start = tx_pending & host_cfg_done & ~host_busy;

    always_ff @(posedge CLOCK_50) begin
        if (rst) begin
            sec_cnt      <= '0;
            tx_pending   <= 1'b0;
            msg_counter  <= '0;
            tx_heartbeat <= 1'b0;
        end else begin
            if (sec_cnt == ONE_SEC-1) begin
                sec_cnt    <= '0;
                tx_pending <= 1'b1;
            end else begin
                sec_cnt <= sec_cnt + 1'b1;
            end

            if (tx_start) begin
                tx_pending   <= 1'b0;
                msg_counter  <= msg_counter + 1'b1;
                tx_heartbeat <= ~tx_heartbeat;
            end
        end
    end

    can_host_fsm u_can_host (
        .clk      (CLOCK_50),
        .rst      (rst),
        .tx_id    (CAN_ID),
        .tx_dlc   (4'd8),
        .tx_data  (tx_data),
        .tx_start (tx_start),
        .busy     (host_busy),
        .cfg_done (host_cfg_done),
        .wb_adr_o (host_wb_adr),
        .wb_dat_o (host_wb_dat),
        .wb_dat_i (can_wb_dat_o),
        .wb_we_o  (host_wb_we),
        .wb_cyc_o (host_wb_cyc),
        .wb_stb_o (host_wb_stb),
        .wb_ack_i (can_wb_ack_o)
    );

    // can_top is third-party IP (Verilog), left untouched.
    can_top u_can (
        .wb_clk_i  (CLOCK_50),
        .wb_rst_i  (rst),
        .wb_dat_i  (host_wb_dat),
        .wb_dat_o  (can_wb_dat_o),
        .wb_cyc_i  (host_wb_cyc),
        .wb_stb_i  (host_wb_stb),
        .wb_we_i   (host_wb_we),
        .wb_adr_i  (host_wb_adr),
        .wb_ack_o  (can_wb_ack_o),
        .clk_i     (can_clk),
        .rx_i      (CAN_RX),
        .tx_o      (CAN_TX),
        .bus_off_on(can_bus_off_on),
        .irq_on    (can_irq_on),
        .clkout_o  (can_clkout)
    );

    assign LED[7]   = can_bus_off_on;
    assign LED[6]   = ~can_irq_on;      // irq_on is active-low
    assign LED[5]   = host_cfg_done;
    assign LED[4]   = tx_heartbeat;
    assign LED[3:0] = adc_ch[0][11:8];

endmodule
