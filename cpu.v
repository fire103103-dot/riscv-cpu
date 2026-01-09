`include "opcode.vh"

module cpu #(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
) (
    input clk,
    input rst,
    input bp_enable,
    input serial_in,
    output serial_out
);

// ============================================
// Wires and Regs Declarations
// ============================================

// Memory interfaces
wire [11:0] bios_addra, bios_addrb;
wire [31:0] bios_douta, bios_doutb;
wire bios_ena, bios_enb;

wire [13:0] dmem_addr;
wire [31:0] dmem_din, dmem_dout;
wire [3:0] dmem_we;
wire dmem_en;

wire [31:0] imem_dina, imem_doutb;
wire [13:0] imem_addra, imem_addrb;
wire [3:0] imem_wea;
wire imem_ena;

// Register file interfaces
wire we;
wire [4:0] ra1, ra2, wa;
wire [31:0] wd;
wire [31:0] rd1, rd2;

// FP Register file interfaces (unused in Checkpoint 2)
wire [4:0] fp_ra1, fp_ra2, fp_ra3, fp_wa;
wire [31:0] fp_wd;
wire [31:0] fp_rd1, fp_rd2, fp_rd3;

// UART interfaces
wire [7:0] uart_rx_data_out;
wire uart_rx_data_out_valid;
wire uart_rx_data_out_ready;
wire [7:0] uart_tx_data_in;
wire uart_tx_data_in_valid;
wire uart_tx_data_in_ready;

// CSR
reg [31:0] tohost_csr;

// Program Counter
reg [31:0] PC;
wire [31:0] PC_plus_4 = PC + 32'd4;
wire [1:0] PCSel;
wire [31:0] branch_target, jump_target;
wire [31:0] next_PC;
wire stall;

// IF/ID Pipeline Registers
reg [31:0] IFID_PC, IFID_PC_plus_4;
reg IFID_valid;
reg IFID_was_imem, IFID_was_bios;
wire Flush;

// Instruction from pipeline - uses memory output directly
// With IMEM address gating during stall, this will hold correct instruction
wire [31:0] IFID_inst;

// Instruction decode
wire [6:0] opcode = IFID_inst[6:0];
wire [4:0] rd = IFID_inst[11:7];
wire [4:0] rs1 = IFID_inst[19:15];
wire [4:0] rs2 = IFID_inst[24:20];
wire [2:0] funct3 = IFID_inst[14:12];
wire [6:0] funct7 = IFID_inst[31:25];

// Control signals
reg RegWEn, MemRW, BSel, IsBranch, IsJump, IsLoad, IsStore, IsCSR;
reg [3:0] ALUSel;
reg [2:0] ImmSel, LoadOp;
reg [1:0] WBSel, StoreOp;

// EX/MEM signals
wire [4:0] EXMEM_rd;
wire EXMEM_IsLoad;
wire [31:0] EXMEM_alu_result;
wire EXMEM_RegWEn;

// Immediate value
reg [31:0] imm;

// Forwarding control
reg [1:0] FwdASel, FwdBSel, FwdStoreSel;

// ALU and data paths
wire [31:0] alu_in_a, alu_in_b, store_data_fwd, final_wb_data;
reg [31:0] alu_result;
wire [31:0] alu_in_a_final = (opcode == `OPC_AUIPC) ? IFID_PC : (opcode == `OPC_LUI) ? 32'b0 : alu_in_a;

// Branch comparison
reg branch_comp_result;
wire branch_taken = IsBranch && branch_comp_result;

// EX/MEM Pipeline Registers
reg [31:0] EXMEM_alu_result_reg, EXMEM_store_data, EXMEM_PC_plus_4_reg, EXMEM_csr_wdata;
reg [11:0] EXMEM_csr_addr;
reg [4:0] EXMEM_rd_reg;
reg [2:0] EXMEM_LoadOp;
reg [1:0] EXMEM_StoreOp, EXMEM_WBSel;
reg EXMEM_PC_30, EXMEM_valid, EXMEM_RegWEn_reg, EXMEM_MemRW;
reg EXMEM_IsLoad_reg, EXMEM_IsStore, EXMEM_IsCSR;
reg EXMEM_IsBranch_reg, EXMEM_branch_was_taken_reg;

// MEM/WB Pipeline Registers
reg [31:0] MEMWB_alu_result_reg, MEMWB_PC_plus_4_reg, MEMWB_mem_addr_reg;
reg [4:0] MEMWB_rd_reg;
reg MEMWB_RegWEn_reg;
reg [1:0] MEMWB_WBSel_reg;
reg [2:0] MEMWB_LoadOp_reg;
reg MEMWB_valid;
reg [31:0] MEMWB_mmio_data;

// ============================================
// Module Instantiations
// ============================================

bios_mem bios_mem (
    .clk(clk),
    .ena(bios_ena),
    .addra(bios_addra),
    .douta(bios_douta),
    .enb(bios_enb),
    .addrb(bios_addrb),
    .doutb(bios_doutb)
);

dmem dmem (
    .clk(clk),
    .en(dmem_en),
    .we(dmem_we),
    .addr(dmem_addr),
    .din(dmem_din),
    .dout(dmem_dout)
);

imem imem (
    .clk(clk),
    .ena(imem_ena),
    .wea(imem_wea),
    .addra(imem_addra),
    .dina(imem_dina),
    .addrb(imem_addrb),
    .doutb(imem_doutb)
);

reg_file rf (
    .clk(clk),
    .we(we),
    .ra1(ra1),
    .ra2(ra2),
    .wa(wa),
    .wd(wd),
    .rd1(rd1),
    .rd2(rd2)
);

// Tie-off unused FPU signals for Checkpoint 2
assign fp_ra1 = 5'b0;
assign fp_ra2 = 5'b0;
assign fp_ra3 = 5'b0;
assign fp_wa = 5'b0;
assign fp_wd = 32'b0;
wire fp_we_dummy = 1'b0;

fp_reg_file fprf (
    .clk(clk),
    .we(fp_we_dummy),
    .ra1(fp_ra1),
    .ra2(fp_ra2),
    .ra3(fp_ra3),
    .wa(fp_wa),
    .wd(fp_wd),
    .rd1(fp_rd1),
    .rd2(fp_rd2),
    .rd3(fp_rd3)
);

uart #(
    .CLOCK_FREQ(CPU_CLOCK_FREQ),
    .BAUD_RATE(BAUD_RATE)
) on_chip_uart (
    .clk(clk),
    .reset(rst),
    .serial_in(serial_in),
    .data_out(uart_rx_data_out),
    .data_out_valid(uart_rx_data_out_valid),
    .data_out_ready(uart_rx_data_out_ready),
    .serial_out(serial_out),
    .data_in(uart_tx_data_in),
    .data_in_valid(uart_tx_data_in_valid),
    .data_in_ready(uart_tx_data_in_ready)
);

// ============================================
// Pipeline Stage 1: IF (Instruction Fetch)
// ============================================

// Instruction Source Detection (Current Cycle)
wire is_imem = (PC[31:28] == 4'h1);
wire is_bios = (PC[31:28] == 4'h4);

// Next PC Selection
assign next_PC = (PCSel == 2'd1) ? branch_target :
                 (PCSel == 2'd2) ? jump_target : PC_plus_4;

// PC Update
always @(posedge clk) begin
    if (rst)
        PC <= RESET_PC;
    else if (!stall)
        PC <= next_PC;
end

// Memory address for instruction fetch
// CRITICAL: During stall, keep fetching the same instruction by using IFID_PC
// This prevents the IMEM from advancing to the next instruction
assign imem_addrb = stall ? IFID_PC[15:2] : PC[15:2];
assign bios_ena = is_bios;
assign bios_addra = stall ? IFID_PC[13:2] : PC[13:2];

// IF/ID Pipeline Register Update
always @(posedge clk) begin
    if (rst) begin
        IFID_valid <= 1'b0;
        IFID_PC <= 32'b0;
        IFID_PC_plus_4 <= 32'b0;
        IFID_was_imem <= 1'b0;
        IFID_was_bios <= 1'b0;
    end
    else if (stall) begin
        // Freeze: Keep all values as is
        // Note: IMEM address is also gated during stall, so imem_doutb
        // will continue to output the correct instruction
    end
    else if (Flush) begin
        IFID_valid <= 1'b0;
    end
    else begin
        IFID_valid <= 1'b1;
        IFID_PC <= PC;
        IFID_PC_plus_4 <= PC_plus_4;
        IFID_was_imem <= is_imem;
        IFID_was_bios <= is_bios;
    end
end

// Instruction Mux (Using Pipelined Source Info)
wire [31:0] raw_inst = IFID_was_imem ? imem_doutb :
                       IFID_was_bios ? bios_douta : 32'h0000_0013;
assign IFID_inst = IFID_valid ? raw_inst : 32'h0000_0013;

// ============================================
// Pipeline Stage 2: ID/EX (Decode & Execute)
// ============================================

// Register file read addresses
assign ra1 = rs1;
assign ra2 = rs2;

// Wire assignments for hazard detection
assign EXMEM_rd = EXMEM_rd_reg;
assign EXMEM_IsLoad = EXMEM_IsLoad_reg;

// Stall Condition: Load-Use Hazard Detection
assign stall = IFID_valid && EXMEM_IsLoad && (EXMEM_rd != 5'b0) &&
               ((EXMEM_rd == rs1) || (EXMEM_rd == rs2));

// Control Unit
always @(*) begin
    // Default values
    RegWEn = 1'b0;
    MemRW = 1'b0;
    ALUSel = 4'b0;
    BSel = 1'b0;
    ImmSel = 3'b0;
    WBSel = 2'b0;
    IsBranch = 1'b0;
    IsJump = 1'b0;
    IsLoad = 1'b0;
    IsStore = 1'b0;
    LoadOp = 3'b0;
    StoreOp = 2'b0;
    IsCSR = 1'b0;

    case (opcode)
        `OPC_LUI: begin
            RegWEn = 1'b1;
            ImmSel = 3'd4;
            BSel = 1'b1;
            ALUSel = 4'b0;  // ADD (0 + imm)
        end

        `OPC_AUIPC: begin
            RegWEn = 1'b1;
            ImmSel = 3'd4;
            BSel = 1'b1;
            ALUSel = 4'b0;  // ADD (PC + imm)
        end

        `OPC_JAL: begin
            RegWEn = 1'b1;
            ImmSel = 3'd3;
            IsJump = 1'b1;
            WBSel = 2'd2;   // PC + 4
        end

        `OPC_JALR: begin
            RegWEn = 1'b1;
            BSel = 1'b1;
            IsJump = 1'b1;
            WBSel = 2'd2;   // PC + 4
        end

        `OPC_BRANCH: begin
            ImmSel = 3'd2;
            IsBranch = 1'b1;
        end

        `OPC_LOAD: begin
            RegWEn = 1'b1;
            BSel = 1'b1;
            WBSel = 2'd1;   // Memory data
            IsLoad = 1'b1;
            LoadOp = funct3;
        end

        `OPC_STORE: begin
            BSel = 1'b1;
            MemRW = 1'b1;
            IsStore = 1'b1;
            StoreOp = funct3[1:0];
            ImmSel = 3'd1;
        end

        `OPC_ARI_ITYPE: begin
            RegWEn = 1'b1;
            BSel = 1'b1;
            case (funct3)
                `FNC_ADD_SUB: ALUSel = 4'd0;
                `FNC_SLT:     ALUSel = 4'd2;
                `FNC_SLTU:    ALUSel = 4'd3;
                `FNC_XOR:     ALUSel = 4'd4;
                `FNC_OR:      ALUSel = 4'd6;
                `FNC_AND:     ALUSel = 4'd7;
                `FNC_SLL:     ALUSel = 4'd9;
                `FNC_SRL_SRA: ALUSel = funct7[5] ? 4'd11 : 4'd10;
                default:      ALUSel = 4'd0;
            endcase
        end

        `OPC_ARI_RTYPE: begin
            RegWEn = 1'b1;
            case (funct3)
                `FNC_ADD_SUB: ALUSel = funct7[5] ? 4'd1 : 4'd0;
                `FNC_SLL:     ALUSel = 4'd9;
                `FNC_SLT:     ALUSel = 4'd2;
                `FNC_SLTU:    ALUSel = 4'd3;
                `FNC_XOR:     ALUSel = 4'd4;
                `FNC_SRL_SRA: ALUSel = funct7[5] ? 4'd11 : 4'd10;
                `FNC_OR:      ALUSel = 4'd6;
                `FNC_AND:     ALUSel = 4'd7;
                default:      ALUSel = 4'd0;
            endcase
        end

        `OPC_CSR: begin
            IsCSR = 1'b1;
            RegWEn = 1'b1;
        end

        default: begin
            // NOP - all signals already at default
        end
    endcase
end

// Immediate Generator
always @(*) begin
    case (ImmSel)
        3'd0: imm = {{20{IFID_inst[31]}}, IFID_inst[31:20]};  // I-type
        3'd1: imm = {{20{IFID_inst[31]}}, IFID_inst[31:25], IFID_inst[11:7]};  // S-type
        3'd2: imm = {{19{IFID_inst[31]}}, IFID_inst[31], IFID_inst[7], 
                     IFID_inst[30:25], IFID_inst[11:8], 1'b0};  // B-type
        3'd3: imm = {{11{IFID_inst[31]}}, IFID_inst[31], IFID_inst[19:12],
                     IFID_inst[20], IFID_inst[30:21], 1'b0};  // J-type
        3'd4: imm = {IFID_inst[31:12], 12'b0};  // U-type
        default: imm = 32'b0;
    endcase
end

// Forwarding Unit
// CRITICAL: EX/MEM forwarding must NOT be used for Load instructions!
// When Load is in EX/MEM, EXMEM_alu_result contains the memory ADDRESS, not data.
// Load data is only available in MEM/WB stage after memory read completes.
always @(*) begin
    // Forward A (rs1)
    // For Load in EX/MEM: stall will occur, then use MEM/WB forwarding
    if (EXMEM_RegWEn && !EXMEM_IsLoad && (EXMEM_rd != 5'b0) && (EXMEM_rd == rs1))
        FwdASel = 2'b01;  // Forward from EX/MEM (ALU results only, NOT loads)
    else if (MEMWB_valid && MEMWB_RegWEn_reg && (MEMWB_rd_reg != 5'b0) && (MEMWB_rd_reg == rs1))
        FwdASel = 2'b10;  // Forward from MEM/WB (includes load data)
    else
        FwdASel = 2'b00;  // No forwarding

    // Forward B (rs2 or immediate)
    if (BSel)
        FwdBSel = 2'b01;  // Use immediate
    else if (EXMEM_RegWEn && !EXMEM_IsLoad && (EXMEM_rd != 5'b0) && (EXMEM_rd == rs2))
        FwdBSel = 2'b10;  // Forward from EX/MEM (ALU results only, NOT loads)
    else if (MEMWB_valid && MEMWB_RegWEn_reg && (MEMWB_rd_reg != 5'b0) && (MEMWB_rd_reg == rs2))
        FwdBSel = 2'b11;  // Forward from MEM/WB (includes load data)
    else
        FwdBSel = 2'b00;  // No forwarding

    // Forward Store Data (rs2)
    if (EXMEM_RegWEn && !EXMEM_IsLoad && (EXMEM_rd != 5'b0) && (EXMEM_rd == rs2))
        FwdStoreSel = 2'b10;  // Forward from EX/MEM (ALU results only, NOT loads)
    else if (MEMWB_valid && MEMWB_RegWEn_reg && (MEMWB_rd_reg != 5'b0) && (MEMWB_rd_reg == rs2))
        FwdStoreSel = 2'b11;  // Forward from MEM/WB (includes load data)
    else
        FwdStoreSel = 2'b00;  // No forwarding
end

// ALU Input Muxes
assign alu_in_a = (FwdASel == 2'b01) ? EXMEM_alu_result :
                  (FwdASel == 2'b10) ? final_wb_data : rd1;

assign alu_in_b = (FwdBSel == 2'b01) ? imm :
                  (FwdBSel == 2'b10) ? EXMEM_alu_result :
                  (FwdBSel == 2'b11) ? final_wb_data : rd2;

assign store_data_fwd = (FwdStoreSel == 2'b11) ? final_wb_data :
                        (FwdStoreSel == 2'b10) ? EXMEM_alu_result : rd2;

// ALU
always @(*) begin
    case (ALUSel)
        4'd0:  alu_result = alu_in_a_final + alu_in_b;                           // ADD
        4'd1:  alu_result = alu_in_a_final - alu_in_b;                           // SUB
        4'd2:  alu_result = ($signed(alu_in_a_final) < $signed(alu_in_b)) ? 32'd1 : 32'd0;  // SLT
        4'd3:  alu_result = (alu_in_a_final < alu_in_b) ? 32'd1 : 32'd0;         // SLTU
        4'd4:  alu_result = alu_in_a_final ^ alu_in_b;                           // XOR
        4'd6:  alu_result = alu_in_a_final | alu_in_b;                           // OR
        4'd7:  alu_result = alu_in_a_final & alu_in_b;                           // AND
        4'd9:  alu_result = alu_in_a_final << alu_in_b[4:0];                     // SLL
        4'd10: alu_result = alu_in_a_final >> alu_in_b[4:0];                     // SRL
        4'd11: alu_result = $signed(alu_in_a_final) >>> alu_in_b[4:0];           // SRA
        default: alu_result = 32'b0;
    endcase
end

// Branch Comparator
always @(*) begin
    case (funct3)
        `FNC_BEQ:  branch_comp_result = (alu_in_a == alu_in_b);
        `FNC_BNE:  branch_comp_result = (alu_in_a != alu_in_b);
        `FNC_BLT:  branch_comp_result = ($signed(alu_in_a) < $signed(alu_in_b));
        `FNC_BGE:  branch_comp_result = ($signed(alu_in_a) >= $signed(alu_in_b));
        `FNC_BLTU: branch_comp_result = (alu_in_a < alu_in_b);
        `FNC_BGEU: branch_comp_result = (alu_in_a >= alu_in_b);
        default:   branch_comp_result = 1'b0;
    endcase
end

// Branch and Jump Target Calculation
assign branch_target = IFID_PC + imm;
wire [31:0] jalr_target = (alu_result & 32'hFFFF_FFFE);
assign jump_target = (opcode == `OPC_JALR) ? jalr_target : (IFID_PC + imm);

// PC Select and Flush Control
assign PCSel = branch_taken ? 2'd1 : (IsJump ? 2'd2 : 2'd0);
assign Flush = branch_taken || IsJump;

// EX/MEM Pipeline Register Update
// NOTE: Flush should NOT invalidate EXMEM - the JAL/Branch instruction itself
// must complete (writeback PC+4 for JAL/JALR). Flush only invalidates IF/ID
// (the wrongly-fetched next instruction).
always @(posedge clk) begin
    if (rst || stall) begin
        EXMEM_valid <= 1'b0;
        EXMEM_RegWEn_reg <= 1'b0;
        EXMEM_rd_reg <= 5'b0;
        EXMEM_MemRW <= 1'b0;
    end
    else begin
        EXMEM_alu_result_reg <= alu_result;
        EXMEM_store_data <= store_data_fwd;
        EXMEM_rd_reg <= rd;
        EXMEM_PC_plus_4_reg <= IFID_PC_plus_4;
        EXMEM_PC_30 <= IFID_PC[30];
        EXMEM_valid <= IFID_valid;
        EXMEM_RegWEn_reg <= RegWEn;
        EXMEM_MemRW <= MemRW;
        EXMEM_WBSel <= WBSel;
        EXMEM_IsLoad_reg <= IsLoad;
        EXMEM_IsStore <= IsStore;
        EXMEM_LoadOp <= LoadOp;
        EXMEM_StoreOp <= StoreOp;
        EXMEM_IsCSR <= IsCSR;
        EXMEM_csr_addr <= IFID_inst[31:20];
        EXMEM_csr_wdata <= (funct3[2] && funct3[0]) ? {27'b0, rs1} : alu_in_a;
        EXMEM_IsBranch_reg <= IsBranch;
        EXMEM_branch_was_taken_reg <= branch_taken;
    end
end

// ============================================
// Pipeline Stage 3: MEM (Memory Access)
// ============================================

assign EXMEM_alu_result = EXMEM_alu_result_reg;
assign EXMEM_RegWEn = EXMEM_RegWEn_reg;

// MMIO data (forward declaration for pipelining)
wire [31:0] mmio_dout;

// MEM/WB Pipeline Register Update
always @(posedge clk) begin
    if (rst) begin
        MEMWB_valid <= 1'b0;
        MEMWB_RegWEn_reg <= 1'b0;
        MEMWB_rd_reg <= 5'b0;
    end
    else begin
        MEMWB_valid <= EXMEM_valid;
        MEMWB_alu_result_reg <= EXMEM_alu_result_reg;
        MEMWB_PC_plus_4_reg <= EXMEM_PC_plus_4_reg;
        MEMWB_rd_reg <= EXMEM_rd_reg;
        MEMWB_RegWEn_reg <= EXMEM_RegWEn_reg;
        MEMWB_WBSel_reg <= EXMEM_WBSel;
        MEMWB_LoadOp_reg <= EXMEM_LoadOp;
        MEMWB_mem_addr_reg <= EXMEM_alu_result_reg;
        MEMWB_mmio_data <= mmio_dout;
    end
end

// Memory Region Decode
wire [3:0] mem_region = EXMEM_alu_result_reg[31:28];
wire access_dmem = (mem_region[3:1] == 3'b000) || (mem_region[3:1] == 3'b001);
wire access_bios = (mem_region == 4'h4);
wire access_mmio = (mem_region == 4'h8);

// Store Data Alignment and Write Enable Mask
reg [3:0] store_we_mask;
reg [31:0] store_data_aligned;

always @(*) begin
    store_we_mask = 4'b0000;
    store_data_aligned = 32'b0;
    
    case (EXMEM_StoreOp)
        2'b00: begin  // SB
            case (EXMEM_alu_result_reg[1:0])
                2'b00: begin
                    store_we_mask = 4'b0001;
                    store_data_aligned = {24'b0, EXMEM_store_data[7:0]};
                end
                2'b01: begin
                    store_we_mask = 4'b0010;
                    store_data_aligned = {16'b0, EXMEM_store_data[7:0], 8'b0};
                end
                2'b10: begin
                    store_we_mask = 4'b0100;
                    store_data_aligned = {8'b0, EXMEM_store_data[7:0], 16'b0};
                end
                2'b11: begin
                    store_we_mask = 4'b1000;
                    store_data_aligned = {EXMEM_store_data[7:0], 24'b0};
                end
            endcase
        end
        2'b01: begin  // SH
            if (EXMEM_alu_result_reg[1]) begin
                store_we_mask = 4'b1100;
                store_data_aligned = {EXMEM_store_data[15:0], 16'b0};
            end
            else begin
                store_we_mask = 4'b0011;
                store_data_aligned = {16'b0, EXMEM_store_data[15:0]};
            end
        end
        2'b10: begin  // SW
            store_we_mask = 4'b1111;
            store_data_aligned = EXMEM_store_data;
        end
        default: begin
            store_we_mask = 4'b0000;
            store_data_aligned = 32'b0;
        end
    endcase
end

// DMEM Interface
assign dmem_en = access_dmem && EXMEM_valid;
assign dmem_addr = EXMEM_alu_result_reg[15:2];
assign dmem_we = (EXMEM_MemRW && access_dmem && EXMEM_valid) ? store_we_mask : 4'b0;
assign dmem_din = store_data_aligned;

// IMEM Write (only allowed when executing from BIOS)
wire imem_write_allowed = EXMEM_PC_30;
assign imem_ena = ((mem_region == 4'h1) || (mem_region == 4'h3)) && EXMEM_MemRW && imem_write_allowed && EXMEM_valid;
assign imem_addra = EXMEM_alu_result_reg[15:2];
assign imem_dina = store_data_aligned;
assign imem_wea = imem_ena ? store_we_mask : 4'b0;

// BIOS Read (Port B for data access)
assign bios_enb = access_bios && !EXMEM_MemRW;
assign bios_addrb = EXMEM_alu_result_reg[13:2];

// ============================================
// Pipeline Stage 4: WB (Write Back)
// ============================================

// WB Stage Memory Region Decode (using pipelined address)
wire [3:0] wb_mem_region = MEMWB_mem_addr_reg[31:28];
wire wb_access_dmem = (wb_mem_region[3:1] == 3'b000) || (wb_mem_region[3:1] == 3'b001);
wire wb_access_bios = (wb_mem_region == 4'h4);
wire wb_access_mmio = (wb_mem_region == 4'h8);

// Load Data Selection
wire [31:0] load_data_raw = wb_access_dmem ? dmem_dout :
                            wb_access_bios ? bios_doutb :
                            wb_access_mmio ? MEMWB_mmio_data : 32'b0;

// Load Data Extension
reg [31:0] final_load_data_extended;

always @(*) begin
    final_load_data_extended = 32'b0;
    
    case (MEMWB_LoadOp_reg)
        `FNC_LB: begin
            case (MEMWB_mem_addr_reg[1:0])
                2'b00: final_load_data_extended = {{24{load_data_raw[7]}}, load_data_raw[7:0]};
                2'b01: final_load_data_extended = {{24{load_data_raw[15]}}, load_data_raw[15:8]};
                2'b10: final_load_data_extended = {{24{load_data_raw[23]}}, load_data_raw[23:16]};
                2'b11: final_load_data_extended = {{24{load_data_raw[31]}}, load_data_raw[31:24]};
            endcase
        end
        
        `FNC_LH: begin
            if (MEMWB_mem_addr_reg[1])
                final_load_data_extended = {{16{load_data_raw[31]}}, load_data_raw[31:16]};
            else
                final_load_data_extended = {{16{load_data_raw[15]}}, load_data_raw[15:0]};
        end
        
        `FNC_LW: begin
            final_load_data_extended = load_data_raw;
        end
        
        `FNC_LBU: begin
            case (MEMWB_mem_addr_reg[1:0])
                2'b00: final_load_data_extended = {24'b0, load_data_raw[7:0]};
                2'b01: final_load_data_extended = {24'b0, load_data_raw[15:8]};
                2'b10: final_load_data_extended = {24'b0, load_data_raw[23:16]};
                2'b11: final_load_data_extended = {24'b0, load_data_raw[31:24]};
            endcase
        end
        
        `FNC_LHU: begin
            if (MEMWB_mem_addr_reg[1])
                final_load_data_extended = {16'b0, load_data_raw[31:16]};
            else
                final_load_data_extended = {16'b0, load_data_raw[15:0]};
        end
        
        default: begin
            final_load_data_extended = load_data_raw;
        end
    endcase
end

// Write Back Data Selection
assign final_wb_data = (MEMWB_WBSel_reg == 2'd1) ? final_load_data_extended :
                       (MEMWB_WBSel_reg == 2'd2) ? MEMWB_PC_plus_4_reg :
                       MEMWB_alu_result_reg;

// Register File Write
assign we = MEMWB_RegWEn_reg && MEMWB_valid;
assign wa = MEMWB_rd_reg;
assign wd = final_wb_data;

// ============================================
// MMIO (Memory Mapped I/O)
// ============================================

wire [31:0] mmio_addr = EXMEM_alu_result_reg;

// MMIO Address Decode
wire mmio_uart_ctrl = (mmio_addr == 32'h8000_0000);
wire mmio_uart_rx   = (mmio_addr == 32'h8000_0004);
wire mmio_uart_tx   = (mmio_addr == 32'h8000_0008);
wire mmio_cycle_ctr = (mmio_addr == 32'h8000_0010);
wire mmio_inst_ctr  = (mmio_addr == 32'h8000_0014);
wire mmio_counter_reset = (mmio_addr == 32'h8000_0018);
wire mmio_branch_ctr = (mmio_addr == 32'h8000_001c);
wire mmio_branch_correct_ctr = (mmio_addr == 32'h8000_0020);

// Performance Counters
reg [31:0] cycle_counter;
reg [31:0] inst_counter;
reg [31:0] branch_counter;
reg [31:0] branch_correct_counter;

wire counter_reset = EXMEM_MemRW && mmio_counter_reset && EXMEM_valid && access_mmio;

always @(posedge clk) begin
    if (rst || counter_reset) begin
        cycle_counter <= 32'b0;
        inst_counter <= 32'b0;
        branch_counter <= 32'b0;
        branch_correct_counter <= 32'b0;
    end
    else begin
        cycle_counter <= cycle_counter + 32'd1;
        if (EXMEM_valid) begin
            inst_counter <= inst_counter + 32'd1;
            if (EXMEM_IsBranch_reg) begin
                branch_counter <= branch_counter + 32'd1;
                if (!EXMEM_branch_was_taken_reg)
                    branch_correct_counter <= branch_correct_counter + 32'd1;
            end
        end
    end
end

// MMIO Read Data
assign mmio_dout = mmio_uart_rx ? {24'b0, uart_rx_data_out} :
                   mmio_cycle_ctr ? cycle_counter :
                   mmio_inst_ctr ? inst_counter :
                   mmio_branch_ctr ? branch_counter :
                   mmio_branch_correct_ctr ? branch_correct_counter :
                   mmio_uart_ctrl ? {30'b0, uart_rx_data_out_valid, uart_tx_data_in_ready} :
                   32'b0;

// UART Interface
assign uart_tx_data_in = EXMEM_store_data[7:0];
assign uart_tx_data_in_valid = EXMEM_MemRW && mmio_uart_tx && EXMEM_valid && access_mmio;
assign uart_rx_data_out_ready = !EXMEM_MemRW && mmio_uart_rx && EXMEM_valid && access_mmio;

// ============================================
// CSR (Control and Status Register)
// ============================================

always @(posedge clk) begin
    if (rst)
        tohost_csr <= 32'b0;
    else if (EXMEM_IsCSR && EXMEM_valid && (EXMEM_csr_addr == 12'h51e))
        tohost_csr <= EXMEM_csr_wdata;
end

// ============================================
// Assertions (SystemVerilog)
// ============================================

// Store write enable mask assertion
always @(posedge clk) begin
    if (!rst && EXMEM_IsStore && EXMEM_valid) begin
        case (EXMEM_StoreOp)
            2'b00: assert ($countones(store_we_mask) == 1) else $error("SB mask error");
            2'b01: assert ($countones(store_we_mask) == 2) else $error("SH mask error");
            2'b10: assert ($countones(store_we_mask) == 4) else $error("SW mask error");
            default: ;
        endcase
    end
end

// Load sign extension assertion
always @(posedge clk) begin
    if (!rst && MEMWB_RegWEn_reg && (MEMWB_WBSel_reg == 2'd1)) begin
        case (MEMWB_LoadOp_reg)
            `FNC_LB: begin
                assert (final_load_data_extended[31:8] == 24'h0 ||
                        final_load_data_extended[31:8] == 24'hffffff)
                else $error("LB sign extension error");
            end
            `FNC_LH: begin
                assert (final_load_data_extended[31:16] == 16'h0 ||
                        final_load_data_extended[31:16] == 16'hffff)
                else $error("LH sign extension error");
            end
            default: ;
        endcase
    end
end

// Register x0 assertion
always @(posedge clk) begin
    if (!rst) begin
        if (ra1 == 5'b0)
            assert (rd1 == 32'b0) else $error("x0 read error on port 1");
        if (ra2 == 5'b0)
            assert (rd2 == 32'b0) else $error("x0 read error on port 2");
    end
end
endmodule
